// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "navigation_neupan_bridge/static_map_overlap_state.hpp"
#include "navigation_neupan_bridge/static_map_fault_geometry.hpp"

namespace navigation_neupan_bridge
{
namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

double normalizeAngle(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct OdomSample
{
  Pose2D pose;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  SteadyTime received{};
  bool available{false};
};

struct LethalFootprintSample
{
  uint32_t count{0};
  double map_x{0.0};
  double map_y{0.0};
  double base_x{0.0};
  double base_y{0.0};
};

struct DynamicFootprint
{
  std::vector<double> points;
  SteadyTime received{};
  bool available{false};
};

struct PublishedFootprint
{
  std::vector<double> points;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  SteadyTime received{};
  bool available{false};
};

bool dynamicFootprintMatchesPublished(
  const DynamicFootprint & dynamic,
  const PublishedFootprint & published, const Pose2D & map_pose, const double tolerance)
{
  // Vertex counts are NOT required to match. The arm hull is a convex hull of
  // disc samples, so its vertex count flickers with sub-millimetre joint noise
  // between the publisher, the costmap's stored polygon, and the re-published
  // oriented footprint. The comparison is therefore point-to-edge: every
  // vertex must sit within the tolerance of the other polygon's boundary.
  // A near-collinear vertex flipping in or out stays on the counterpart's
  // edge and passes; a genuinely displaced footprint still fails because its
  // vertices leave the counterpart boundary entirely.
  if (dynamic.points.size() < 6 || published.points.size() < 6) {
    return false;
  }
  const double cosine = std::cos(map_pose.yaw);
  const double sine = std::sin(map_pose.yaw);
  const auto hasNearbyEdge = [tolerance](const double x, const double y,
      const std::vector<double> & polygon) {
      const size_t vertex_count = polygon.size() / 2;
      for (size_t index = 0; index < vertex_count; ++index) {
        const size_t next = (index + 1) % vertex_count;
        const double ax = polygon[index * 2];
        const double ay = polygon[index * 2 + 1];
        const double bx = polygon[next * 2];
        const double by = polygon[next * 2 + 1];
        const double edge_x = bx - ax;
        const double edge_y = by - ay;
        const double length_sq = edge_x * edge_x + edge_y * edge_y;
        double u = 0.0;
        if (length_sq > 1.0e-12) {
          u = ((x - ax) * edge_x + (y - ay) * edge_y) / length_sq;
          u = std::max(0.0, std::min(1.0, u));
        }
        const double cx = ax + u * edge_x;
        const double cy = ay + u * edge_y;
        if (std::hypot(x - cx, y - cy) <= tolerance) {
          return true;
        }
      }
      return false;
    };
  for (size_t index = 0; index < dynamic.points.size(); index += 2) {
    const double map_x = map_pose.x + cosine * dynamic.points[index] - sine *
      dynamic.points[index + 1];
    const double map_y = map_pose.y + sine * dynamic.points[index] + cosine *
      dynamic.points[index + 1];
    if (!hasNearbyEdge(map_x, map_y, published.points)) {
      return false;
    }
  }
  for (size_t index = 0; index < published.points.size(); index += 2) {
    const double base_x = cosine * (published.points[index] - map_pose.x) +
      sine * (published.points[index + 1] - map_pose.y);
    const double base_y = -sine * (published.points[index] - map_pose.x) +
      cosine * (published.points[index + 1] - map_pose.y);
    if (!hasNearbyEdge(base_x, base_y, dynamic.points)) {
      return false;
    }
  }
  return true;
}
}  // namespace

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor()
  : Node("r1_localization_monitor"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto localization_topic = declare_parameter<std::string>(
      "localization_status_topic", "/indoor_location/status");
    const auto initial_pose_topic = declare_parameter<std::string>(
      "initial_pose_topic",
      "/initialpose");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/r1_localization_health");
    costmap_service_ = declare_parameter<std::string>(
      "costmap_service", "/global_costmap/get_costmap");
    // R1 localization publishes map -> base_link at approximately 1 Hz.  The
    // freshness budget must span a complete publication period, otherwise a
    // healthy localization stream is falsely faulted between TF updates.
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 1.5);
    // The collision guard independently validates raw odom at 0.15 s. This
    // monitor observes a ROS callback stream and must tolerate a bounded
    // executor scheduling pause without latching a false localization fault.
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.5);
    localization_timeout_sec_ = declare_parameter<double>("localization_timeout_sec", 1.50);
    localization_fault_timeout_sec_ = declare_parameter<double>(
      "localization_fault_timeout_sec", 3.00);
    stable_window_sec_ = declare_parameter<double>("stable_window_sec", 3.0);
    static_map_overlap_state_ = StaticMapOverlapState(stable_window_sec_);
    soft_translation_jump_m_ = declare_parameter<double>("soft_translation_jump_m", 0.08);
    hard_translation_jump_m_ = declare_parameter<double>("hard_translation_jump_m", 0.20);
    soft_yaw_jump_rad_ = declare_parameter<double>("soft_yaw_jump_rad", 0.0872665);
    hard_yaw_jump_rad_ = declare_parameter<double>("hard_yaw_jump_rad", 0.2094395);
    footprint_padding_m_ = declare_parameter<double>("footprint_padding_m", 0.01);
    static_map_fault_inset_m_ = declare_parameter<double>("static_map_fault_inset_m", 0.0);
    footprint_ = declare_parameter<std::vector<double>>(
      "footprint_points", std::vector<double>{-0.341, -0.266, -0.341, 0.266, 0.341, 0.266, 0.341,
        -0.266});
    require_dynamic_footprint_ = declare_parameter<bool>("require_dynamic_footprint", false);
    dynamic_footprint_timeout_sec_ = declare_parameter<double>(
      "dynamic_footprint_timeout_sec", 0.50);
    const auto dynamic_footprint_topic = declare_parameter<std::string>(
      "dynamic_footprint_topic", "/r1_arm_footprint_planner");
    costmap_footprint_timeout_sec_ = declare_parameter<double>(
      "costmap_footprint_timeout_sec", 2.50);
    footprint_sync_tolerance_m_ = declare_parameter<double>(
      "footprint_sync_tolerance_m", 0.03);
    const auto costmap_footprint_topic = declare_parameter<std::string>(
      "costmap_footprint_topic", "/global_costmap/published_footprint");
    if (footprint_.size() < 6 || footprint_.size() % 2 != 0 || tf_timeout_sec_ <= 0.0 ||
      odom_timeout_sec_ <= 0.0 || localization_timeout_sec_ <= 0.0 ||
      localization_fault_timeout_sec_ < localization_timeout_sec_ || stable_window_sec_ <= 0.0 ||
      dynamic_footprint_timeout_sec_ <= 0.0 || costmap_footprint_timeout_sec_ <= 0.0 ||
      footprint_sync_tolerance_m_ <= 0.0 || static_map_fault_inset_m_ < 0.0)
    {
      throw std::invalid_argument("invalid localization monitor configuration");
    }

    input_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions input_subscription_options;
    input_subscription_options.callback_group = input_callback_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50),
      std::bind(&LocalizationMonitor::odomCallback, this, std::placeholders::_1),
      input_subscription_options);
    localization_sub_ = create_subscription<std_msgs::msg::String>(
      localization_topic, rclcpp::QoS(20),
      std::bind(&LocalizationMonitor::localizationCallback, this, std::placeholders::_1),
      input_subscription_options);
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic, rclcpp::QoS(10),
      std::bind(&LocalizationMonitor::initialPoseCallback, this, std::placeholders::_1));
    dynamic_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      dynamic_footprint_topic, rclcpp::QoS(20),
      std::bind(&LocalizationMonitor::dynamicFootprintCallback, this, std::placeholders::_1),
      input_subscription_options);
    costmap_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      costmap_footprint_topic, rclcpp::QoS(20),
      std::bind(&LocalizationMonitor::costmapFootprintCallback, this, std::placeholders::_1),
      input_subscription_options);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(20));
    const auto static_map_topic = declare_parameter<std::string>("static_map_topic", "/map");
    rclcpp::QoS map_qos(1);
    map_qos.reliable().transient_local();
    static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_map_topic, map_qos,
      std::bind(&LocalizationMonitor::staticMapCallback, this, std::placeholders::_1));
    costmap_client_ = create_client<nav2_msgs::srv::GetCostmap>(costmap_service_);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/r1_localization_monitor/reset",
      std::bind(
        &LocalizationMonitor::resetCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(100), std::bind(
        &LocalizationMonitor::tick,
        this));
  }

private:
  void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    static_map_ = *message;
    static_map_available_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    tf2::Quaternion quaternion;
    tf2::fromMsg(message->pose.pose.orientation, quaternion);
    std::lock_guard<std::mutex> lock(mutex_);
    OdomSample sample;
    sample.pose = {message->pose.pose.position.x, message->pose.pose.position.y, tf2::getYaw(
        quaternion)};
    sample.stamp = message->header.stamp;
    sample.received = std::chrono::steady_clock::now();
    sample.available = true;
    odom_ = sample;
    odom_history_.push_back(sample);
    while (odom_history_.size() > 200) {
      odom_history_.pop_front();
    }
  }

  bool odomAt(
    const std::deque<OdomSample> & history, const rclcpp::Time & stamp,
    OdomSample & result) const
  {
    if (history.empty() || stamp.nanoseconds() == 0) {
      return false;
    }
    const auto upper = std::lower_bound(
      history.begin(), history.end(), stamp,
      [](const OdomSample & sample, const rclcpp::Time & requested) {
        return sample.stamp.nanoseconds() < requested.nanoseconds();
      });
    if (upper == history.begin()) {
      if ((upper->stamp - stamp).seconds() > 0.15) {
        return false;
      }
      result = *upper;
      return true;
    }
    if (upper == history.end()) {
      const auto & latest = history.back();
      if ((stamp - latest.stamp).seconds() > 0.15) {
        return false;
      }
      result = latest;
      return true;
    }
    const auto lower = std::prev(upper);
    const double interval = (upper->stamp - lower->stamp).seconds();
    if (interval <= 0.0) {
      return false;
    }
    const double ratio = std::clamp((stamp - lower->stamp).seconds() / interval, 0.0, 1.0);
    result.pose.x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
    result.pose.y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
    result.pose.yaw =
      normalizeAngle(lower->pose.yaw + ratio * normalizeAngle(upper->pose.yaw - lower->pose.yaw));
    result.stamp = stamp;
    result.received = lower->received;
    result.available = true;
    return true;
  }

  void localizationCallback(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const auto payload = nlohmann::json::parse(message->data);
      if (payload.value("schema", "") != "r1_indoor_localization_status_v1") {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      localization_state_ = payload.value("state", "INVALID");
      localization_reason_ = payload.value("reason", "status_incomplete");
      localization_received_ = std::chrono::steady_clock::now();
      localization_available_ = true;
    } catch (const std::exception &) {
      std::lock_guard<std::mutex> lock(mutex_);
      localization_state_ = "INVALID";
      localization_reason_ = "status_parse_error";
      localization_received_ = std::chrono::steady_clock::now();
      localization_available_ = true;
    }
  }

  void dynamicFootprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
  {
    std::vector<double> points;
    points.reserve(message->polygon.points.size() * 2);
    for (const auto & point : message->polygon.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return;
      }
      points.push_back(point.x);
      points.push_back(point.y);
    }
    if (points.size() < 6) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_footprint_.points = std::move(points);
    dynamic_footprint_.received = std::chrono::steady_clock::now();
    dynamic_footprint_.available = true;
  }

  void costmapFootprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      return;
    }
    std::vector<double> points;
    points.reserve(message->polygon.points.size() * 2);
    for (const auto & point : message->polygon.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return;
      }
      points.push_back(point.x);
      points.push_back(point.y);
    }
    if (points.size() < 6) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    published_footprint_.points = std::move(points);
    published_footprint_.stamp = rclcpp::Time(message->header.stamp);
    published_footprint_.received = std::chrono::steady_clock::now();
    published_footprint_.available = true;
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    if (message->header.frame_id != map_frame_ || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return;
    }

    // A 2D Pose Estimate is an explicit operator acknowledgement that the
    // previous localization origin may be invalid. Re-run all admission
    // checks; this does not allow motion until the new pose proves stable.
    std::lock_guard<std::mutex> lock(health_mutex_);
    fault_latched_ = false;
    fault_reason_.clear();
    ever_ready_ = false;
    candidate_healthy_ = false;
    preflight_ready_ = false;
    previous_available_ = false;
    last_map_tf_stamp_available_ = false;
    soft_jump_count_ = 0;
    static_map_overlap_state_.reset();
    last_checked_costmap_generation_ = 0;
    last_lethal_sample_ = {};
    stable_costmap_samples_ = 0;
    stable_since_ = std::chrono::steady_clock::now();
    operator_relocalization_pending_ = true;
  }

  void requestCostmap()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!costmap_client_->service_is_ready()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (costmap_request_active_ || now - last_costmap_request_ < std::chrono::seconds(1)) {
        return;
      }
      costmap_request_active_ = true;
      last_costmap_request_ = now;
    }
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    costmap_client_->async_send_request(
      request,
      [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        costmap_request_active_ = false;
        try {
          costmap_ = future.get()->map;
          costmap_received_ = std::chrono::steady_clock::now();
          costmap_available_ = true;
          ++costmap_generation_;
        } catch (const std::exception &) {
          costmap_available_ = false;
        }
      });
  }

  LethalFootprintSample footprintLethalCost(
    const Pose2D & pose,
    const std::vector<double> & footprint) const
  {
    // Sample the A-PRIORI STATIC MAP, never the live master costmap. The live
    // obstacle layer marks whatever the robot currently sees, so an envelope
    // legitimately passing close to a freshly marked wall would otherwise
    // latch a localization fault while localization is perfectly self
    // consistent. The robot inside a known static wall is the actual evidence
    // that the map pose is wrong; proximity to live obstacles is the
    // collision guard's domain.
    LethalFootprintSample result;
    nav_msgs::msg::OccupancyGrid map;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!static_map_available_) {
        return result;
      }
      map = static_map_;
    }
    if (map.info.resolution <= 0.0 || map.data.empty()) {
      return result;
    }
    const auto & metadata = map.info;
    const double resolution = metadata.resolution;
    const auto & origin = metadata.origin.position;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < footprint.size(); index += 2) {
      const double x = pose.x + cosine * footprint[index] - sine * footprint[index + 1];
      const double y = pose.y + sine * footprint[index] + cosine * footprint[index + 1];
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
    const auto map_index = [resolution](const double coordinate, const double map_origin) {
        return static_cast<int64_t>(std::floor((coordinate - map_origin) / resolution));
      };
    const int64_t first_x = std::max<int64_t>(
      0, map_index(min_x - footprint_padding_m_, origin.x));
    const int64_t last_x = std::min<int64_t>(
      static_cast<int64_t>(metadata.width) - 1,
      map_index(max_x + footprint_padding_m_, origin.x));
    const int64_t first_y = std::max<int64_t>(
      0, map_index(min_y - footprint_padding_m_, origin.y));
    const int64_t last_y = std::min<int64_t>(
      static_cast<int64_t>(metadata.height) - 1,
      map_index(max_y + footprint_padding_m_, origin.y));
    if (first_x > last_x || first_y > last_y) {
      return result;
    }
    for (int64_t ix = first_x; ix <= last_x; ++ix) {
      for (int64_t iy = first_y; iy <= last_y; ++iy) {
        const double x = origin.x + (static_cast<double>(ix) + 0.5) * resolution;
        const double y = origin.y + (static_cast<double>(iy) + 0.5) * resolution;
        const double dx = cosine * (x - pose.x) + sine * (y - pose.y);
        const double dy = -sine * (x - pose.x) + cosine * (y - pose.y);
        const bool inside = deepInsideFootprint(footprint, dx, dy, static_map_fault_inset_m_);
        const size_t offset = static_cast<size_t>(iy) * metadata.width +
          static_cast<size_t>(ix);
        // Occupancy >= 65 is the standard lethal threshold. Unknown (-1) is
        // not evidence of a localization fault.
        if (inside && offset < map.data.size() && map.data[offset] >= 65) {
          ++result.count;
          if (result.count == 1) {
            result.map_x = x;
            result.map_y = y;
            result.base_x = dx;
            result.base_y = dy;
          }
        }
      }
    }
    return result;
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    const auto stable_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - stable_since_).count();
    if (!preflight_ready_ || !candidate_healthy_ || stable_age < stable_window_sec_ ||
      last_lethal_sample_.count != 0)
    {
      response->success = false;
      response->message = "localization is not stably READY";
      return;
    }
    fault_latched_ = false;
    fault_reason_.clear();
    static_map_overlap_state_.reset();
    operator_relocalization_pending_ = false;
    response->success = true;
    response->message = "localization fault latch cleared";
  }

  void publish(
    const std::string & state, const std::string & reason,
    const double translation_jump, const double yaw_jump, const double localization_status_age)
  {
    std_msgs::msg::String message;
    std::ostringstream payload;
    payload.setf(std::ios::fixed);
    payload.precision(6);
    payload << "{\"schema\":\"r1_localization_health_v1\""
            << ",\"state\":\"" << state << "\""
            << ",\"reason\":\"" << reason << "\""
            << ",\"ready\":" << (state == "READY" ? "true" : "false")
            << ",\"fault_latched\":" << (fault_latched_ ? "true" : "false")
            << ",\"translation_jump_m\":" << translation_jump
            << ",\"yaw_jump_rad\":" << yaw_jump
            << ",\"localization_status_age_sec\":" << localization_status_age
            << ",\"preflight_ready\":" << (preflight_ready_ ? "true" : "false")
            << ",\"operator_relocalization_pending\":" <<
      (operator_relocalization_pending_ ? "true" : "false")
            << ",\"stable_costmap_samples\":" << stable_costmap_samples_
            << ",\"lethal_cells\":" << last_lethal_sample_.count
            << ",\"lethal_map_x\":" << last_lethal_sample_.map_x
            << ",\"lethal_map_y\":" << last_lethal_sample_.map_y
            << ",\"lethal_base_x\":" << last_lethal_sample_.base_x
            << ",\"lethal_base_y\":" << last_lethal_sample_.base_y
            << "}";
    message.data = payload.str();
    status_pub_->publish(message);
  }

  void tick()
  {
    requestCostmap();
    const auto steady_now = std::chrono::steady_clock::now();
    OdomSample odom;
    std::deque<OdomSample> odom_history;
    std::string localization_state;
    std::string localization_reason;
    SteadyTime localization_received{};
    bool localization_available = false;
    SteadyTime costmap_received{};
    bool costmap_available = false;
    uint64_t costmap_generation = 0;
    bool static_map_available = false;
    DynamicFootprint dynamic_footprint;
    PublishedFootprint published_footprint;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom = odom_;
      odom_history = odom_history_;
      localization_state = localization_state_;
      localization_reason = localization_reason_;
      localization_received = localization_received_;
      localization_available = localization_available_;
      costmap_received = costmap_received_;
      costmap_available = costmap_available_;
      costmap_generation = costmap_generation_;
      static_map_available = static_map_available_;
      dynamic_footprint = dynamic_footprint_;
      published_footprint = published_footprint_;
    }
    std::lock_guard<std::mutex> health_lock(health_mutex_);

    const double localization_status_age = localization_available ?
      std::chrono::duration<double>(steady_now - localization_received).count() : -1.0;

    Pose2D map_pose;
    std::string reason;
    bool healthy = false;
    double translation_jump = 0.0;
    double yaw_jump = 0.0;
    rclcpp::Time map_tf_stamp{0, 0, RCL_ROS_TIME};
    try {
      const auto transform =
        tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      map_tf_stamp = rclcpp::Time(transform.header.stamp);
      const auto tf_age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      tf2::Quaternion quaternion;
      tf2::fromMsg(transform.transform.rotation, quaternion);
      map_pose = {transform.transform.translation.x, transform.transform.translation.y, tf2::getYaw(
          quaternion)};
      Pose2D footprint_map_pose;
      bool footprint_transform_available = false;
      if (require_dynamic_footprint_ && published_footprint.available &&
        published_footprint.stamp.nanoseconds() != 0)
      {
        try {
          const auto footprint_transform = tf_buffer_.lookupTransform(
            map_frame_, base_frame_, published_footprint.stamp);
          tf2::Quaternion footprint_quaternion;
          tf2::fromMsg(footprint_transform.transform.rotation, footprint_quaternion);
          footprint_map_pose = {
            footprint_transform.transform.translation.x,
            footprint_transform.transform.translation.y,
            tf2::getYaw(footprint_quaternion)};
          footprint_transform_available = true;
        } catch (const tf2::TransformException &) {
          // The current map transform remains the localization freshness
          // check. This historical transform is only for comparing geometry
          // expressed at the published footprint timestamp.
        }
      }
      if (tf_age < -0.02 || tf_age > tf_timeout_sec_) {
        reason = "tf_stale";
      } else if (!odom.available || std::chrono::duration<double>(
          steady_now - odom.received).count() > odom_timeout_sec_)
      {
        reason = "odom_stale";
      } else if (!localization_available || localization_status_age > localization_timeout_sec_) {
        reason = "localization_status_stale";
      } else if (localization_state != "NORMAL") {
        reason = "localization_" + localization_state + ":" + localization_reason;
      } else if (!costmap_available ||
        std::chrono::duration<double>(steady_now - costmap_received).count() > 2.0)
      {
        reason = "global_costmap_unavailable_or_stale";
      } else if (require_dynamic_footprint_ && (!dynamic_footprint.available ||
        std::chrono::duration<double>(steady_now - dynamic_footprint.received).count() >
        dynamic_footprint_timeout_sec_))
      {
        reason = "dynamic_footprint_unavailable_or_stale";
      } else if (require_dynamic_footprint_ && (!published_footprint.available ||
        std::chrono::duration<double>(steady_now - published_footprint.received).count() >
        costmap_footprint_timeout_sec_))
      {
        reason = "costmap_footprint_unavailable_or_stale";
      } else if (require_dynamic_footprint_ && !footprint_transform_available) {
        reason = "costmap_footprint_transform_unavailable";
      } else if (require_dynamic_footprint_ && !dynamicFootprintMatchesPublished(
          dynamic_footprint, published_footprint, footprint_map_pose, footprint_sync_tolerance_m_))
      {
        // Do not test a new arm shape against a costmap that still contains
        // the previous footprint.  The published footprint is Nav2's
        // acknowledgement that its costmap has applied the same geometry.
        reason = "costmap_footprint_not_synchronized";
      } else {
        healthy = true;
      }
    } catch (const std::exception &) {
      reason = "map_to_base_unavailable";
    }

    const bool sustained_localization_loss = reason == "localization_status_stale" &&
      localization_status_age >= localization_fault_timeout_sec_;
    if (ever_ready_ && !healthy && !fault_latched_ &&
      (reason != "localization_status_stale" || sustained_localization_loss))
    {
      fault_latched_ = true;
      fault_reason_ = reason;
    }
    const bool fresh_map_transform = healthy && map_tf_stamp.nanoseconds() != 0 &&
      (!last_map_tf_stamp_available_ ||
      map_tf_stamp.nanoseconds() > last_map_tf_stamp_.nanoseconds());
    if (fresh_map_transform) {
      OdomSample matched_odom;
      if (odomAt(odom_history, map_tf_stamp, matched_odom)) {
        if (previous_available_) {
          const double odom_dx = matched_odom.pose.x - previous_odom_.pose.x;
          const double odom_dy = matched_odom.pose.y - previous_odom_.pose.y;
          const double heading = previous_map_pose_.yaw - previous_odom_.pose.yaw;
          const double expected_dx = std::cos(heading) * odom_dx - std::sin(heading) * odom_dy;
          const double expected_dy = std::sin(heading) * odom_dx + std::cos(heading) * odom_dy;
          translation_jump = std::hypot(
            (map_pose.x - previous_map_pose_.x) - expected_dx,
            (map_pose.y - previous_map_pose_.y) - expected_dy);
          yaw_jump = std::abs(
            normalizeAngle(
              (map_pose.yaw - previous_map_pose_.yaw) -
              (matched_odom.pose.yaw - previous_odom_.pose.yaw)));
          if (ever_ready_ &&
            (translation_jump >= hard_translation_jump_m_ || yaw_jump >= hard_yaw_jump_rad_))
          {
            fault_latched_ = true;
            fault_reason_ = "localization_jump_hard";
          } else if (ever_ready_ &&
            (translation_jump >= soft_translation_jump_m_ || yaw_jump >= soft_yaw_jump_rad_))
          {
            ++soft_jump_count_;
            if (soft_jump_count_ >= 2) {
              fault_latched_ = true;
              fault_reason_ = "localization_jump_repeated";
            }
          } else if (ever_ready_) {
            soft_jump_count_ = 0;
          }
        }
        previous_map_pose_ = map_pose;
        previous_odom_ = matched_odom;
        previous_available_ = true;
      }
      last_map_tf_stamp_ = map_tf_stamp;
      last_map_tf_stamp_available_ = true;
    }
    if (healthy) {
      if (!candidate_healthy_) {
        stable_since_ = steady_now;
        stable_costmap_samples_ = 1;
        last_stable_costmap_generation_ = costmap_generation;
      } else if (costmap_generation != last_stable_costmap_generation_) {
        ++stable_costmap_samples_;
        last_stable_costmap_generation_ = costmap_generation;
      }
      candidate_healthy_ = true;
    } else {
      candidate_healthy_ = false;
      stable_since_ = steady_now;
      stable_costmap_samples_ = 0;
    }
    preflight_ready_ = candidate_healthy_ &&
      std::chrono::duration<double>(steady_now - stable_since_).count() >= stable_window_sec_ &&
      stable_costmap_samples_ >= 2;
    if (preflight_ready_) {
      ever_ready_ = true;
    }
    const bool valid_static_map_check = preflight_ready_ && static_map_available;
    const bool fresh_static_map_check = valid_static_map_check &&
      costmap_generation != last_checked_costmap_generation_;
    if (fresh_static_map_check) {
      last_checked_costmap_generation_ = costmap_generation;
      // Only the chassis box is checked against the static map. Arm proximity
      // is handled by the collision guard and the planner footprint.
      last_lethal_sample_ = footprintLethalCost(map_pose, footprint_);
    }
    const auto static_map_transition = static_map_overlap_state_.observe(
      valid_static_map_check, fresh_static_map_check, last_lethal_sample_.count != 0,
      steady_now);
    if (static_map_transition == StaticMapOverlapTransition::Latched && !fault_latched_) {
      fault_latched_ = true;
      fault_reason_ = "start_in_lethal_static_map";
      RCLCPP_ERROR(
        get_logger(), "Static map overlap: %u occupied cells; first cell at (%.3f, %.3f)",
        last_lethal_sample_.count, last_lethal_sample_.map_x, last_lethal_sample_.map_y);
    } else if (static_map_transition == StaticMapOverlapTransition::Recovered &&
      fault_reason_ == "start_in_lethal_static_map")
    {
      // Recover only this transient map-overlap fault after stable clear samples.
      fault_latched_ = false;
      fault_reason_.clear();
      RCLCPP_INFO(get_logger(), "Static map overlap cleared after stable free samples");
    }
    if (!healthy) {
      previous_available_ = false;
      last_map_tf_stamp_available_ = false;
    }

    if (fault_latched_) {
      publish("FAULT", fault_reason_, translation_jump, yaw_jump, localization_status_age);
    } else if (!preflight_ready_) {
      // Surface the failing admission check even while an operator
      // relocalization is settling, so a stuck bootstrap is diagnosable
      // from the status topic alone.
      publish(
        "BOOTSTRAP", operator_relocalization_pending_ ?
        "operator_initial_pose_stabilizing:" + reason :
        (healthy ? "stability_window" : reason),
        translation_jump, yaw_jump, localization_status_age);
    } else {
      operator_relocalization_pending_ = false;
      publish("READY", "ready", translation_jump, yaw_jump, localization_status_age);
    }
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string status_topic_;
  std::string costmap_service_;
  double tf_timeout_sec_{0.25};
  double odom_timeout_sec_{1.5};
  double localization_timeout_sec_{1.5};
  double localization_fault_timeout_sec_{3.0};
  double stable_window_sec_{3.0};
  double soft_translation_jump_m_{0.08};
  double hard_translation_jump_m_{0.20};
  double soft_yaw_jump_rad_{0.0872665};
  double hard_yaw_jump_rad_{0.2094395};
  double footprint_padding_m_{0.01};
  double static_map_fault_inset_m_{0.0};
  std::vector<double> footprint_;
  bool require_dynamic_footprint_{false};
  double dynamic_footprint_timeout_sec_{0.50};
  double costmap_footprint_timeout_sec_{2.50};
  double footprint_sync_tolerance_m_{0.03};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  mutable std::mutex mutex_;
  mutable std::mutex health_mutex_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  OdomSample odom_;
  std::deque<OdomSample> odom_history_;
  std::string localization_state_;
  std::string localization_reason_;
  SteadyTime localization_received_{};
  bool localization_available_{false};
  DynamicFootprint dynamic_footprint_;
  PublishedFootprint published_footprint_;
  nav2_msgs::msg::Costmap costmap_;
  SteadyTime costmap_received_{};
  bool costmap_available_{false};
  bool costmap_request_active_{false};
  SteadyTime last_costmap_request_{};
  uint64_t costmap_generation_{0};
  Pose2D previous_map_pose_;
  OdomSample previous_odom_;
  bool previous_available_{false};
  rclcpp::Time last_map_tf_stamp_{0, 0, RCL_ROS_TIME};
  bool last_map_tf_stamp_available_{false};
  int soft_jump_count_{0};
  bool candidate_healthy_{false};
  SteadyTime stable_since_{std::chrono::steady_clock::now()};
  uint64_t last_stable_costmap_generation_{0};
  uint32_t stable_costmap_samples_{0};
  bool preflight_ready_{false};
  bool ever_ready_{false};
  uint64_t last_checked_costmap_generation_{0};
  StaticMapOverlapState static_map_overlap_state_{3.0};
  LethalFootprintSample last_lethal_sample_;
  bool fault_latched_{false};
  bool operator_relocalization_pending_{false};
  std::string fault_reason_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr dynamic_footprint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr costmap_footprint_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  nav_msgs::msg::OccupancyGrid static_map_;
  bool static_map_available_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigation_neupan_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_neupan_bridge::LocalizationMonitor>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
