// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
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
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

bool pointInPolygon(const std::vector<double> & polygon, const double x, const double y)
{
  bool inside = false;
  const size_t count = polygon.size() / 2;
  for (size_t index = 0, previous = count - 1; index < count; previous = index++) {
    const double current_x = polygon[index * 2];
    const double current_y = polygon[index * 2 + 1];
    const double previous_x = polygon[previous * 2];
    const double previous_y = polygon[previous * 2 + 1];
    const bool crosses = (current_y > y) != (previous_y > y);
    if (crosses && x < (previous_x - current_x) * (y - current_y) /
      (previous_y - current_y) + current_x)
    {
      inside = !inside;
    }
  }
  return inside;
}

double pointSegmentDistance(const double x, const double y, const double ax, const double ay,
  const double bx, const double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-12) {
    return std::hypot(x - ax, y - ay);
  }
  const double ratio = std::clamp(((x - ax) * dx + (y - ay) * dy) / length_squared, 0.0, 1.0);
  return std::hypot(x - (ax + ratio * dx), y - (ay + ratio * dy));
}

bool insideOrNearPolygon(const std::vector<double> & polygon, const double x, const double y,
  const double padding)
{
  if (pointInPolygon(polygon, x, y)) {
    return true;
  }
  const size_t count = polygon.size() / 2;
  for (size_t index = 0; index < count; ++index) {
    const size_t next = (index + 1) % count;
    if (pointSegmentDistance(x, y, polygon[index * 2], polygon[index * 2 + 1],
      polygon[next * 2], polygon[next * 2 + 1]) <= padding)
    {
      return true;
    }
  }
  return false;
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
    localization_timeout_sec_ = declare_parameter<double>("localization_timeout_sec", 0.50);
    stable_window_sec_ = declare_parameter<double>("stable_window_sec", 3.0);
    soft_translation_jump_m_ = declare_parameter<double>("soft_translation_jump_m", 0.08);
    hard_translation_jump_m_ = declare_parameter<double>("hard_translation_jump_m", 0.20);
    soft_yaw_jump_rad_ = declare_parameter<double>("soft_yaw_jump_rad", 0.0872665);
    hard_yaw_jump_rad_ = declare_parameter<double>("hard_yaw_jump_rad", 0.2094395);
    footprint_padding_m_ = declare_parameter<double>("footprint_padding_m", 0.01);
    footprint_ = declare_parameter<std::vector<double>>(
      "footprint_points", std::vector<double>{-0.341, -0.266, -0.341, 0.266, 0.341, 0.266, 0.341, -0.266});
    require_dynamic_footprint_ = declare_parameter<bool>("require_dynamic_footprint", false);
    dynamic_footprint_timeout_sec_ = declare_parameter<double>(
      "dynamic_footprint_timeout_sec", 0.50);
    const auto dynamic_footprint_topic = declare_parameter<std::string>(
      "dynamic_footprint_topic", "/r1_arm_footprint");
    if (footprint_.size() < 6 || footprint_.size() % 2 != 0 || tf_timeout_sec_ <= 0.0 ||
      odom_timeout_sec_ <= 0.0 || localization_timeout_sec_ <= 0.0 || stable_window_sec_ <= 0.0 ||
      dynamic_footprint_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid localization monitor configuration");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50), std::bind(&LocalizationMonitor::odomCallback, this, std::placeholders::_1));
    localization_sub_ = create_subscription<std_msgs::msg::String>(
      localization_topic, rclcpp::QoS(20), std::bind(&LocalizationMonitor::localizationCallback, this, std::placeholders::_1));
    dynamic_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      dynamic_footprint_topic, rclcpp::QoS(20),
      std::bind(&LocalizationMonitor::dynamicFootprintCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(20));
    costmap_client_ = create_client<nav2_msgs::srv::GetCostmap>(costmap_service_);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/r1_localization_monitor/reset",
      std::bind(&LocalizationMonitor::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalizationMonitor::tick, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    tf2::Quaternion quaternion;
    tf2::fromMsg(message->pose.pose.orientation, quaternion);
    std::lock_guard<std::mutex> lock(mutex_);
    OdomSample sample;
    sample.pose = {message->pose.pose.position.x, message->pose.pose.position.y, tf2::getYaw(quaternion)};
    sample.stamp = message->header.stamp;
    sample.received = std::chrono::steady_clock::now();
    sample.available = true;
    odom_ = sample;
    odom_history_.push_back(sample);
    while (odom_history_.size() > 200) {
      odom_history_.pop_front();
    }
  }

  bool odomAt(const std::deque<OdomSample> & history, const rclcpp::Time & stamp, OdomSample & result) const
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
    result.pose.yaw = normalizeAngle(lower->pose.yaw + ratio * normalizeAngle(upper->pose.yaw - lower->pose.yaw));
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
    costmap_client_->async_send_request(request,
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

  LethalFootprintSample footprintLethalCost(const Pose2D & pose,
    const std::vector<double> & footprint) const
  {
    LethalFootprintSample result;
    nav2_msgs::msg::Costmap costmap;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!costmap_available_) {
        return result;
      }
      costmap = costmap_;
    }
    if (costmap.metadata.resolution <= 0.0 || costmap.data.empty()) {
      return result;
    }
    const auto & metadata = costmap.metadata;
    const double resolution = metadata.resolution;
    const auto & origin = metadata.origin.position;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    for (uint32_t ix = 0; ix < metadata.size_x; ++ix) {
      for (uint32_t iy = 0; iy < metadata.size_y; ++iy) {
        const double x = origin.x + (static_cast<double>(ix) + 0.5) * resolution;
        const double y = origin.y + (static_cast<double>(iy) + 0.5) * resolution;
        const double dx = cosine * (x - pose.x) + sine * (y - pose.y);
        const double dy = -sine * (x - pose.x) + cosine * (y - pose.y);
        const bool inside = insideOrNearPolygon(footprint, dx, dy, footprint_padding_m_);
        const size_t offset = static_cast<size_t>(iy) * metadata.size_x + ix;
        if (inside && offset < costmap.data.size() && costmap.data[offset] >= 254) {
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

  void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stable_age = std::chrono::duration<double>(std::chrono::steady_clock::now() - stable_since_).count();
    if (!preflight_ready_ || !candidate_healthy_ || stable_age < stable_window_sec_ ||
      last_lethal_sample_.count != 0)
    {
      response->success = false;
      response->message = "localization is not stably READY";
      return;
    }
    fault_latched_ = false;
    fault_reason_.clear();
    consecutive_lethal_costmaps_ = 0;
    response->success = true;
    response->message = "localization fault latch cleared";
  }

  void publish(const std::string & state, const std::string & reason,
    const double translation_jump, const double yaw_jump)
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
            << ",\"preflight_ready\":" << (preflight_ready_ ? "true" : "false")
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
    DynamicFootprint dynamic_footprint;
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
      dynamic_footprint = dynamic_footprint_;
    }

    Pose2D map_pose;
    std::string reason;
    bool healthy = false;
    double translation_jump = 0.0;
    double yaw_jump = 0.0;
    rclcpp::Time map_tf_stamp{0, 0, RCL_ROS_TIME};
    try {
      const auto transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      map_tf_stamp = rclcpp::Time(transform.header.stamp);
      const auto tf_age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      tf2::Quaternion quaternion;
      tf2::fromMsg(transform.transform.rotation, quaternion);
      map_pose = {transform.transform.translation.x, transform.transform.translation.y, tf2::getYaw(quaternion)};
      if (tf_age < -0.02 || tf_age > tf_timeout_sec_) {
        reason = "tf_stale";
      } else if (!odom.available || std::chrono::duration<double>(steady_now - odom.received).count() > odom_timeout_sec_) {
        reason = "odom_stale";
      } else if (!localization_available ||
        std::chrono::duration<double>(steady_now - localization_received).count() > localization_timeout_sec_)
      {
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
      } else {
        healthy = true;
      }
    } catch (const std::exception &) {
      reason = "map_to_base_unavailable";
    }

    if (ever_ready_ && !healthy && !fault_latched_) {
      fault_latched_ = true;
      fault_reason_ = reason;
    }
    const bool fresh_map_transform = healthy && map_tf_stamp.nanoseconds() != 0 &&
      (!last_map_tf_stamp_available_ || map_tf_stamp.nanoseconds() > last_map_tf_stamp_.nanoseconds());
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
          yaw_jump = std::abs(normalizeAngle(
            (map_pose.yaw - previous_map_pose_.yaw) - (matched_odom.pose.yaw - previous_odom_.pose.yaw)));
          if (ever_ready_ && (translation_jump >= hard_translation_jump_m_ || yaw_jump >= hard_yaw_jump_rad_)) {
            fault_latched_ = true;
            fault_reason_ = "localization_jump_hard";
          } else if (ever_ready_ &&
            (translation_jump >= soft_translation_jump_m_ || yaw_jump >= soft_yaw_jump_rad_)) {
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
      if (costmap_generation != last_checked_costmap_generation_) {
        last_checked_costmap_generation_ = costmap_generation;
        last_lethal_sample_ = footprintLethalCost(
          map_pose, require_dynamic_footprint_ ? dynamic_footprint.points : footprint_);
        if (last_lethal_sample_.count != 0) {
          ++consecutive_lethal_costmaps_;
          if (consecutive_lethal_costmaps_ >= 2) {
            fault_latched_ = true;
            fault_reason_ = "start_in_lethal_static_map";
          }
        } else {
          consecutive_lethal_costmaps_ = 0;
        }
      }
    }
    if (!healthy) {
      previous_available_ = false;
      last_map_tf_stamp_available_ = false;
    }

    if (fault_latched_) {
      publish("FAULT", fault_reason_, translation_jump, yaw_jump);
    } else if (!preflight_ready_) {
      publish("BOOTSTRAP", healthy ? "stability_window" : reason, translation_jump, yaw_jump);
    } else {
      publish("READY", "ready", translation_jump, yaw_jump);
    }
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string status_topic_;
  std::string costmap_service_;
  double tf_timeout_sec_{0.25};
  double odom_timeout_sec_{1.5};
  double localization_timeout_sec_{0.5};
  double stable_window_sec_{3.0};
  double soft_translation_jump_m_{0.08};
  double hard_translation_jump_m_{0.20};
  double soft_yaw_jump_rad_{0.0872665};
  double hard_yaw_jump_rad_{0.2094395};
  double footprint_padding_m_{0.01};
  std::vector<double> footprint_;
  bool require_dynamic_footprint_{false};
  double dynamic_footprint_timeout_sec_{0.50};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  mutable std::mutex mutex_;
  OdomSample odom_;
  std::deque<OdomSample> odom_history_;
  std::string localization_state_;
  std::string localization_reason_;
  SteadyTime localization_received_{};
  bool localization_available_{false};
  DynamicFootprint dynamic_footprint_;
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
  uint32_t consecutive_lethal_costmaps_{0};
  LethalFootprintSample last_lethal_sample_;
  bool fault_latched_{false};
  std::string fault_reason_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr dynamic_footprint_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigation_neupan_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_neupan_bridge::LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
