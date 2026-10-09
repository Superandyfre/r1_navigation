// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#include "r1_collision_guard/guard_logic.hpp"
#include "r1_collision_guard/scan_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace r1_collision_guard
{

namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

struct OdomSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Pose2D pose{};
  double linear_x{0.0};
  double angular_z{0.0};
  std::string frame;
  std::string child_frame;
  SteadyTime received{};
};

struct LocalizationHealth
{
  std::string state;
  std::string reason;
  SteadyTime received{};
  bool available{false};
};

struct ScanSample
{
  sensor_msgs::msg::LaserScan scan;
  SteadyTime received{};
};

struct ArmState
{
  bool available{false};
  bool valid{false};
  bool stowed{false};
  bool moving{false};
  double max_position_error{0.0};
  SteadyTime received{};
  SteadyTime oldest_joint_received{};
  SteadyTime stable_since{};
};

struct DynamicFootprint
{
  std::vector<Point2D> points;
  SteadyTime received{};
  bool available{false};
};

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny, cosy);
}

Point2D transformPoint(const Point2D & point, const Pose2D & from, const Pose2D & to)
{
  const double from_cosine = std::cos(from.yaw);
  const double from_sine = std::sin(from.yaw);
  const double world_x = from.x + from_cosine * point.x - from_sine * point.y;
  const double world_y = from.y + from_sine * point.x + from_cosine * point.y;
  const double dx = world_x - to.x;
  const double dy = world_y - to.y;
  const double to_cosine = std::cos(to.yaw);
  const double to_sine = std::sin(to.yaw);
  return {to_cosine * dx + to_sine * dy, -to_sine * dx + to_cosine * dy};
}
}  // namespace

class CollisionGuardNode : public rclcpp::Node
{
public:
  CollisionGuardNode()
  : Node("r1_collision_guard")
  {
    const auto footprint_points = declare_parameter<std::vector<double>>(
      "footprint_points", std::vector<double>{});
    GuardConfig config;
    config.max_linear_mps = declare_parameter<double>("max_linear_mps", 0.15);
    config.max_reverse_mps = declare_parameter<double>("max_reverse_mps", 0.15);
    config.max_angular_radps = declare_parameter<double>("max_angular_radps", 0.4);
    control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
    config.control_period_sec = 1.0 / control_frequency_;
    config.reaction_delay_sec = declare_parameter<double>("reaction_delay_sec", 0.10);
    config.braking_deceleration_mps2 = declare_parameter<double>("braking_deceleration_mps2", 0.30);
    config.braking_angular_deceleration_radps2 = declare_parameter<double>(
      "braking_angular_deceleration_radps2", 0.80);
    config.geometry_margin_m = declare_parameter<double>("geometry_margin_m", 0.02);
    config.tracking_error_mps = declare_parameter<double>("tracking_error_mps", 0.0);
    config.binary_search_iterations = declare_parameter<int>("binary_search_iterations", 10);
    config.allow_reverse = declare_parameter<bool>("allow_reverse", false);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.25);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.15);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.20);
    source_command_timeout_sec_ = declare_parameter<double>("source_command_timeout_sec", command_timeout_sec_);
    scan_history_size_ = declare_parameter<int>("scan_history_size", 32);
    max_scan_range_m_ = declare_parameter<double>("max_scan_range_m", 8.0);
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan_neupan");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto command_topic = declare_parameter<std::string>(
      "cmd_vel_in_topic", "/neupan_nav2_cmd_vel");
    const auto source_command_topic = declare_parameter<std::string>(
      "source_cmd_vel_topic", command_topic);
    const auto output_topic = declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel");
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/r1_collision_guard/status");
    const auto localization_health_topic = declare_parameter<std::string>(
      "localization_health_topic", "/r1_localization_health");
    localization_health_timeout_sec_ = declare_parameter<double>(
      "localization_health_timeout_sec", 0.50);
    require_localization_ready_ = declare_parameter<bool>(
      "require_localization_ready", true);
    require_arms_stowed_ = declare_parameter<bool>("require_arms_stowed", true);
    require_dynamic_footprint_ = declare_parameter<bool>("require_dynamic_footprint", false);
    mesh_self_filter_preapplied_ = declare_parameter<bool>("mesh_self_filter_preapplied", false);
    dynamic_footprint_timeout_sec_ = declare_parameter<double>(
      "dynamic_footprint_timeout_sec", 0.50);
    const auto dynamic_footprint_topic = declare_parameter<std::string>(
      "dynamic_footprint_topic", "/r1_arm_footprint");
    joint_state_timeout_sec_ = declare_parameter<double>("joint_state_timeout_sec", 0.50);
    arm_stow_tolerance_rad_ = declare_parameter<double>("arm_stow_tolerance_rad", 0.035);
    arm_motion_tolerance_radps_ = declare_parameter<double>(
      "arm_motion_tolerance_radps", 0.020);
    arm_settle_time_sec_ = declare_parameter<double>("arm_settle_time_sec", 1.0);
    arm_joint_names_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names", std::vector<std::string>{});
    arm_stowed_positions_ = declare_parameter<std::vector<double>>(
      "arm_stowed_positions", std::vector<double>{});
    arm_joint_state_topics_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_state_topics", std::vector<std::string>{});

    if (config.max_linear_mps <= 0.0 || config.max_reverse_mps <= 0.0 ||
      control_frequency_ <= 0.0 || scan_timeout_sec_ <= 0.0 || odom_timeout_sec_ <= 0.0 ||
      command_timeout_sec_ <= 0.0 || source_command_timeout_sec_ <= 0.0 ||
      localization_health_timeout_sec_ <= 0.0 ||
      dynamic_footprint_timeout_sec_ <= 0.0 ||
      joint_state_timeout_sec_ <= 0.0 || arm_stow_tolerance_rad_ <= 0.0 ||
      arm_motion_tolerance_radps_ < 0.0 || arm_settle_time_sec_ < 0.0 ||
      scan_history_size_ < 2 ||
      footprint_points.size() < 6 || footprint_points.size() % 2 != 0)
    {
      throw std::runtime_error("invalid R1 collision guard parameters");
    }
    if (require_arms_stowed_ && (arm_joint_names_.empty() ||
      arm_joint_names_.size() != arm_stowed_positions_.size() ||
      arm_joint_state_topics_.empty()))
    {
      throw std::runtime_error("arm stow guard requires matching joint names and positions");
    }
    std::vector<Point2D> footprint;
    footprint.reserve(footprint_points.size() / 2);
    for (size_t index = 0; index < footprint_points.size(); index += 2) {
      footprint.push_back({footprint_points[index], footprint_points[index + 1]});
    }
    logic_ = std::make_unique<GuardLogic>(config, std::move(footprint));

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::commandCallback, this, std::placeholders::_1));
    source_command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      source_command_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::sourceCommandCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&CollisionGuardNode::scanCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50),
      std::bind(&CollisionGuardNode::odomCallback, this, std::placeholders::_1));
    localization_health_sub_ = create_subscription<std_msgs::msg::String>(
      localization_health_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::localizationHealthCallback, this, std::placeholders::_1));
    for (const auto & topic : arm_joint_state_topics_) {
      arm_joint_state_subs_.push_back(create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::QoS(20),
        std::bind(&CollisionGuardNode::jointStateCallback, this, std::placeholders::_1)));
    }
    dynamic_footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      dynamic_footprint_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::dynamicFootprintCallback, this, std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(20));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(20));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&CollisionGuardNode::tick, this));
    RCLCPP_INFO(get_logger(), "R1 collision guard active: %s (source %s) -> %s at %.1f Hz",
      command_topic.c_str(), source_command_topic.c_str(), output_topic.c_str(), control_frequency_);
  }

private:
  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = *message;
    command_received_ = std::chrono::steady_clock::now();
    command_available_ = true;
  }

  void sourceCommandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    source_command_received_ = std::chrono::steady_clock::now();
    source_command_available_ = true;
    source_command_valid_ = std::isfinite(message->linear.x) && std::isfinite(message->angular.z);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan_ = *message;
    scan_received_ = std::chrono::steady_clock::now();
    scan_history_.push_back({*message, scan_received_});
    while (static_cast<int>(scan_history_.size()) > scan_history_size_) {
      scan_history_.pop_front();
    }
    scan_available_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    OdomSample sample;
    sample.stamp = message->header.stamp;
    sample.pose = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      yawFromQuaternion(message->pose.pose.orientation)};
    sample.linear_x = message->twist.twist.linear.x;
    sample.angular_z = message->twist.twist.angular.z;
    sample.frame = message->header.frame_id;
    sample.child_frame = message->child_frame_id;
    sample.received = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    odom_history_.push_back(sample);
    while (odom_history_.size() > 240) {
      odom_history_.pop_front();
    }
  }

  void localizationHealthCallback(const std_msgs::msg::String::SharedPtr message)
  {
    LocalizationHealth health;
    try {
      const auto payload = nlohmann::json::parse(message->data);
      if (payload.value("schema", "") != "r1_localization_health_v1") {
        throw std::runtime_error("schema_invalid");
      }
      health.state = payload.at("state").get<std::string>();
      health.reason = payload.at("reason").get<std::string>();
      if (health.state != "BOOTSTRAP" && health.state != "READY" && health.state != "FAULT") {
        throw std::runtime_error("state_invalid");
      }
      health.available = true;
      health.received = std::chrono::steady_clock::now();
    } catch (const std::exception &) {
      health.state = "FAULT";
      health.reason = "status_parse_error";
      health.available = true;
      health.received = std::chrono::steady_clock::now();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (health.state == "READY") {
      localization_ready_seen_ = true;
    }
    localization_health_ = std::move(health);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    ArmState arm;
    arm.available = true;
    arm.received = std::chrono::steady_clock::now();
    std::unordered_map<std::string, size_t> indices;
    for (size_t index = 0; index < message->name.size(); ++index) {
      indices.emplace(message->name[index], index);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    bool message_valid = message->name.size() == message->position.size();
    for (const auto & joint_name : arm_joint_names_) {
      const auto iterator = indices.find(joint_name);
      if (iterator == indices.end()) {
        continue;
      }
      const double position = message->position[iterator->second];
      if (!std::isfinite(position)) {
        message_valid = false;
        continue;
      }
      const auto prior = arm_positions_.find(joint_name);
      const auto prior_received = arm_joint_received_.find(joint_name);
      if (prior != arm_positions_.end() && prior_received != arm_joint_received_.end()) {
        const double elapsed = std::chrono::duration<double>(
          arm.received - prior_received->second).count();
        if (elapsed > 1e-4 &&
          std::abs(position - prior->second) / elapsed > arm_motion_tolerance_radps_)
        {
          arm.moving = true;
        }
      }
      if (message->velocity.size() == message->name.size()) {
        const double velocity = message->velocity[iterator->second];
        if (!std::isfinite(velocity)) {
          message_valid = false;
        } else if (std::abs(velocity) > arm_motion_tolerance_radps_) {
          arm.moving = true;
        }
      }
      arm_positions_[joint_name] = position;
      arm_joint_received_[joint_name] = arm.received;
    }
    arm.valid = message_valid;
    for (size_t index = 0; index < arm_joint_names_.size(); ++index) {
      const auto position = arm_positions_.find(arm_joint_names_[index]);
      const auto received = arm_joint_received_.find(arm_joint_names_[index]);
      if (position == arm_positions_.end() || received == arm_joint_received_.end()) {
        arm.valid = false;
        break;
      }
      arm.max_position_error = std::max(
        arm.max_position_error, std::abs(position->second - arm_stowed_positions_[index]));
      if (arm.oldest_joint_received == SteadyTime{} ||
        received->second < arm.oldest_joint_received)
      {
        arm.oldest_joint_received = received->second;
      }
    }
    arm.stowed = arm.valid && arm.max_position_error <= arm_stow_tolerance_rad_;
    if (arm.valid && arm.stowed && !arm.moving) {
      arm.stable_since = (arm_state_.valid && arm_state_.stowed && !arm_state_.moving)
        ? arm_state_.stable_since : arm.received;
    }
    arm_state_ = std::move(arm);
  }

  void dynamicFootprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
  {
    DynamicFootprint footprint;
    if (message->header.frame_id != base_frame_ || message->polygon.points.size() < 3) {
      return;
    }
    footprint.points.reserve(message->polygon.points.size());
    for (const auto & point : message->polygon.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return;
      }
      footprint.points.push_back({point.x, point.y});
    }
    footprint.received = std::chrono::steady_clock::now();
    footprint.available = true;
    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_footprint_ = std::move(footprint);
  }

  bool interpolateOdom(const std::deque<OdomSample> & history, const rclcpp::Time & stamp,
    OdomSample & output) const
  {
    if (history.empty() || stamp.nanoseconds() == 0) {
      return false;
    }
    if (stamp < history.front().stamp || stamp > history.back().stamp) {
      return false;
    }
    if (stamp == history.front().stamp) {
      output = history.front();
      return true;
    }
    for (size_t index = 1; index < history.size(); ++index) {
      const auto & before = history[index - 1];
      const auto & after = history[index];
      // The driver can publish several transport messages with the same odom
      // stamp. They do not form an interpolation interval.
      if (after.stamp <= before.stamp) {
        continue;
      }
      if (stamp > after.stamp) {
        continue;
      }
      const double span = (after.stamp - before.stamp).seconds();
      if (span <= 0.0) {
        return false;
      }
      const double ratio = (stamp - before.stamp).seconds() / span;
      output = after;
      output.stamp = stamp;
      output.pose.x = before.pose.x + ratio * (after.pose.x - before.pose.x);
      output.pose.y = before.pose.y + ratio * (after.pose.y - before.pose.y);
      output.pose.yaw = before.pose.yaw + ratio * std::atan2(
        std::sin(after.pose.yaw - before.pose.yaw), std::cos(after.pose.yaw - before.pose.yaw));
      output.linear_x = before.linear_x + ratio * (after.linear_x - before.linear_x);
      output.angular_z = before.angular_z + ratio * (after.angular_z - before.angular_z);
      return true;
    }
    return false;
  }

  bool selectAlignedScan(const std::deque<ScanSample> & history,
    const rclcpp::Time & latest_odom_stamp, ScanSample & output) const
  {
    for (auto sample = history.rbegin(); sample != history.rend(); ++sample) {
      if (rclcpp::Time(sample->scan.header.stamp).nanoseconds() != 0 &&
        rclcpp::Time(sample->scan.header.stamp) <= latest_odom_stamp)
      {
        output = *sample;
        return true;
      }
    }
    return false;
  }

  std::vector<Point2D> currentObstaclePoints(
    const sensor_msgs::msg::LaserScan & scan, const OdomSample & scan_odom,
    const OdomSample & current_odom) const
  {
    std::vector<Point2D> points;
    points.reserve(scan.ranges.size());
    double angle = scan.angle_min;
    for (const float range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max &&
        range <= max_scan_range_m_)
      {
        points.push_back(transformPoint(
          {range * std::cos(angle), range * std::sin(angle)}, scan_odom.pose, current_odom.pose));
      }
      angle += scan.angle_increment;
    }
    return points;
  }

  Decision fault(const std::string & reason) const
  {
    Decision decision;
    decision.state = Decision::State::FAULT;
    decision.reason = reason;
    return decision;
  }

  Decision stop(const std::string & reason) const
  {
    Decision decision;
    decision.state = Decision::State::STOP;
    decision.reason = reason;
    return decision;
  }

  void tick()
  {
    geometry_msgs::msg::Twist command;
    sensor_msgs::msg::LaserScan scan;
    std::deque<ScanSample> scan_history;
    std::deque<OdomSample> odom_history;
    SteadyTime command_received{};
    SteadyTime source_command_received{};
    SteadyTime scan_received{};
    bool command_available = false;
    bool source_command_available = false;
    bool source_command_valid = false;
    bool scan_available = false;
    bool localization_ready_seen = false;
    LocalizationHealth localization_health;
    ArmState arm_state;
    DynamicFootprint dynamic_footprint;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command = command_;
      scan = scan_;
      scan_history = scan_history_;
      odom_history = odom_history_;
      command_received = command_received_;
      source_command_received = source_command_received_;
      scan_received = scan_received_;
      command_available = command_available_;
      source_command_available = source_command_available_;
      source_command_valid = source_command_valid_;
      scan_available = scan_available_;
      localization_ready_seen = localization_ready_seen_;
      localization_health = localization_health_;
      arm_state = arm_state_;
      dynamic_footprint = dynamic_footprint_;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    const auto ros_now = now();
    Decision decision;
    size_t self_filtered_scan_points{0};
    const bool localization_health_stale = !localization_health.available ||
      std::chrono::duration<double>(steady_now - localization_health.received).count() >
      localization_health_timeout_sec_;
    const bool arm_state_stale = !arm_state.available ||
      arm_state.oldest_joint_received == SteadyTime{} ||
      std::chrono::duration<double>(steady_now - arm_state.oldest_joint_received).count() >
      joint_state_timeout_sec_;
    const bool dynamic_footprint_stale = !dynamic_footprint.available ||
      std::chrono::duration<double>(steady_now - dynamic_footprint.received).count() >
      dynamic_footprint_timeout_sec_;
    if (require_localization_ready_ && !localization_ready_seen &&
      (localization_health_stale || localization_health.state == "BOOTSTRAP"))
    {
      // A map-to-odom transform is not expected until the operator supplies an
      // initial pose. Keep the base stopped, but do not turn that startup state
      // into an irreversible task fault.
      decision = stop("waiting_for_localization_ready");
    } else if (require_localization_ready_ && localization_health_stale) {
      decision = fault("localization_status_stale");
    } else if (require_localization_ready_ && localization_health.state != "READY") {
      decision = fault("localization_" + localization_health.state + ":" + localization_health.reason);
    } else if (require_dynamic_footprint_ && dynamic_footprint_stale) {
      decision = fault("dynamic_footprint_stale");
    } else if (require_arms_stowed_ && arm_state_stale) {
      decision = fault("arm_joint_state_stale");
    } else if (require_arms_stowed_ && !arm_state.valid) {
      decision = fault("arm_joint_state_invalid");
    } else if (require_arms_stowed_ && !arm_state.stowed) {
      decision = fault("arm_not_stowed");
    } else if (require_arms_stowed_ && arm_state.moving) {
      decision = fault("arm_moving");
    } else if (require_arms_stowed_ &&
      std::chrono::duration<double>(steady_now - arm_state.stable_since).count() <
      arm_settle_time_sec_)
    {
      decision = stop("arm_stabilizing");
    } else if (!source_command_available ||
      std::chrono::duration<double>(steady_now - source_command_received).count() > source_command_timeout_sec_)
    {
      decision = fault("source_command_stale");
    } else if (!source_command_valid) {
      decision = fault("source_command_nonfinite");
    } else if (!command_available || std::chrono::duration<double>(steady_now - command_received).count() > command_timeout_sec_) {
      decision = fault("command_stale");
    } else if (!scan_available || std::chrono::duration<double>(steady_now - scan_received).count() > scan_timeout_sec_) {
      decision = fault("scan_stale");
    } else if (odom_history.empty() ||
      std::chrono::duration<double>(steady_now - odom_history.back().received).count() > odom_timeout_sec_) {
      decision = fault("odom_stale");
    } else if (odom_history.back().stamp.nanoseconds() == 0 ||
      (ros_now - odom_history.back().stamp).seconds() < -0.02 ||
      (ros_now - odom_history.back().stamp).seconds() > odom_timeout_sec_)
    {
      decision = fault("odom_stamp_stale_or_future");
    } else if (odom_history.back().frame != odom_frame_) {
      decision = fault("odom_frame_mismatch");
    } else if (odom_history.back().child_frame != base_frame_) {
      decision = fault("odom_child_frame_mismatch");
    } else if (!std::isfinite(command.linear.x) || !std::isfinite(command.angular.z) ||
      !std::isfinite(odom_history.back().linear_x) || !std::isfinite(odom_history.back().angular_z))
    {
      decision = fault("nonfinite_input");
    } else {
      ScanSample aligned_scan;
      if (!selectAlignedScan(scan_history, odom_history.back().stamp, aligned_scan)) {
        decision = stop("waiting_scan_odom_alignment");
      } else {
        scan = aligned_scan.scan;
        if (scan.header.frame_id != base_frame_) {
          decision = fault("scan_frame_mismatch");
        } else if ((ros_now - rclcpp::Time(scan.header.stamp)).seconds() < -0.02 ||
          (ros_now - rclcpp::Time(scan.header.stamp)).seconds() > scan_timeout_sec_)
        {
          decision = fault("scan_stamp_stale_or_future");
        } else {
          OdomSample scan_odom;
          if (!interpolateOdom(odom_history, scan.header.stamp, scan_odom)) {
            decision = stop("waiting_scan_odom_interpolation");
          } else {
            // Scan self-filtering and the stopping-trajectory geometry are
            // independent.  A pre-filtered scan must still be evaluated
            // against the live base-plus-arms projection.
            if (require_dynamic_footprint_) {
              logic_->setFootprint(dynamic_footprint.points);
            }
            auto points = currentObstaclePoints(scan, scan_odom, odom_history.back());
            if (require_dynamic_footprint_) {
              const auto first_self_return = std::remove_if(
                points.begin(), points.end(), [&dynamic_footprint](const Point2D & point) {
                  return strictlyInsidePolygon(point, dynamic_footprint.points);
                });
              self_filtered_scan_points = static_cast<size_t>(
                std::distance(first_self_return, points.end()));
              points.erase(first_self_return, points.end());
            }
            decision = logic_->evaluate(
              command.linear.x, command.angular.z,
              odom_history.back().linear_x, odom_history.back().angular_z, points);
          }
        }
      }
    }
    geometry_msgs::msg::Twist output;
    output.linear.x = decision.linear_x;
    output.angular.z = decision.angular_z;
    command_pub_->publish(output);
    publishStatus(
      decision, command, scan, odom_history, localization_health, scan_history.size(),
      command_available, command_received, source_command_available, source_command_valid,
      source_command_received, arm_state, dynamic_footprint, self_filtered_scan_points, steady_now);
  }

  void publishStatus(const Decision & decision, const geometry_msgs::msg::Twist & command,
    const sensor_msgs::msg::LaserScan & scan, const std::deque<OdomSample> & odom_history,
    const LocalizationHealth & localization_health, const size_t scan_history_size,
    const bool command_available, const SteadyTime & command_received,
    const bool source_command_available, const bool source_command_valid,
    const SteadyTime & source_command_received, const ArmState & arm_state,
    const DynamicFootprint & dynamic_footprint, const size_t self_filtered_scan_points,
    const SteadyTime & steady_now)
  {
    std_msgs::msg::String message;
    const bool dynamic_footprint_ready = dynamic_footprint.available &&
      std::chrono::duration<double>(steady_now - dynamic_footprint.received).count() <=
      dynamic_footprint_timeout_sec_;
    const double dynamic_footprint_age_sec = dynamic_footprint.available ?
      std::chrono::duration<double>(steady_now - dynamic_footprint.received).count() : -1.0;
    std::ostringstream payload;
    payload.setf(std::ios::fixed);
    payload.precision(6);
    payload << "{\"schema\":\"r1_collision_guard_status_v1\""
            << ",\"state\":\"" << stateName(decision.state) << "\""
            << ",\"reason\":\"" << decision.reason << "\""
            << ",\"requested_linear_x\":" << command.linear.x
            << ",\"requested_angular_z\":" << command.angular.z
            << ",\"output_linear_x\":" << decision.linear_x
            << ",\"output_angular_z\":" << decision.angular_z
            << ",\"safe_scale\":" << decision.safe_scale
            << ",\"stopping_distance_m\":" << decision.stopping_distance_m
            << ",\"collision_time_sec\":" << decision.collision_time_sec
            << ",\"collision_point_x\":" << decision.collision_point.x
            << ",\"collision_point_y\":" << decision.collision_point.y
            << ",\"localization_state\":\"" << localization_health.state << "\""
            << ",\"localization_reason\":\"" << localization_health.reason << "\""
            << ",\"arm_state\":\"" << (
              require_dynamic_footprint_ ?
              (dynamic_footprint_ready ? "DYNAMIC_PROJECTION_READY" : "DYNAMIC_PROJECTION_STALE") :
              (!arm_state.available ? "MISSING" :
              !arm_state.valid ? "INVALID" :
              !arm_state.stowed ? "NOT_STOWED" :
              arm_state.moving ? "MOVING" : "STOWED")) << "\""
            << ",\"arm_max_position_error_rad\":" << arm_state.max_position_error
            << ",\"dynamic_footprint_points\":" << dynamic_footprint.points.size()
            << ",\"dynamic_footprint_ready\":" << (dynamic_footprint_ready ? "true" : "false")
            << ",\"dynamic_footprint_age_sec\":" << dynamic_footprint_age_sec
            << ",\"mesh_self_filter_preapplied\":" <<
      (mesh_self_filter_preapplied_ ? "true" : "false")
            << ",\"self_filtered_scan_points\":" << self_filtered_scan_points
            << ",\"scan_history_size\":" << scan_history_size
            << ",\"scan_stamp_ns\":" << rclcpp::Time(scan.header.stamp).nanoseconds();
    if (command_available) {
      payload << ",\"command_age_sec\":" <<
        std::chrono::duration<double>(steady_now - command_received).count();
    }
    payload << ",\"source_command_ready\":" <<
      (source_command_available && source_command_valid ? "true" : "false");
    if (source_command_available) {
      payload << ",\"source_command_age_sec\":" <<
        std::chrono::duration<double>(steady_now - source_command_received).count();
    }
    if (!odom_history.empty()) {
      payload << ",\"actual_linear_x\":" << odom_history.back().linear_x
              << ",\"actual_angular_z\":" << odom_history.back().angular_z
              << ",\"odom_stamp_ns\":" << odom_history.back().stamp.nanoseconds();
    }
    payload << "}";
    message.data = payload.str();
    status_pub_->publish(message);
  }

  std::unique_ptr<GuardLogic> logic_;
  double control_frequency_{50.0};
  double scan_timeout_sec_{0.25};
  double odom_timeout_sec_{0.15};
  double command_timeout_sec_{0.20};
  double source_command_timeout_sec_{0.20};
  int scan_history_size_{32};
  double localization_health_timeout_sec_{0.50};
  double dynamic_footprint_timeout_sec_{0.50};
  double joint_state_timeout_sec_{0.50};
  double arm_stow_tolerance_rad_{0.035};
  double arm_motion_tolerance_radps_{0.020};
  double arm_settle_time_sec_{1.0};
  double max_scan_range_m_{8.0};
  bool require_localization_ready_{true};
  bool require_arms_stowed_{true};
  bool require_dynamic_footprint_{false};
  bool mesh_self_filter_preapplied_{false};
  std::string base_frame_;
  std::string odom_frame_;
  std::vector<std::string> arm_joint_names_;
  std::vector<double> arm_stowed_positions_;
  std::vector<std::string> arm_joint_state_topics_;
  std::mutex mutex_;
  geometry_msgs::msg::Twist command_;
  sensor_msgs::msg::LaserScan scan_;
  std::deque<ScanSample> scan_history_;
  std::deque<OdomSample> odom_history_;
  SteadyTime command_received_{};
  SteadyTime source_command_received_{};
  SteadyTime scan_received_{};
  bool command_available_{false};
  bool source_command_available_{false};
  bool source_command_valid_{false};
  bool scan_available_{false};
  bool localization_ready_seen_{false};
  LocalizationHealth localization_health_;
  ArmState arm_state_;
  DynamicFootprint dynamic_footprint_;
  std::unordered_map<std::string, double> arm_positions_;
  std::unordered_map<std::string, SteadyTime> arm_joint_received_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr source_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_health_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> arm_joint_state_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr dynamic_footprint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace r1_collision_guard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r1_collision_guard::CollisionGuardNode>());
  rclcpp::shutdown();
  return 0;
}
