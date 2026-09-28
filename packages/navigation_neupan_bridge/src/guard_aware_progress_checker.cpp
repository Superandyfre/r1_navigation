// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include "navigation_neupan_bridge/guard_aware_progress_checker.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace navigation_neupan_bridge
{

void GuardAwareProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  const auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("unable to lock controller server for progress checker");
  }
  clock_ = node->get_clock();
  const auto guard_status_topic = node->declare_parameter<std::string>(
    plugin_name + ".guard_status_topic", "/r1_collision_guard/status");
  guard_block_reason_ = node->declare_parameter<std::string>(
    plugin_name + ".guard_block_reason", "immediate_braking_trajectory_blocked");
  guard_status_timeout_sec_ = node->declare_parameter<double>(
    plugin_name + ".guard_status_timeout_sec", 0.20);
  command_epsilon_ = node->declare_parameter<double>(
    plugin_name + ".command_epsilon", 0.005);
  required_movement_radius_ = node->declare_parameter<double>(
    plugin_name + ".required_movement_radius", 0.01);
  required_movement_angle_ = node->declare_parameter<double>(
    plugin_name + ".required_movement_angle", 0.0175);
  movement_time_allowance_sec_ = node->declare_parameter<double>(
    plugin_name + ".movement_time_allowance", 60.0);
  max_guarded_block_time_sec_ = node->declare_parameter<double>(
    plugin_name + ".max_guarded_block_time_sec", 300.0);
  if (guard_status_timeout_sec_ <= 0.0 || command_epsilon_ < 0.0 ||
    required_movement_radius_ < 0.0 || required_movement_angle_ < 0.0 ||
    movement_time_allowance_sec_ <= 0.0 || max_guarded_block_time_sec_ < 0.0)
  {
    throw std::invalid_argument("invalid guard-aware progress checker configuration");
  }
  guard_status_sub_ = node->create_subscription<std_msgs::msg::String>(
    guard_status_topic, rclcpp::QoS(20),
    std::bind(&GuardAwareProgressChecker::guardStatusCallback, this, std::placeholders::_1));
}

void GuardAwareProgressChecker::guardStatusCallback(const std_msgs::msg::String::SharedPtr message)
{
  GuardStatus status;
  try {
    const auto payload = nlohmann::json::parse(message->data);
    if (payload.value("schema", "") != "r1_collision_guard_status_v1") {
      throw std::runtime_error("schema_invalid");
    }
    status.state = payload.at("state").get<std::string>();
    status.reason = payload.at("reason").get<std::string>();
    status.requested_linear_x = payload.at("requested_linear_x").get<double>();
    status.requested_angular_z = payload.at("requested_angular_z").get<double>();
    status.output_linear_x = payload.at("output_linear_x").get<double>();
    status.output_angular_z = payload.at("output_angular_z").get<double>();
    status.valid = std::isfinite(status.requested_linear_x) &&
      std::isfinite(status.requested_angular_z) && std::isfinite(status.output_linear_x) &&
      std::isfinite(status.output_angular_z);
  } catch (const std::exception &) {
    status.valid = false;
  }
  status.received = clock_->now();
  std::lock_guard<std::mutex> lock(mutex_);
  guard_status_ = std::move(status);
}

bool GuardAwareProgressChecker::isGuardBlocking(const rclcpp::Time & now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!guard_status_.valid || guard_status_.received.nanoseconds() == 0 ||
    (now - guard_status_.received).seconds() > guard_status_timeout_sec_)
  {
    return false;
  }
  return guard_status_.state == "STOP" && guard_status_.reason == guard_block_reason_ &&
    (std::abs(guard_status_.requested_linear_x) > command_epsilon_ ||
    std::abs(guard_status_.requested_angular_z) > command_epsilon_) &&
    std::abs(guard_status_.output_linear_x) <= command_epsilon_ &&
    std::abs(guard_status_.output_angular_z) <= command_epsilon_;
}

geometry_msgs::msg::Pose2D GuardAwareProgressChecker::toPose2D(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::Pose2D result;
  result.x = pose.pose.position.x;
  result.y = pose.pose.position.y;
  const auto & q = pose.pose.orientation;
  result.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return result;
}

double GuardAwareProgressChecker::normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool GuardAwareProgressChecker::movedEnough(const geometry_msgs::msg::Pose2D & pose) const
{
  return std::hypot(pose.x - baseline_pose_.x, pose.y - baseline_pose_.y) >
    required_movement_radius_ || std::abs(normalizeAngle(pose.theta - baseline_pose_.theta)) >
    required_movement_angle_;
}

void GuardAwareProgressChecker::resetBaseline(
  const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & now)
{
  baseline_pose_ = pose;
  baseline_time_ = now;
  baseline_pose_set_ = true;
}

bool GuardAwareProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto now = clock_->now();
  const auto pose = toPose2D(current_pose);
  if (isGuardBlocking(now)) {
    if (!guard_blocked_since_set_) {
      guard_blocked_since_ = now;
      guard_blocked_since_set_ = true;
    }
    if (max_guarded_block_time_sec_ > 0.0 &&
      (now - guard_blocked_since_).seconds() > max_guarded_block_time_sec_)
    {
      return false;
    }
    resetBaseline(pose, now);
    return true;
  }
  guard_blocked_since_set_ = false;
  if (!baseline_pose_set_ || movedEnough(pose)) {
    resetBaseline(pose, now);
    return true;
  }
  return (now - baseline_time_).seconds() <= movement_time_allowance_sec_;
}

void GuardAwareProgressChecker::reset()
{
  baseline_pose_set_ = false;
  guard_blocked_since_set_ = false;
}

}  // namespace navigation_neupan_bridge

PLUGINLIB_EXPORT_CLASS(
  navigation_neupan_bridge::GuardAwareProgressChecker, nav2_core::ProgressChecker)
