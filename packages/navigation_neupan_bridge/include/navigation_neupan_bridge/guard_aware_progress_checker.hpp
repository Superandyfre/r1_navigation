// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#ifndef NAVIGATION_NEUPAN_BRIDGE__GUARD_AWARE_PROGRESS_CHECKER_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__GUARD_AWARE_PROGRESS_CHECKER_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation_neupan_bridge
{

class GuardAwareProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
  void reset() override;

private:
  struct GuardStatus
  {
    std::string state;
    std::string reason;
    double requested_linear_x{0.0};
    double requested_angular_z{0.0};
    double output_linear_x{0.0};
    double output_angular_z{0.0};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void guardStatusCallback(const std_msgs::msg::String::SharedPtr message);
  bool isGuardBlocking(const rclcpp::Time & now) const;
  static geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::PoseStamped & pose);
  static double normalizeAngle(double angle);
  bool movedEnough(const geometry_msgs::msg::Pose2D & pose) const;
  void resetBaseline(const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & now);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_status_sub_;
  mutable std::mutex mutex_;
  GuardStatus guard_status_;
  std::string guard_block_reason_{"immediate_braking_trajectory_blocked"};
  double guard_status_timeout_sec_{0.20};
  double command_epsilon_{0.005};
  double required_movement_radius_{0.01};
  double required_movement_angle_{0.0175};
  double movement_time_allowance_sec_{60.0};
  double max_guarded_block_time_sec_{300.0};
  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_{0, 0, RCL_ROS_TIME};
  bool baseline_pose_set_{false};
  rclcpp::Time guard_blocked_since_{0, 0, RCL_ROS_TIME};
  bool guard_blocked_since_set_{false};
};

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__GUARD_AWARE_PROGRESS_CHECKER_HPP_
