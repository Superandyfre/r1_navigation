// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
// Isolated fault injection only: block the Action thread, not runtime timers.
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include "navigation_neupan_bridge/neupan_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation_neupan_bridge
{
class ActionStallController : public NeuPANController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    auto matches = [](const char * key, const std::string & expected) {
        const char * value = std::getenv(key); return value && value == expected;
      };
    if (!matches("NEUPAN_ISOLATED_ROS_TEST", "1") || !matches("ROS_DOMAIN_ID", "192") ||
      !matches("ROS_LOCALHOST_ONLY", "1"))
    {throw std::runtime_error("action_stall_adapter_isolated_test_only");}
    NeuPANController::configure(parent, name, tf, costmap);
    ack_ = parent.lock()->create_publisher<std_msgs::msg::String>(
      "/test_failstop_chain/action_stall_ack", 10);
    sub_ = parent.lock()->create_subscription<std_msgs::msg::Empty>(
      "/test_failstop_chain/action_stall", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {stall_.store(true);});
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override
  {
    if (stall_.exchange(false)) {
      std_msgs::msg::String message; message.data = "paused"; ack_->publish(message);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      message.data = "resumed"; ack_->publish(message);
    }
    return NeuPANController::computeVelocityCommands(pose, velocity, checker);
  }
  void cleanup() override
  {
    sub_.reset(); ack_.reset(); stall_.store(false);
    NeuPANController::cleanup();
  }
private:
  std::atomic<bool> stall_{false};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_;
};
}
PLUGINLIB_EXPORT_CLASS(navigation_neupan_bridge::ActionStallController, nav2_core::Controller)
