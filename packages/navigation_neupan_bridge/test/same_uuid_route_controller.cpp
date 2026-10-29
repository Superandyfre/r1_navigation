// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
// Test-only adapter: insert setPlan on the real control thread without issuing
// a second Action UUID. All authorization and command logic stays in the base.
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include "navigation_neupan_bridge/neupan_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigation_neupan_bridge
{
class SameUUIDRouteController : public NeuPANController
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
    {throw std::runtime_error("same_uuid_adapter_isolated_test_only");}
    NeuPANController::configure(parent, name, tf, costmap);
    sub_ = parent.lock()->create_subscription<nav_msgs::msg::Path>(
      "/test_failstop_chain/route_update", 10,
      [this](nav_msgs::msg::Path::SharedPtr path) {
        std::lock_guard<std::mutex> lock(route_mutex_); pending_ = *path;
      });
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override
  {
    std::optional<nav_msgs::msg::Path> route;
    {std::lock_guard<std::mutex> lock(route_mutex_); route.swap(pending_);}
    if (route) {NeuPANController::setPlan(*route);}
    return NeuPANController::computeVelocityCommands(pose, velocity, checker);
  }
  void cleanup() override
  {
    sub_.reset();
    {std::lock_guard<std::mutex> lock(route_mutex_); pending_.reset();}
    NeuPANController::cleanup();
  }
private:
  std::mutex route_mutex_;
  std::optional<nav_msgs::msg::Path> pending_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
};
}
PLUGINLIB_EXPORT_CLASS(navigation_neupan_bridge::SameUUIDRouteController, nav2_core::Controller)
