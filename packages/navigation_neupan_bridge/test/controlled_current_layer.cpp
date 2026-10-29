// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
// Test-only current-state control; no obstacle/cost modification.
#include <atomic>
#include <cstdlib>
#include <stdexcept>
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/bool.hpp"

namespace navigation_neupan_bridge
{
class ControlledCurrentLayer : public nav2_costmap_2d::Layer
{
public:
  ControlledCurrentLayer() {requireIsolated();}
  static void requireIsolated()
  {
    auto matches = [](const char * key, const std::string & expected) {
        const char * value = std::getenv(key); return value && value == expected;
      };
    if (!matches("NEUPAN_ISOLATED_ROS_TEST", "1") || !matches("ROS_DOMAIN_ID", "192") ||
      !matches("ROS_LOCALHOST_ONLY", "1"))
    {throw std::runtime_error("controlled_current_layer_isolated_test_only");}
  }
  void onInitialize() override
  {
    requireIsolated();
    current_ = true; enabled_ = true;
    auto node = node_.lock();
    ack_ = node->create_publisher<std_msgs::msg::Bool>(
      "/test_failstop_chain/costmap_noncurrent_ack", 10);
    sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/test_failstop_chain/costmap_noncurrent", 10,
      [this](std_msgs::msg::Bool::SharedPtr message) {stale_.store(message->data);});
  }
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override
  {
    current_ = !stale_.load();
    std_msgs::msg::Bool message; message.data = !current_; ack_->publish(message);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
private:
  std::atomic<bool> stale_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ack_;
};
}
PLUGINLIB_EXPORT_CLASS(navigation_neupan_bridge::ControlledCurrentLayer, nav2_costmap_2d::Layer)
