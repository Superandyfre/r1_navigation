// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cstdlib>
#include <future>
#include <limits>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "navigation_neupan_bridge/neupan_controller.hpp"

namespace
{
using Json = nlohmann::json;
using namespace std::chrono_literals;
double monotonicNow()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Actual plugin and DDS callbacks. Worker/Guard peers are synthetic here;
// no ActionServer, solver, driver, or final velocity publisher is created.
TEST(NeuPANExecutionROS, RealCallbacksHandshakeThenFatalRetiresContext)
{
  if (!std::getenv("NEUPAN_ISOLATED_ROS_TEST")) {GTEST_SKIP() << "isolated opt-in only";}
  ASSERT_STREQ(std::getenv("NEUPAN_ISOLATED_ROS_TEST"), "1");
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "191");
  ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
  rclcpp::init(0, nullptr);
  const std::string prefix = "/test_wp3_plugin/";
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params{
    {"FollowPath.shadow_mode", false}, {"FollowPath.require_localization_ready", false},
    {"FollowPath.require_gate_match", false}, {"FollowPath.navigation_frame", "map"}};
  for (const auto & name : {"path_topic", "goal_topic", "envelope_topic", "gate_command_topic",
      "gate_status_topic", "guard_status_topic", "localization_health_topic", "diagnostics_topic",
      "worker_health_topic", "execution_context_topic", "motion_grant_topic", "motion_arm_service",
      "odom_topic"})
  {
    params.emplace_back(std::string("FollowPath.") + name, prefix + name);
  }
  options.parameter_overrides(params);
  auto parent =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_neupan_controller", options);
  auto peer = std::make_shared<rclcpp::Node>("test_neupan_peer");
  navigation_neupan_bridge::NeuPANController plugin;
  plugin.configure(parent, "FollowPath", nullptr, nullptr);
  parent->configure(); parent->activate(); plugin.activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent->get_node_base_interface()); executor.add_node(peer);
  auto guard_pub = peer->create_publisher<std_msgs::msg::String>(prefix + "guard_status_topic", 20);
  auto health_pub =
    peer->create_publisher<std_msgs::msg::String>(prefix + "worker_health_topic", 20);
  auto source_pub = peer->create_publisher<std_msgs::msg::String>(prefix + "envelope_topic", 20);
  auto odom_pub = peer->create_publisher<nav_msgs::msg::Odometry>(prefix + "odom_topic", 20);
  Json context, grant;
  std::vector<Json> contexts;
  auto context_sub = peer->create_subscription<std_msgs::msg::String>(
    prefix + "execution_context_topic", 20,
    [&](std_msgs::msg::String::SharedPtr msg) {
      context = Json::parse(msg->data); contexts.push_back(context);
    });
  auto grant_sub = peer->create_subscription<std_msgs::msg::String>(
    prefix + "motion_grant_topic", 20,
    [&](std_msgs::msg::String::SharedPtr msg) {grant = Json::parse(msg->data);});
  auto client = peer->create_client<std_srvs::srv::Trigger>(prefix + "motion_arm_service");
  auto publish = [](const auto & pub, const Json & j) {
      std_msgs::msg::String msg; msg.data = j.dump(); pub->publish(msg);
    };
  int sequence = 0;
  int odom_fault = 1;
  const auto frozen_stamp = peer->now();
  auto pulse = [&](bool fatal = false) {
      const double now = monotonicNow();
      Json status{{"schema", "r1_collision_guard_status_v1"},
        {"authorization_protocol", r1_neupan_runtime::kGrantSchema},
        {"source_protocol", r1_neupan_runtime::kSourceSchema}, {"guard_boot_id", "test-guard"},
        {"sequence", ++sequence}, {"monotonic", now}, {"authorization_state", "DISARMED"},
        {"physical_safety_state", "PASS"}, {"command_barrier_ready", true}, {"output_linear_x", 0.}, {"output_angular_z", 0.},
        {"state", "PASS"}, {"reason", "clear"}};
      if (!grant.is_null()) {
        for (const auto & key :
          {"grant_id", "authority_boot_id", "controller_boot_id", "navigation_attempt_id",
            "worker_boot_id", "worker_generation", "path_version", "goal_version"})
        {
          status[key] = grant.at(key);
        }
        status["authorization_state"] = grant.at("state") == "COMMIT" ? "ARMED" : "PREPARED";
      }
      publish(guard_pub, status);
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = peer->now() - rclcpp::Duration::from_seconds(odom_fault == 1 ? 2. : 0.);
      if (odom_fault == 4) {odom.header.stamp = frozen_stamp;}
      odom.header.frame_id = odom_fault == 2 ? "map" : "odom";
      odom.child_frame_id = "base_link";
      if (odom_fault == 3) {odom.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();}
      odom_pub->publish(odom);
      if (!context.is_null() && context.at("state") == "ACTIVE") {
        Json health{{"schema", r1_neupan_runtime::kHealthSchema}, {"boot_id", "test-worker"},
          {"worker_generation", 1}, {"sequence", sequence}, {"monotonic", now},
          {"state", fatal ? "FAULT_LATCHED" : "RUNNING"},
          {"navigation_attempt_id", context.at("navigation_attempt_id")},
          {"path_version", context.at("path_version")},
          {"goal_version", context.at("goal_version")}};
        if (fatal) {health["fault_id"] = "test-fault"; health["fault_code"] = "worker_crash";}
        publish(health_pub, health);
        if (!fatal) {
          Json source{{"schema", r1_neupan_runtime::kSourceSchema},
            {"worker_boot_id", "test-worker"},
            {"worker_generation", 1}, {"request_id", sequence}, {"cycle_id", sequence},
            {"scan_counter", sequence},
            {"scan_receive_monotonic", now - .01}, {"request_dispatched_monotonic", now - .008},
            {"worker_solve_end_monotonic", now - .002}, {"planner_result_monotonic", now},
            {"finite", true}, {"command", {{"linear_x", .1}, {"angular_z", 0.}}}};
          for (const auto & key :
            {"authority_boot_id", "controller_boot_id", "navigation_attempt_id", "path_version",
              "goal_version"})
          {
            source[key] = context.at(key);
          }
          publish(source_pub, source);
        }
      }
      executor.spin_some(); std::this_thread::sleep_for(10ms); executor.spin_some();
    };
  // Discovery and explicit arm use only isolated test topics and this service.
  for (int i = 0; i < 40; ++i) {
    pulse();
  }
  ASSERT_TRUE(client->service_is_ready());
  auto arm = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int i = 0; i < 40 && arm.wait_for(0ms) != std::future_status::ready; ++i) {
    pulse();
  }
  ASSERT_EQ(arm.wait_for(0ms), std::future_status::ready);
  EXPECT_FALSE(arm.get()->success) << "fresh receipt cannot wash out stale source stamp";
  for (odom_fault = 2; odom_fault <= 4; ++odom_fault) {
    for (int i = 0; i < 40; ++i) {
      pulse();
    }
    arm = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    for (int i = 0; i < 40 && arm.wait_for(0ms) != std::future_status::ready; ++i) {
      pulse();
    }
    ASSERT_EQ(arm.wait_for(0ms), std::future_status::ready);
    EXPECT_FALSE(arm.get()->success) << "invalid odometry mode " << odom_fault;
  }
  odom_fault = 0;
  for (int i = 0; i < 40; ++i) {
    pulse();
  }
  arm = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  for (int i = 0; i < 40 && arm.wait_for(0ms) != std::future_status::ready; ++i) {
    pulse();
  }
  ASSERT_EQ(arm.wait_for(0ms), std::future_status::ready); ASSERT_TRUE(arm.get()->success);
  const auto changed = parent->set_parameter(rclcpp::Parameter("FollowPath.shadow_mode", true));
  EXPECT_FALSE(changed.successful) << "authorization mode must be immutable";
  // Keep this failed-first harness usable on the old implementation too.
  EXPECT_TRUE(parent->set_parameter(rclcpp::Parameter("FollowPath.shadow_mode", false)).successful);
  const auto timeout_changed = parent->set_parameter(
    rclcpp::Parameter("FollowPath.navigation_authority_timeout", 10.));
  EXPECT_FALSE(timeout_changed.successful) << "lease parameter must not drift from runtime";
  parent->set_parameter(rclcpp::Parameter("FollowPath.navigation_authority_timeout", .5));
  plugin.beginExecution("test-authority", "test-attempt");
  nav_msgs::msg::Path path; path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose; pose.header = path.header; pose.pose.orientation.w = 1.;
  path.poses.push_back(pose); pose.pose.position.x = 1.; path.poses.push_back(pose);
  plugin.setPlan(path);
  auto zero = plugin.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
  EXPECT_DOUBLE_EQ(zero.twist.linear.x, 0.);
  bool moved = false;
  for (int i = 0; i < 40 && !moved; ++i) {
    plugin.checkExecution("test-authority", "test-attempt"); pulse();
    moved =
      plugin.computeVelocityCommands(
      pose, geometry_msgs::msg::Twist(),
      nullptr).twist.linear.x > 0.;
  }
  ASSERT_TRUE(moved); ASSERT_FALSE(grant.is_null()); EXPECT_EQ(grant.at("state"), "COMMIT");
  EXPECT_EQ(context.at("path").size(), 2u);
  EXPECT_EQ(context.at("goal"), context.at("path").back());
  pulse(true);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some(); std::this_thread::sleep_for(10ms);
  }
  try {
    plugin.checkExecution("test-authority", "test-attempt"); FAIL() << "FATAL must fail";
  } catch (const std::exception & e) {
    EXPECT_STREQ(e.what(), "worker_fault_latched");
  }
  EXPECT_THROW(
    plugin.computeVelocityCommands(
      pose,
      geometry_msgs::msg::Twist(), nullptr), std::exception);
  EXPECT_TRUE(
    std::any_of(
      contexts.begin(), contexts.end(), [](const Json & j) {
        return j.at("state") == "END";
      }));
  EXPECT_THROW(plugin.beginExecution("test-authority", "another-attempt"), std::exception);
  plugin.deactivate(); parent->deactivate(); plugin.cleanup();
  EXPECT_FALSE(parent->set_parameter(rclcpp::Parameter("FollowPath.shadow_mode", true)).successful);
  // Same configuration may be recreated, but cleanup/configure cannot clear FATAL.
  plugin.configure(parent, "FollowPath", nullptr, nullptr);
  parent->activate(); plugin.activate();
  EXPECT_THROW(plugin.beginExecution("test-authority", "after-reconfigure"), std::exception);
  plugin.deactivate(); parent->deactivate(); plugin.cleanup();
  executor.remove_node(parent->get_node_base_interface()); executor.remove_node(peer);
  rclcpp::shutdown();
}
}  // namespace
