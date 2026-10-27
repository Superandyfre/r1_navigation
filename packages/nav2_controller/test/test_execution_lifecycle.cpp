// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include "gtest/gtest.h"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_controller/controller_server.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
namespace
{
class ObservedController : public nav2_core::Controller,
  public r1_neupan_runtime::ExecutionObserver
{
public:
  bool fail_begin{false}, fail_path{true};
  std::atomic<bool> fatal{false};
  std::atomic<int> checks{0};
  std::mutex mutex;
  std::vector<std::string> events;
  std::string authority, attempt;
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setSpeedLimit(const double &, const bool &) override {}
  void setPlan(const nav_msgs::msg::Path &) override
  {
    std::lock_guard<std::mutex> lock(mutex);events.push_back("plan");
    if (fail_path) {throw nav2_core::PlannerException("injected_path_install_failure");}
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override {return geometry_msgs::msg::TwistStamped();}
  void beginExecution(const std::string & a, const std::string & b) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    authority = a;attempt = b;events.push_back("begin");
    if (fail_begin) {throw nav2_core::PlannerException("injected_begin_failure");}
  }
  void checkExecution(const std::string &, const std::string &) override
  {
    ++checks;
    if (fatal) {throw nav2_core::PlannerException("injected_worker_fatal");}
  }
  void endExecution(
    const std::string & a, const std::string & b,
    const std::string & reason) noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(a == authority && b == attempt ? "end:" + reason : "WRONG_ID");
  }
};
class NoncurrentLayer : public nav2_costmap_2d::Layer
{
public:
  NoncurrentLayer() {current_ = false;enabled_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
};
class ActionServerShim : public nav2_controller::ControllerServer
{
public:
  ActionServerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true)})) {}
  void start(const std::shared_ptr<ObservedController> & observer, bool costmap_wait = false)
  {
    controllers_["Observed"] = observer;
    goal_checkers_["goal"] = std::make_shared<nav2_controller::SimpleGoalChecker>();
    progress_checker_ = std::make_shared<nav2_controller::SimpleProgressChecker>();
    controller_frequency_ = 20.;
    if (costmap_wait) {
      runtime_costmap_wait_timeout_sec_ = 0.3;
      costmap_ros_->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
      costmap_ros_->configure();
      costmap_ros_->getLayeredCostmap()->addPlugin(std::make_shared<NoncurrentLayer>());
    }
    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/test_wp3/raw_cmd", 10);
    vel_publisher_->on_activate();
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(), "test_wp3/follow_path",
      std::bind(&ActionServerShim::computeControl, this), nullptr, 500ms, true);
    action_server_->activate();
  }
  void stop() {action_server_->deactivate();}
};
class LifecycleAction : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!std::getenv("NEUPAN_ISOLATED_ROS_TEST")) {GTEST_SKIP() << "opt-in isolated Action test";}
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "189");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
  void exercise(bool fail_begin, bool empty)
  {
    auto observer = std::make_shared<ObservedController>();observer->fail_begin = fail_begin;
    auto server = std::make_shared<ActionServerShim>();server->start(observer);
    auto client_node = std::make_shared<rclcpp::Node>("wp3_lifecycle_client");
    using Follow = nav2_msgs::action::FollowPath;
    auto client = rclcpp_action::create_client<Follow>(client_node, "test_wp3/follow_path");
    ASSERT_TRUE(client->wait_for_action_server(3s));
    Follow::Goal goal;goal.controller_id = "Observed";goal.goal_checker_id = "goal";
    goal.path.header.frame_id = "map";
    if (!empty) {goal.path.poses.resize(1);goal.path.poses[0].pose.orientation.w = 1.;}
    auto sent = client->async_send_goal(goal);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        client_node, sent,
        3s), rclcpp::FutureReturnCode::SUCCESS);
    auto handle = sent.get();ASSERT_NE(handle, nullptr);
    auto result = client->async_get_result(handle);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        client_node, result,
        3s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
    server->stop();
    std::ostringstream expected;expected << std::hex << std::setfill('0');
    for (auto byte : handle->get_goal_id()) {
      expected << std::setw(2) << static_cast<unsigned>(byte);
    }
    std::lock_guard<std::mutex> lock(observer->mutex);
    EXPECT_EQ(observer->attempt, expected.str());EXPECT_FALSE(observer->authority.empty());
    std::vector<std::string> events{"begin"};
    if (!fail_begin && !empty) {events.push_back("plan");}
    events.push_back(fail_begin ? "end:begin_failed" : "end:failed");
    EXPECT_EQ(observer->events, events);
  }
  void waitScenario(const std::string & mode)
  {
    auto observer = std::make_shared<ObservedController>();observer->fail_path = false;
    auto server = std::make_shared<ActionServerShim>();server->start(observer, true);
    ASSERT_EQ(server->now().nanoseconds(), 0);
    auto node = std::make_shared<rclcpp::Node>("wp3_costmap_client");
    using Follow = nav2_msgs::action::FollowPath;
    auto client = rclcpp_action::create_client<Follow>(node, "test_wp3/follow_path");
    ASSERT_TRUE(client->wait_for_action_server(3s));
    Follow::Goal goal;goal.controller_id = "Observed";goal.goal_checker_id = "goal";
    goal.path.header.frame_id = "map";goal.path.poses.resize(1);
    auto sent = client->async_send_goal(goal);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node, sent, 3s),
      rclcpp::FutureReturnCode::SUCCESS);
    auto handle = sent.get();ASSERT_NE(handle, nullptr);
    const auto wait_limit = std::chrono::steady_clock::now() + 1s;
    while (observer->checks.load() < 2 && std::chrono::steady_clock::now() < wait_limit) {
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GE(observer->checks.load(), 2);
    auto started = std::chrono::steady_clock::now();
    auto result = client->async_get_result(handle);
    if (mode == "fatal") {observer->fatal = true;}
    if (mode == "cancel") {
      auto canceled = client->async_cancel_goal(handle);
      ASSERT_EQ(
        rclcpp::spin_until_future_complete(
          node, canceled,
          1s), rclcpp::FutureReturnCode::SUCCESS);
    }
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        node, result,
        1s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(
      result.get().code, mode == "cancel" ?
      rclcpp_action::ResultCode::CANCELED : rclcpp_action::ResultCode::ABORTED);
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    EXPECT_LT(elapsed, 1.0);
    std::cout << "scenario=" << mode << " action_terminal_sec=" << elapsed << std::endl;
    server->stop();
    std::lock_guard<std::mutex> lock(observer->mutex);
    ASSERT_EQ(observer->events.size(), 3u);
    EXPECT_EQ(observer->events.back(), mode == "cancel" ? "end:canceled" : "end:failed");
  }

  void preemptDuringWait(bool invalid_controller)
  {
    auto observer = std::make_shared<ObservedController>();observer->fail_path = false;
    auto server = std::make_shared<ActionServerShim>();server->start(observer, true);
    ASSERT_EQ(server->now().nanoseconds(), 0);
    auto node = std::make_shared<rclcpp::Node>("wp3_preempt_client");
    using Follow = nav2_msgs::action::FollowPath;
    auto client = rclcpp_action::create_client<Follow>(node, "test_wp3/follow_path");
    ASSERT_TRUE(client->wait_for_action_server(3s));
    Follow::Goal goal;goal.controller_id = "Observed";goal.goal_checker_id = "goal";
    goal.path.header.frame_id = "map";goal.path.poses.resize(1);
    auto first = client->async_send_goal(goal);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node, first, 3s),
      rclcpp::FutureReturnCode::SUCCESS);
    auto first_handle = first.get();ASSERT_NE(first_handle, nullptr);
    auto first_result = client->async_get_result(first_handle);
    auto limit = std::chrono::steady_clock::now() + 1s;
    while (observer->checks.load() < 2 && std::chrono::steady_clock::now() < limit) {
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GE(observer->checks.load(), 2);
    if (invalid_controller) {goal.controller_id = "invalid";}
    auto second = client->async_send_goal(goal);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        node, second,
        1s), rclcpp::FutureReturnCode::SUCCESS);
    auto second_handle = second.get();ASSERT_NE(second_handle, nullptr);
    auto second_result = client->async_get_result(second_handle);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        node, first_result,
        1s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::ABORTED);
    if (!invalid_controller) {
      limit = std::chrono::steady_clock::now() + 1s;
      bool ready = false;
      while (std::chrono::steady_clock::now() < limit) {
        {std::lock_guard<std::mutex> lock(observer->mutex);ready = observer->events.size() >= 5;}
        if (ready) {break;}
        std::this_thread::sleep_for(5ms);
      }
      ASSERT_TRUE(ready);
      auto cancel = client->async_cancel_goal(second_handle);
      ASSERT_EQ(
        rclcpp::spin_until_future_complete(
          node, cancel,
          1s), rclcpp::FutureReturnCode::SUCCESS);
    }
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        node, second_result,
        1s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(
      second_result.get().code, invalid_controller ?
      rclcpp_action::ResultCode::ABORTED : rclcpp_action::ResultCode::CANCELED);
    server->stop();
    std::lock_guard<std::mutex> lock(observer->mutex);
    std::vector<std::string> expected{"begin", "plan", "end:preempted"};
    if (!invalid_controller) {
      expected.insert(expected.end(), {"begin", "plan", "end:canceled"});
      std::ostringstream id;id << std::hex << std::setfill('0');
      for (auto byte : second_handle->get_goal_id()) {
        id << std::setw(2) << static_cast<unsigned>(byte);
      }
      EXPECT_EQ(observer->attempt, id.str());
      EXPECT_NE(first_handle->get_goal_id(), second_handle->get_goal_id());
    }
    EXPECT_EQ(observer->events, expected);
  }
};
TEST_F(LifecycleAction, RealActionUuidAndPathFailureRevokeExactExecution) {
  exercise(false, false);
}
TEST_F(LifecycleAction, PartialBeginFailureReleasesExactExecutionOnce) {
  exercise(true, false);
}
TEST_F(LifecycleAction, EmptyPathAbortsAndReleasesExecution) {
  exercise(false, true);
}
TEST_F(LifecycleAction, FatalInterruptsRealNoncurrentCostmapWithFrozenRosClock) {
  waitScenario("fatal");
}
TEST_F(LifecycleAction, CancelInterruptsRealNoncurrentCostmapWithFrozenRosClock) {
  waitScenario("cancel");
}
TEST_F(LifecycleAction, NoncurrentCostmapHasWallDeadlineWithFrozenRosClock) {
  waitScenario("deadline");
}
TEST_F(LifecycleAction, SamePathPreemptionDuringCostmapWaitRetiresOldUuid) {
  preemptDuringWait(false);
}
TEST_F(LifecycleAction, InvalidPreemptingControllerAbortsWithoutOldCommandContinuation) {
  preemptDuringWait(true);
}
}  // namespace
