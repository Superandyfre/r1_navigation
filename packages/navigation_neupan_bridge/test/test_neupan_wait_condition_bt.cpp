// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

TEST(NeuPANWaitBehaviorTree, KeepsAcceptedPathWhenCandidateFails)
{
  rclcpp::init(0, nullptr);
  {
    BT::BehaviorTreeFactory factory;
    ASSERT_NO_THROW(factory.registerFromPlugin(WAIT_PLUGIN_PATH));
    auto blackboard = BT::Blackboard::create();
    auto node = std::make_shared<rclcpp::Node>("test_neupan_wait_condition");
    blackboard->set("node", node);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 2.0;
    goal.pose.orientation.w = 1.0;
    nav_msgs::msg::Path candidate;
    candidate.header.frame_id = "map";
    candidate.poses.push_back(goal);
    blackboard->set("goal", goal);
    blackboard->set("candidate_path", candidate);

    const auto xml = R"(
      <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <AcceptNeuPANPath candidate_path="{candidate_path}" goal="{goal}"
            path="{path}" accepted_goal="{accepted_goal}"/>
        </BehaviorTree>
      </root>)";
    auto accept = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(accept.tickRoot(), BT::NodeStatus::SUCCESS);
    blackboard->set("candidate_path", nav_msgs::msg::Path{});
    EXPECT_EQ(accept.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(blackboard->get<nav_msgs::msg::Path>("path").poses.size(), 1u);

    const auto wait_xml = R"(
      <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <IsNeuPANWaiting path="{path}" goal="{goal}" accepted_goal="{accepted_goal}"
            semantic_topic="/test_neupan_wait_semantics"/>
        </BehaviorTree>
      </root>)";
    auto wait = factory.createTreeFromText(wait_xml, blackboard);
    EXPECT_EQ(wait.tickRoot(), BT::NodeStatus::FAILURE);
    auto publisher = node->create_publisher<std_msgs::msg::String>(
      "/test_neupan_wait_semantics", 10);
    std_msgs::msg::String semantic;
    semantic.data = R"({"obstacle_motion":{"enabled":true,"state":"WAIT"}})";
    bool waiting = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      publisher->publish(semantic);
      if (wait.tickRoot() == BT::NodeStatus::SUCCESS) {
        waiting = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_TRUE(waiting);

    goal.pose.position.x = 3.0;
    blackboard->set("goal", goal);
    EXPECT_EQ(wait.tickRoot(), BT::NodeStatus::FAILURE);
  }
  rclcpp::shutdown();
}

// Real Nav2 RecoveryNode plus the loaded production retry plugin. The fake
// Follow action fails once; count whether recovery creates a second attempt.
TEST(NeuPANRetryBehaviorTree, FreshAuthorityResponseControlsRetry) {
  if (!std::getenv("NEUPAN_ISOLATED_ROS_TEST")) {GTEST_SKIP();}
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("test_retry_authority");
    bool permit = false;
    std::string reply = "neupan_retry_denied";
    auto server = node->create_service<std_srvs::srv::Trigger>("/test_retry_authority",
      [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = permit; response->message = reply;
      });
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(WAIT_PLUGIN_PATH);
    factory.registerFromPlugin(RECOVERY_PLUGIN_PATH);
    int follows = 0, replans = 0;
    factory.registerSimpleAction("TestFollow", [&](BT::TreeNode &) {
      return ++follows == 1 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("TestReplan", [&](BT::TreeNode &) {
      ++replans; return BT::NodeStatus::SUCCESS;
    });
    auto bb = BT::Blackboard::create(); bb->set("node", node);
    // Freeze ROS time: the retry timeout must still terminate using steady time.
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    for (int scene = 0; scene < 6; ++scene) {
      permit = scene != 0;
      reply = scene == 1 ? "wrong_reply" : "neupan_retry_permitted";
      follows = 0; replans = 0;
      auto tree = factory.createTreeFromText(R"(
        <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
          <RecoveryNode number_of_retries="1"><TestFollow/>
            <Sequence><CanRetryNeuPANExecution service_name="/test_retry_authority" timeout_sec="0.15"/>
              <TestReplan/></Sequence>
          </RecoveryNode>
        </BehaviorTree></root>)", bb);
      // Allow DDS discovery before the first query; no motion publishers exist.
      auto client = node->create_client<std_srvs::srv::Trigger>("/test_retry_authority");
      ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      if (scene == 5) {server.reset(); std::this_thread::sleep_for(std::chrono::milliseconds(100));}
      auto status = tree.tickRoot();
      if (scene == 4) {tree.haltTree(); EXPECT_EQ(replans, 0); continue;}
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
        if (scene != 3) {rclcpp::spin_some(node);}
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        status = tree.tickRoot();
      }
      const bool allowed = scene == 2;
      EXPECT_EQ(status, allowed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE) << scene;
      EXPECT_EQ(follows, allowed ? 2 : 1) << scene;
      EXPECT_EQ(replans, allowed ? 1 : 0) << scene;
    }
  }
  rclcpp::shutdown();
}
