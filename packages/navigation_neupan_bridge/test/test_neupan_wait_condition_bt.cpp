// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <memory>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
