// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation_neupan_bridge
{

using Pose = geometry_msgs::msg::PoseStamped;
using Goals = std::vector<Pose>;

bool sameGoal(const Pose & left, const Pose & right)
{
  const auto & a = left.pose.position;
  const auto & b = right.pose.position;
  const auto & q = left.pose.orientation;
  const auto & r = right.pose.orientation;
  const double dot = q.x * r.x + q.y * r.y + q.z * r.z + q.w * r.w;
  return left.header.frame_id == right.header.frame_id &&
         std::hypot(a.x - b.x, a.y - b.y) < 1e-4 &&
         std::abs(a.z - b.z) < 1e-4 && std::abs(std::abs(dot) - 1.0) < 1e-6;
}

bool sameGoals(const Goals & left, const Goals & right)
{
  if (left.size() != right.size()) {
    return false;
  }
  for (size_t index = 0; index < left.size(); ++index) {
    if (!sameGoal(left[index], right[index])) {
      return false;
    }
  }
  return true;
}

class AcceptNeuPANPath : public BT::SyncActionNode
{
public:
  AcceptNeuPANPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate_path"),
      BT::InputPort<Pose>("goal"),
      BT::InputPort<Goals>("goals"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<Pose>("accepted_goal"),
      BT::OutputPort<Goals>("accepted_goals"),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path candidate;
    if (!getInput("candidate_path", candidate) || candidate.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    if (auto goal = getInput<Pose>("goal")) {
      setOutput("accepted_goal", goal.value());
    } else if (auto goals = getInput<Goals>("goals")) {
      if (goals.value().empty()) {
        return BT::NodeStatus::FAILURE;
      }
      setOutput("accepted_goals", goals.value());
    } else {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("path", candidate);
    return BT::NodeStatus::SUCCESS;
  }
};

class IsNeuPANWaiting : public BT::ConditionNode
{
public:
  IsNeuPANWaiting(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string topic = "/neupan_semantics";
    getInput("semantic_topic", topic);
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    semantics_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr message) {
        bool waiting = false;
        try {
          const auto payload = nlohmann::json::parse(message->data);
          const auto motion = payload.at("obstacle_motion");
          waiting = motion.value("enabled", false) && motion.value("state", "") == "WAIT";
        } catch (const std::exception &) {
          waiting = false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (waiting && !waiting_) {
          waiting_since_ = now;
        }
        waiting_ = waiting;
        received_at_ = now;
      }, options);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Last accepted path"),
      BT::InputPort<Pose>("goal"),
      BT::InputPort<Goals>("goals"),
      BT::InputPort<Pose>("accepted_goal"),
      BT::InputPort<Goals>("accepted_goals"),
      BT::InputPort<std::string>("semantic_topic", "/neupan_semantics"),
    };
  }

  BT::NodeStatus tick() override
  {
    executor_.spin_some();
    nav_msgs::msg::Path path;
    if (!getInput("path", path) || path.poses.empty() || !waiting_) {
      return BT::NodeStatus::FAILURE;
    }
    if (auto goal = getInput<Pose>("goal")) {
      Pose accepted;
      if (!getInput("accepted_goal", accepted) || !sameGoal(goal.value(), accepted)) {
        return BT::NodeStatus::FAILURE;
      }
    } else if (auto goals = getInput<Goals>("goals")) {
      Goals accepted;
      if (!getInput("accepted_goals", accepted) || !sameGoals(goals.value(), accepted)) {
        return BT::NodeStatus::FAILURE;
      }
    } else {
      return BT::NodeStatus::FAILURE;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - received_at_ > std::chrono::milliseconds(500) ||
      now - waiting_since_ > std::chrono::seconds(30))
    {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr semantics_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool waiting_{false};
  std::chrono::steady_clock::time_point waiting_since_{};
  std::chrono::steady_clock::time_point received_at_{};
};

}  // namespace navigation_neupan_bridge

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigation_neupan_bridge::AcceptNeuPANPath>("AcceptNeuPANPath");
  factory.registerNodeType<navigation_neupan_bridge::IsNeuPANWaiting>("IsNeuPANWaiting");
}
