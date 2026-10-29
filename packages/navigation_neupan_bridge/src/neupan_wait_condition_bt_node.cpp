// Copyright 2026 R1 navigation
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <future>
#include <stdexcept>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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

// A fresh request to the authority gates recovery. Topic delivery timing cannot
// turn an already latched runtime fault into a BT retry permission.
class CanRetryNeuPANExecution : public BT::StatefulActionNode
{
public:
  CanRetryNeuPANExecution(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    std::string service;
    getInput("service_name", service);
    client_ = node_->create_client<std_srvs::srv::Trigger>(
      service, rmw_qos_profile_services_default, callback_group_);
    getInput("timeout_sec", timeout_);
    if (!std::isfinite(timeout_) || timeout_ <= 0.) {throw std::invalid_argument("invalid_retry_query_timeout");}
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("service_name", "/neupan_controller/can_retry", "Read-only authority query"),
      BT::InputPort<double>("timeout_sec", .5, "Steady-clock response deadline")};
  }
  BT::NodeStatus onStart() override
  {
    clear();
    try {
      if (!client_->service_is_ready()) {return BT::NodeStatus::FAILURE;}
      started_ = std::chrono::steady_clock::now();
      pending_.emplace(client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()));
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception &) {
      clear(); return BT::NodeStatus::FAILURE;
    }
  }
  BT::NodeStatus onRunning() override
  {
    try {
      executor_.spin_some();
      if (!pending_ || std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count() > timeout_) {
        clear(); return BT::NodeStatus::FAILURE;
      }
      if (pending_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {return BT::NodeStatus::RUNNING;}
      const auto response = pending_->get(); pending_.reset();
      return response->success && response->message == "neupan_retry_permitted" ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const std::exception &) {
      clear(); return BT::NodeStatus::FAILURE;
    }
  }
  void onHalted() override {clear();}
  ~CanRetryNeuPANExecution() override {clear();}
private:
  void clear() {if (pending_) {client_->remove_pending_request(pending_->request_id); pending_.reset();}}
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> pending_;
  std::chrono::steady_clock::time_point started_;
  double timeout_{.5};
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
  factory.registerNodeType<navigation_neupan_bridge::CanRetryNeuPANExecution>("CanRetryNeuPANExecution");
}
