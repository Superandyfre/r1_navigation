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
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "r1_neupan_runtime/motion_authorization.hpp"
#include "r1_neupan_runtime/srv/update_path.hpp"
#include "r1_neupan_runtime/route_revision.hpp"
#include "navigation_neupan_bridge/execution_owner_lease.hpp"

namespace navigation_neupan_bridge
{

using Pose = geometry_msgs::msg::PoseStamped;
using Goals = std::vector<Pose>;

class NeuPANFollowPath : public nav2_behavior_tree::FollowPathAction
{
public:
  NeuPANFollowPath(const std::string & name, const BT::NodeConfiguration & config)
  : FollowPathAction(name, "follow_path", config)
  {
    owner_id_ = r1_neupan_runtime::newBootId();
    owner_pub_ = node_->create_publisher<std_msgs::msg::String>("/neupan_execution_owner", 20);
    rclcpp::SubscriptionOptions options; options.callback_group = callback_group_;
    owner_context_ = node_->create_subscription<std_msgs::msg::String>(
      "/neupan_execution_context", 20,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        try {context_ = nlohmann::json::parse(msg->data);} catch (const std::exception &) {}
      }, options);
    revoke_ = node_->create_client<std_srvs::srv::Trigger>(
      "/neupan_controller/revoke_uncertain_execution", rmw_qos_profile_services_default,
      callback_group_);
    update_path_ = node_->create_client<r1_neupan_runtime::srv::UpdatePath>(
      "/update_follow_path", rmw_qos_profile_services_default, callback_group_);
  }

  static BT::PortsList providedPorts()
  {
    auto ports = FollowPathAction::providedPorts();
    ports.insert(BT::InputPort<Pose>("navigation_goal"));
    ports.insert(BT::InputPort<Goals>("navigation_goals"));
    return ports;
  }

  void on_tick() override
  {
    clearUpdate(); update_failed_ = false;
    readParent();
    FollowPathAction::on_tick();
  }

  void on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback) override
  {
    Pose parent; Goals parents;
    const bool has_parent = bool(getInput("navigation_goal", parent));
    const bool has_parents = bool(getInput("navigation_goals", parents));
    std::string controller, checker;
    getInput("controller_id", controller); getInput("goal_checker_id", checker);
    nav_msgs::msg::Path path; getInput("path", path);
    // A parent request (including its stamp), terminal or plugin change still
    // creates a new action UUID. Never infer task identity from endpoint alone.
    const bool parent_changed = has_parent != has_parent_ || has_parents != has_parents_ ||
      (has_parent && parent != parent_) || (has_parents && parents != parents_);
    if ((!has_parent && !has_parents) || parent_changed || controller != goal_.controller_id ||
      checker != goal_.goal_checker_id ||
      !r1_neupan_runtime::validRouteRevision(path, goal_.path))
    {
      clearUpdate(); readParent();
      FollowPathAction::on_wait_for_result(feedback);
      // A new parent can have exactly the same path: it still needs a new UUID.
      if (parent_changed) {goal_updated_ = true;}
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (pending_update_) {
      if (pending_update_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          const auto response = pending_update_->get();
          if (response->accepted) {goal_.path = sent_path_;}
          else {update_failed_ = true;}
        } catch (const std::exception &) {update_failed_ = true;}
        pending_update_.reset();
      } else if (now - update_started_ > std::chrono::milliseconds(500)) {
        clearUpdate(); update_failed_ = true;
      }
      return;
    }
    if (r1_neupan_runtime::sameRouteGeometry(path, goal_.path) || update_failed_) {return;}
    if (!goal_handle_ || context_.value("state", "") != "ACTIVE" ||
      context_.value("navigation_attempt_id", "") != executionAttemptId(goal_handle_->get_goal_id()))
    {return;}
    if (!update_path_->service_is_ready()) {update_failed_ = true; return;}
    auto request = std::make_shared<r1_neupan_runtime::srv::UpdatePath::Request>();
    request->authority_boot_id = context_.at("authority_boot_id").get<std::string>();
    request->navigation_attempt_id = executionAttemptId(goal_handle_->get_goal_id());
    request->path = path; sent_path_ = path; update_started_ = now;
    pending_update_.emplace(update_path_->async_send_request(request));
  }

  BT::NodeStatus tick() override
  {
    const auto pending = future_goal_handle_;
    const auto status = FollowPathAction::tick();
    if (update_failed_) {
      revokeUncertain(); halt();
      return BT::NodeStatus::FAILURE;
    }
    if (status == BT::NodeStatus::RUNNING && goal_handle_ && !future_goal_handle_) {
      // Humble's human-readable to_string omits per-byte zero padding.
      // Use the exact fixed-width identity emitted by ControllerServer.
      const auto id = executionAttemptId(goal_handle_->get_goal_id());
      if (context_.value("state", "") == "ACTIVE" &&
        context_.value("navigation_attempt_id", "") == id)
      {
        nlohmann::json owner{{"schema", "r1_neupan_execution_owner_v1"},
          {"owner_id", owner_id_}, {"sequence", ++owner_sequence_},
          {"monotonic", std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count()}};
        for (const auto key : {"authority_boot_id", "controller_boot_id", "navigation_attempt_id"}) {
          owner[key] = context_.at(key);
        }
        std_msgs::msg::String msg; msg.data = owner.dump(); owner_pub_->publish(msg);
      }
    }
    if (status == BT::NodeStatus::FAILURE && pending &&
      pending->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      // Keep the unresolved handle visible to the base result callback while
      // revocation spins the executor; it must not dereference a null handle.
      future_goal_handle_ = pending;
      revokeUncertain();
    }
    return status;
  }

  void halt() override
  {
    clearUpdate();
    if (future_goal_handle_) {revokeUncertain();}
    FollowPathAction::halt();
  }

private:
  void readParent()
  {
    has_parent_ = bool(getInput("navigation_goal", parent_));
    has_parents_ = bool(getInput("navigation_goals", parents_));
  }
  void clearUpdate()
  {
    if (pending_update_) {
      update_path_->remove_pending_request(pending_update_->request_id);
      pending_update_.reset();
    }
  }
  using UpdatePath = r1_neupan_runtime::srv::UpdatePath;
  rclcpp::Client<UpdatePath>::SharedPtr update_path_;
  std::optional<rclcpp::Client<UpdatePath>::FutureAndRequestId> pending_update_;
  std::chrono::steady_clock::time_point update_started_;
  nav_msgs::msg::Path sent_path_;
  Pose parent_;
  Goals parents_;
  bool has_parent_{false}, has_parents_{false}, update_failed_{false};
  void revokeUncertain()
  {
    auto request = revoke_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto result = callback_group_executor_.spin_until_future_complete(request, server_timeout_);
    if (result != rclcpp::FutureReturnCode::SUCCESS || !request.get()->success) {
      RCLCPP_ERROR(node_->get_logger(), "Uncertain FollowPath revoke not confirmed; release blocked");
    }
  }
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr revoke_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr owner_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr owner_context_;
  nlohmann::json context_ = nlohmann::json::object();
  std::string owner_id_;
  std::int64_t owner_sequence_{0};
};

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
  factory.registerNodeType<navigation_neupan_bridge::NeuPANFollowPath>("NeuPANFollowPath");
  factory.registerNodeType<navigation_neupan_bridge::AcceptNeuPANPath>("AcceptNeuPANPath");
  factory.registerNodeType<navigation_neupan_bridge::IsNeuPANWaiting>("IsNeuPANWaiting");
  factory.registerNodeType<navigation_neupan_bridge::CanRetryNeuPANExecution>("CanRetryNeuPANExecution");
}
