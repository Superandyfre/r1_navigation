#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "navigation_neupan_bridge/neupan_controller.hpp"
#include "navigation_neupan_bridge/path_replan_distance.hpp"
#include "navigation_neupan_bridge/guard_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <utility>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <nlohmann/json.hpp>
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigation_neupan_bridge
{
namespace
{
constexpr char kEnvelopeSchema[] = "r1_n2_3_command_envelope_v1";
// NeuPAN actions are serialized as float32 before entering the JSON envelope.
// Accept only representation noise at the actuator boundary, then return the
// exact configured bound to Nav2.  This does not relax the gate's hard limit.
constexpr double kActuatorLimitRepresentationEpsilon = 1e-6;

template<typename T>
void declareIfMissing(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, const T & value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
  }
}

bool sameGoal(
  const geometry_msgs::msg::PoseStamped & left,
  const geometry_msgs::msg::PoseStamped & right)
{
  constexpr double kPositionEpsilon = 1e-4;
  constexpr double kOrientationEpsilon = 1e-6;
  if (std::abs(left.pose.position.x - right.pose.position.x) > kPositionEpsilon ||
      std::abs(left.pose.position.y - right.pose.position.y) > kPositionEpsilon ||
      std::abs(left.pose.position.z - right.pose.position.z) > kPositionEpsilon)
  {
    return false;
  }
  const auto & l = left.pose.orientation;
  const auto & r = right.pose.orientation;
  const double dot = l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w;
  return std::abs(std::abs(dot) - 1.0) <= kOrientationEpsilon;
}

}  // namespace

void NeuPANController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("NeuPANController parent expired during configure");
  }
  plugin_name_ = std::move(name);
  (void)tf;
  costmap_ros_ = std::move(costmap_ros);

  declareIfMissing(node_, plugin_name_ + ".navigation_frame", navigation_frame_);
  declareIfMissing(node_, plugin_name_ + ".path_topic", path_topic_);
  declareIfMissing(node_, plugin_name_ + ".goal_topic", goal_topic_);
  declareIfMissing(node_, plugin_name_ + ".envelope_topic", envelope_topic_);
  declareIfMissing(node_, plugin_name_ + ".gate_command_topic", gate_command_topic_);
  declareIfMissing(node_, plugin_name_ + ".gate_status_topic", gate_status_topic_);
  declareIfMissing(node_, plugin_name_ + ".guard_status_topic", guard_status_topic_);
  declareIfMissing(node_, plugin_name_ + ".localization_health_topic", localization_health_topic_);
  declareIfMissing(node_, plugin_name_ + ".diagnostics_topic", diagnostics_topic_);
  declareIfMissing(node_, plugin_name_ + ".envelope_timeout", envelope_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".envelope_startup_timeout", envelope_startup_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".envelope_recovery_timeout", envelope_recovery_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".max_linear", max_linear_);
  declareIfMissing(node_, plugin_name_ + ".max_reverse", max_reverse_);
  declareIfMissing(node_, plugin_name_ + ".max_angular", max_angular_);
  declareIfMissing(node_, plugin_name_ + ".allow_reverse", allow_reverse_);
  declareIfMissing(node_, plugin_name_ + ".shadow_mode", shadow_mode_);
  declareIfMissing(node_, plugin_name_ + ".require_gate_match", require_gate_match_);
  declareIfMissing(node_, plugin_name_ + ".command_match_epsilon", command_match_epsilon_);
  declareIfMissing(node_, plugin_name_ + ".guard_status_timeout", guard_status_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".guard_startup_timeout", guard_startup_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".guard_stop_timeout", guard_stop_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".localization_health_timeout", localization_health_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".localization_startup_timeout", localization_startup_timeout_sec_);
  declareIfMissing(node_, plugin_name_ + ".require_localization_ready", require_localization_ready_);

  node_->get_parameter(plugin_name_ + ".navigation_frame", navigation_frame_);
  node_->get_parameter(plugin_name_ + ".path_topic", path_topic_);
  node_->get_parameter(plugin_name_ + ".goal_topic", goal_topic_);
  node_->get_parameter(plugin_name_ + ".envelope_topic", envelope_topic_);
  node_->get_parameter(plugin_name_ + ".gate_command_topic", gate_command_topic_);
  node_->get_parameter(plugin_name_ + ".gate_status_topic", gate_status_topic_);
  node_->get_parameter(plugin_name_ + ".guard_status_topic", guard_status_topic_);
  node_->get_parameter(plugin_name_ + ".localization_health_topic", localization_health_topic_);
  node_->get_parameter(plugin_name_ + ".diagnostics_topic", diagnostics_topic_);
  node_->get_parameter(plugin_name_ + ".envelope_timeout", envelope_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".envelope_startup_timeout", envelope_startup_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".envelope_recovery_timeout", envelope_recovery_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".max_linear", max_linear_);
  node_->get_parameter(plugin_name_ + ".max_reverse", max_reverse_);
  node_->get_parameter(plugin_name_ + ".max_angular", max_angular_);
  node_->get_parameter(plugin_name_ + ".allow_reverse", allow_reverse_);
  node_->get_parameter(plugin_name_ + ".shadow_mode", shadow_mode_);
  node_->get_parameter(plugin_name_ + ".require_gate_match", require_gate_match_);
  node_->get_parameter(plugin_name_ + ".command_match_epsilon", command_match_epsilon_);
  node_->get_parameter(plugin_name_ + ".guard_status_timeout", guard_status_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".guard_startup_timeout", guard_startup_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".guard_stop_timeout", guard_stop_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".localization_health_timeout", localization_health_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".localization_startup_timeout", localization_startup_timeout_sec_);
  node_->get_parameter(plugin_name_ + ".require_localization_ready", require_localization_ready_);

  for (auto pair : {std::make_pair("worker_health_topic", &worker_health_topic_),
    std::make_pair("execution_context_topic", &execution_context_topic_),
    std::make_pair("motion_grant_topic", &motion_grant_topic_),
    std::make_pair("motion_arm_service", &motion_arm_service_),
    std::make_pair("motion_retry_service", &motion_retry_service_),
    std::make_pair("execution_owner_topic", &owner_topic_),
    std::make_pair("motion_revoke_service", &motion_revoke_service_),
    std::make_pair("arm_odom_frame", &arm_odom_frame_),
    std::make_pair("arm_base_frame", &arm_base_frame_),
    std::make_pair("odom_topic", &odom_topic_)})
  {
    declareIfMissing(node_, plugin_name_ + "." + pair.first, *pair.second);
    node_->get_parameter(plugin_name_ + "." + pair.first, *pair.second);
  }
  r1_neupan_runtime::AuthorizationConfig runtime_config;
  runtime_config.source_timeout = std::min(envelope_timeout_sec_, 0.5);
  runtime_config.recovery_timeout = envelope_recovery_timeout_sec_;
  for (auto pair : {std::make_pair("worker_health_timeout", &runtime_config.health_timeout),
    std::make_pair("planner_input_timeout", &runtime_config.input_timeout),
    std::make_pair("navigation_authority_timeout", &runtime_config.authority_timeout)})
  {
    declareIfMissing(node_, plugin_name_ + "." + pair.first, *pair.second);
    node_->get_parameter(plugin_name_ + "." + pair.first, *pair.second);
  }
  // Freeze the plugin namespace. These values are read at configure time;
  // accepting a live mutation would make the parameter server disagree with
  // the retained authorization monitor (or change shadow/production mode).
  std::map<std::string, rclcpp::ParameterValue> configuration;
  for (const auto & parameter : node_->get_parameters(
      node_->list_parameters({plugin_name_}, 0).names))
  {configuration.emplace(parameter.get_name(), parameter.get_parameter_value());}
  if (!configured_parameters_.empty() && configuration != configured_parameters_) {
    if (execution_) {execution_->fail("immutable_configuration_changed");}
    throw nav2_core::PlannerException("immutable_configuration_changed_require_process_restart");
  }
  configured_parameters_ = configuration;
  parameter_guard_ = node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      std::lock_guard<std::mutex> lock(mutex_);
      rcl_interfaces::msg::SetParametersResult result; result.successful = true;
      const std::string prefix = plugin_name_ + ".";
      for (const auto & parameter : parameters) {
        if (parameter.get_name().compare(0, prefix.size(), prefix) != 0) {continue;}
        const auto original = configured_parameters_.find(parameter.get_name());
        if (original == configured_parameters_.end() ||
          original->second != parameter.get_parameter_value())
        {
          result.successful = false;
          result.reason = "NeuPAN plugin configuration is immutable; controlled process restart required";
          break;
        }
      }
      return result;
    });
  if (!execution_) {
    execution_ = std::make_unique<r1_neupan_runtime::ControllerExecution>(
      r1_neupan_runtime::newBootId(), runtime_config, envelope_startup_timeout_sec_);
  }
  execution_context_pub_ = node_->create_publisher<std_msgs::msg::String>(execution_context_topic_, 20);
  motion_grant_pub_ = node_->create_publisher<std_msgs::msg::String>(motion_grant_topic_, 20);
  worker_health_sub_ = node_->create_subscription<std_msgs::msg::String>(worker_health_topic_, 20,
    [this](std_msgs::msg::String::SharedPtr msg) {
      if (shadow_mode_) {return;}
      std::lock_guard<std::mutex> lock(mutex_);
      try {execution_->health(nlohmann::json::parse(msg->data), monotonicNow());}
      catch (const std::exception &) {if (execution_->active()) {execution_->fail("invalid_worker_health");}}
      if (execution_->faulted()) {envelope_ = Envelope{};}
    });
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {
        stopped_odometry_.reset(); return;
      }
      const auto & v = msg->twist.twist;
      const std::int64_t stamp = static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
        msg->header.stamp.nanosec;
      stopped_odometry_.observe(stamp,
        node_->now().nanoseconds(), monotonicNow(), msg->header.frame_id, msg->child_frame_id,
        arm_odom_frame_, arm_base_frame_,
        {v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z});
    });
  arm_service_ = node_->create_service<std_srvs::srv::Trigger>(motion_arm_service_,
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = monotonicNow();
      const bool stopped = stopped_odometry_.stopped(now, node_->now().nanoseconds());
      response->success = active_ && !shadow_mode_ && execution_->arm(now, stopped);
      response->message = response->success ? "run_armed_each_action_still_requires_grant" :
        "arm_denied_require_idle_stopped_fresh_guard_and_no_fatal";
    });
  owner_sub_ = node_->create_subscription<std_msgs::msg::String>(owner_topic_, rclcpp::QoS(20),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!execution_->active() || execution_->faulted()) {return;}
      try {owner_lease_.observe(nlohmann::json::parse(msg->data), monotonicNow());}
      catch (const std::exception &) {execution_->fail("invalid_execution_owner");}
      ownerReadyLocked(monotonicNow());
    });
  retry_service_ = node_->create_service<std_srvs::srv::Trigger>(motion_retry_service_,
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::mutex> lock(mutex_);
      response->success = active_ && !shadow_mode_ && execution_->retryAllowed(monotonicNow());
      response->message = response->success ? "neupan_retry_permitted" : "neupan_retry_denied";
    });
  // Deny-only, idempotent run-wide latch: a late/duplicate request may stop a
  // newer execution, but can never authorize one or clear the original fault.
  revoke_service_ = node_->create_service<std_srvs::srv::Trigger>(motion_revoke_service_,
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::mutex> lock(mutex_);
      execution_->fail("follow_path_execution_uncertain");
      endExecutionLocked("follow_path_execution_uncertain");
      response->success = true;
      response->message = "run_revoked_restart_required";
    });
  runtime_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
    std::bind(&NeuPANController::runtimeTick, this));

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1));
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(1));
  diagnostics_pub_ = node_->create_publisher<std_msgs::msg::String>(diagnostics_topic_, rclcpp::QoS(20));
  envelope_sub_ = node_->create_subscription<std_msgs::msg::String>(
    envelope_topic_, rclcpp::QoS(20),
    std::bind(&NeuPANController::envelopeCallback, this, std::placeholders::_1));
  gate_command_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    gate_command_topic_, rclcpp::QoS(20),
    std::bind(&NeuPANController::gateCommandCallback, this, std::placeholders::_1));
  gate_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    gate_status_topic_, rclcpp::QoS(20),
    std::bind(&NeuPANController::gateStatusCallback, this, std::placeholders::_1));
  guard_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    guard_status_topic_, rclcpp::QoS(20),
    std::bind(&NeuPANController::guardStatusCallback, this, std::placeholders::_1));
  localization_health_sub_ = node_->create_subscription<std_msgs::msg::String>(
    localization_health_topic_, rclcpp::QoS(20),
    std::bind(&NeuPANController::localizationHealthCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "NeuPAN controller configured in %s mode", shadow_mode_ ? "shadow" : "active");
}

void NeuPANController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  endExecutionLocked("cleanup");
  if (execution_) {execution_->disarm();}
  stopped_odometry_.reset();
  active_ = false;
  plan_valid_ = false;
  path_published_ = false;
  route_update_gate_.reset();
  localization_ready_observed_ = false;
  envelope_recovery_active_ = false;
  envelope_ = Envelope{};
  runtime_timer_.reset();
  arm_service_.reset();
  owner_sub_.reset();
  retry_service_.reset();
  revoke_service_.reset();
  worker_health_sub_.reset();
  odom_sub_.reset();
  execution_context_pub_.reset();
  motion_grant_pub_.reset();
  path_pub_.reset();
  goal_pub_.reset();
  diagnostics_pub_.reset();
  envelope_sub_.reset();
  gate_command_sub_.reset();
  gate_status_sub_.reset();
  guard_status_sub_.reset();
  localization_health_sub_.reset();
}

void NeuPANController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
}

void NeuPANController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  endExecutionLocked("deactivated");
  if (execution_) {execution_->disarm();}
  stopped_odometry_.reset();
  active_ = false;
}

void NeuPANController::beginExecution(const std::string & authority, const std::string & attempt)
{
  if (shadow_mode_) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {throw nav2_core::PlannerException("controller_inactive");}
  execution_->begin(authority, attempt, monotonicNow());
  owner_lease_.begin(authority, execution_->boot(), attempt, monotonicNow());
  path_published_ = false; goal_published_ = false; plan_valid_ = false;
  route_update_gate_.reset(); envelope_ = Envelope{};
  envelope_recovery_active_ = false; guard_stop_active_ = false;
}

void NeuPANController::checkExecution(const std::string & authority, const std::string & attempt)
{
  if (shadow_mode_) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  ownerReadyLocked(monotonicNow());
  execution_->check(authority, attempt, monotonicNow());
}

bool NeuPANController::ownerReadyLocked(double now)
{
  const bool ready = owner_lease_.ready(now);
  if (!owner_lease_.fault().empty()) {execution_->fail(owner_lease_.fault());}
  return ready;
}

nlohmann::json NeuPANController::executionContextLocked(const char * state)
{
  nlohmann::json payload{{"schema", "r1_neupan_execution_context_v1"}, {"state", state},
    {"authority_boot_id", execution_->authority()}, {"controller_boot_id", execution_->boot()},
    {"navigation_attempt_id", execution_->attempt()}, {"path_version", path_version_},
    {"goal_version", std::max(0, goal_version_)}, {"sequence", ++context_sequence_},
    {"monotonic", monotonicNow()}, {"frame_id", navigation_frame_}};
  auto coordinates = nlohmann::json::array();
  for (const auto & pose : published_path_.poses) {
    coordinates.push_back({pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)});
  }
  payload["path"] = coordinates;
  if (!coordinates.empty()) {payload["goal"] = coordinates.back();}
  return payload;
}

void NeuPANController::publishExecutionContextLocked(const char * state)
{
  if (!execution_context_pub_ || !execution_ || !execution_->active()) {return;}
  std_msgs::msg::String message; message.data = executionContextLocked(state).dump();
  execution_context_pub_->publish(message);
}

void NeuPANController::endExecutionLocked(const std::string & reason)
{
  if (shadow_mode_ || !execution_ || !execution_->active()) {return;}
  auto end = executionContextLocked("END");
  const auto revoke = execution_->end(monotonicNow());
  envelope_ = Envelope{}; plan_valid_ = false;  // invalidate locally before any IO
  if (execution_context_pub_) {
    std_msgs::msg::String msg; msg.data = end.dump(); execution_context_pub_->publish(msg);
  }
  if (revoke && motion_grant_pub_) {
    std_msgs::msg::String msg; msg.data = revoke->dump(); motion_grant_pub_->publish(msg);
  }
  publishDiagnostic("info", "execution_ended:" + reason);
}

void NeuPANController::endExecution(const std::string & authority, const std::string & attempt,
  const std::string & reason) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (execution_ && execution_->authority() == authority && execution_->attempt() == attempt) {
      endExecutionLocked(reason);
    }
  } catch (...) {
    // Any publisher failure stops future grants; the final Guard's independent
    // authority lease cannot be extended by an exception during teardown.
    try {std::lock_guard<std::mutex> lock(mutex_); execution_->fail("execution_end_failed");}
    catch (...) {}
  }
}

void NeuPANController::runtimeTick()
{
  if (shadow_mode_) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!execution_) {return;}
  // Periodic, read-only state remains visible after an Action has ended. It
  // never grants motion; consumers must still pass the normal grant protocol.
  if (diagnostics_pub_) {
    std_msgs::msg::String status;
    status.data = nlohmann::json{{"schema", "r1_neupan_execution_status_v1"},
      {"controller_boot_id", execution_->boot()}, {"sequence", ++runtime_status_sequence_},
      {"authority_boot_id", execution_->authority()},
      {"monotonic", monotonicNow()}, {"plugin_active", active_},
      {"execution_active", execution_->active()}, {"fault_latched", execution_->faulted()},
      {"fault_reason", execution_->fault()}, {"navigation_attempt_id", execution_->attempt()}}.dump();
    try {diagnostics_pub_->publish(status);}
    catch (const std::exception &) {execution_->fail("execution_status_publish_failed");}
  }
  if (!active_ || !execution_->active()) {return;}
  try {
    // Ownership needs the exact execution identity before it can be confirmed.
    // Repeat this non-authorizing context even while waiting for the owner;
    // a missed first publication must not deadlock the ownership handshake.
    if (path_published_) {publishExecutionContextLocked("ACTIVE");}
    if (!ownerReadyLocked(monotonicNow())) {
      if (execution_->faulted()) {endExecutionLocked(execution_->fault());}
      return;
    }
    const auto grant = execution_->tick(monotonicNow());
    if (execution_->faulted()) {envelope_ = Envelope{}; endExecutionLocked(execution_->fault()); return;}
    if (grant) {std_msgs::msg::String msg; msg.data = grant->dump(); motion_grant_pub_->publish(msg);}
  } catch (const std::exception &) {execution_->fail("execution_publish_failed"); envelope_ = Envelope{};}
}

void NeuPANController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    rejectPlan("empty_or_frameless_path");
    return;
  }

  if (path.header.frame_id != navigation_frame_) {
    rejectPlan("unexpected_path_frame:" + path.header.frame_id);
    return;
  }
  nav_msgs::msg::Path navigation_path = path;
  for (auto & pose : navigation_path.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != navigation_frame_) {
      rejectPlan("mixed_path_frames");
      return;
    }
    if (!finite(pose.pose.position.x) || !finite(pose.pose.position.y) ||
      !finite(pose.pose.position.z) || !finite(pose.pose.orientation.x) ||
      !finite(pose.pose.orientation.y) || !finite(pose.pose.orientation.z) ||
      !finite(pose.pose.orientation.w)) {rejectPlan("nonfinite_path"); return;}
    const auto & q = pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!finite(norm) || std::abs(norm - 1.) > 1e-3 || !finite(tf2::getYaw(q))) {
      rejectPlan("invalid_path_quaternion"); return;
    }
    pose.header.frame_id = navigation_frame_;
  }

  geometry_msgs::msg::PoseStamped goal = navigation_path.poses.back();
  bool publish_path = false;
  bool publish_goal = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!shadow_mode_ && (!execution_->active() || execution_->faulted())) {
      throw nav2_core::PlannerException("setPlan_without_current_execution");
    }
    const auto now = std::chrono::steady_clock::now();
    // Moderate route shifts need repeated evidence before resetting the
    // asynchronous solver; a material new bend needs only one sample.
    const bool route_departed = path_published_ && sameGoal(goal, published_goal_) &&
      route_update_gate_.consider(navigation_path, published_path_, now);
    publish_path = !path_published_ || !sameGoal(goal, published_goal_) || route_departed;
    if (publish_path) {
      ++path_version_;
      published_path_ = navigation_path;
      path_published_ = true;
      route_update_gate_.published(now);
      // The Python bridge advances both context versions on every path.
      ++goal_version_;
      published_goal_ = goal;
      goal_published_ = true;
      publish_goal = true;
      envelope_ = Envelope{};
      if (shadow_mode_) {worker_generation_ = -1;}
      localization_ready_observed_ = false;
      plan_published_at_ = std::chrono::steady_clock::now();
      envelope_recovery_active_ = false;
      if (!shadow_mode_) {
        execution_->plan(path_version_, goal_version_, monotonicNow());
        // Send route PREPARE synchronously, before publishing new Worker context.
        // The Guard ACK, not local publication order, establishes invalidation.
        const auto grant = ownerReadyLocked(monotonicNow()) ?
          execution_->tick(monotonicNow()) : std::nullopt;
        if (grant) {std_msgs::msg::String msg; msg.data = grant->dump(); motion_grant_pub_->publish(msg);}
      }
    }
    plan_valid_ = true;
    if (publish_path && !shadow_mode_) {publishExecutionContextLocked("ACTIVE");}
  }
  if (publish_path) {
    path_pub_->publish(navigation_path);
  }
  if (publish_goal) {
    goal_pub_->publish(goal);
  }
  if (publish_path) {
    publishDiagnostic("info", "path_published");
  }
}

geometry_msgs::msg::TwistStamped NeuPANController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  Envelope envelope;
  geometry_msgs::msg::Twist gate_command;
  GateStatus gate_status;
  GuardStatus guard_status;
  LocalizationHealth localization_health;
  bool localization_ready_observed = false;
  bool gate_command_received = false;
  int expected_path = 0;
  int expected_goal = 0;
  std::chrono::steady_clock::time_point plan_published_at;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      throw nav2_core::PlannerException("NeuPAN controller inactive");
    }
    if (!shadow_mode_) {
      if (execution_->faulted()) {throw nav2_core::PlannerException(execution_->fault());}
      if (!execution_->active()) {throw nav2_core::PlannerException("execution_not_current");}
    }
    if (!plan_valid_) {
      throw nav2_core::PlannerException("NeuPAN path unavailable");
    }
    envelope = envelope_;
    gate_command = gate_command_;
    gate_status = gate_status_;
    guard_status = guard_status_;
    localization_health = localization_health_;
    localization_ready_observed = localization_ready_observed_;
    gate_command_received = gate_command_received_;
    expected_path = path_version_;
    expected_goal = goal_version_;
    plan_published_at = plan_published_at_;
  }
  if (require_localization_ready_ && !shadow_mode_) {
    const auto health_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - localization_health.received_at).count();
    const auto plan_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - plan_published_at).count();
    const bool waiting_for_initial_health =
      !localization_ready_observed &&
      (!localization_health.valid || localization_health.state == "BOOTSTRAP");
    if (waiting_for_initial_health && plan_age <= localization_startup_timeout_sec_) {
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      command.header.stamp = node_->now();
      publishDiagnostic("info", "awaiting_localization_ready");
      return command;
    }
    if (!localization_health.valid || health_age > localization_health_timeout_sec_) {
      throw nav2_core::PlannerException("R1 localization health status unavailable");
    }
    if (localization_health.state == "READY") {
      std::lock_guard<std::mutex> lock(mutex_);
      localization_ready_observed_ = true;
    }
    if (localization_health.state != "READY") {
      throw nav2_core::PlannerException(
              "R1 localization unavailable:" + localization_health.reason);
    }
  }
  if (!shadow_mode_) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool permitted = ownerReadyLocked(monotonicNow()) && execution_->permitted(monotonicNow());
    if (execution_->faulted()) {throw nav2_core::PlannerException(execution_->fault());}
    if (!permitted) {
      geometry_msgs::msg::TwistStamped zero; zero.header = pose.header; zero.header.stamp = node_->now();
      return zero;
    }
  }
  if (!envelope.valid) {
    const auto startup_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - plan_published_at).count();
    if (startup_age <= envelope_startup_timeout_sec_) {
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      command.header.stamp = node_->now();
      publishDiagnostic("info", "awaiting_initial_envelope");
      return command;
    }
    // A failed FollowPath action may later be retried with identical geometry.
    // Re-arm its path/goal context so that retry cannot inherit this expired
    // startup deadline or an obsolete worker version.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan_valid_ = false;
      path_published_ = false;
      route_update_gate_.reset();
      goal_published_ = false;
      envelope_ = Envelope{};
      worker_generation_ = -1;
    }
    publishDiagnostic(
      "error", "envelope_unavailable:path_version=" + std::to_string(expected_path) +
      ":goal_version=" + std::to_string(expected_goal) +
      ":wait_sec=" + std::to_string(startup_age));
    throw nav2_core::PlannerException("NeuPAN envelope unavailable");
  }
  const auto receive_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - envelope.received_at).count();
  const auto planner_age = monotonicNow() - envelope.planner_result_monotonic;
  if (receive_age > envelope_timeout_sec_ || planner_age < 0.0 ||
      planner_age > envelope_timeout_sec_) {
    const auto now = std::chrono::steady_clock::now();
    double recovery_age = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!envelope_recovery_active_) {
        envelope_recovery_active_ = true;
        envelope_recovery_started_at_ = now;
      }
      recovery_age = std::chrono::duration<double>(
        now - envelope_recovery_started_at_).count();
    }
    // Never replay an old planner action. During a bounded recovery window
    // return zero so Nav2 keeps the FollowPath action alive for a fresh scan
    // solve. The collision guard receives a fresh zero command as well.
    if (recovery_age <= envelope_recovery_timeout_sec_) {
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      command.header.stamp = node_->now();
      publishDiagnostic(
        "info", "awaiting_fresh_envelope:receive_age_sec=" +
        std::to_string(receive_age) + ":recovery_age_sec=" +
        std::to_string(recovery_age));
      return command;
    }
    publishDiagnostic(
      "error", "envelope_stale:path_version=" + std::to_string(expected_path) +
      ":goal_version=" + std::to_string(expected_goal) +
      ":receive_age_sec=" + std::to_string(receive_age) +
      ":planner_age_sec=" + std::to_string(planner_age));
    throw nav2_core::PlannerException("NeuPAN envelope stale");
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    envelope_recovery_active_ = false;
  }
  if (envelope.path_version != expected_path || envelope.goal_version != expected_goal) {
    publishDiagnostic(
      "error", "envelope_context_mismatch:expected_path=" + std::to_string(expected_path) +
      ":expected_goal=" + std::to_string(expected_goal) +
      ":received_path=" + std::to_string(envelope.path_version) +
      ":received_goal=" + std::to_string(envelope.goal_version));
    throw nav2_core::PlannerException("NeuPAN envelope context mismatch");
  }
  if (!finite(envelope.linear_x) || !finite(envelope.angular_z)) {
    throw nav2_core::PlannerException("NeuPAN envelope nonfinite command");
  }
  const double minimum_linear = allow_reverse_ ? -max_reverse_ : 0.0;
  if (envelope.linear_x < minimum_linear - kActuatorLimitRepresentationEpsilon ||
      envelope.linear_x > max_linear_ + kActuatorLimitRepresentationEpsilon ||
      std::abs(envelope.angular_z) > max_angular_ + kActuatorLimitRepresentationEpsilon) {
    throw nav2_core::PlannerException("NeuPAN envelope violates actuator limits");
  }
  const double bounded_linear = std::clamp(envelope.linear_x, minimum_linear, max_linear_);
  const double bounded_angular = std::clamp(
    envelope.angular_z, -max_angular_, max_angular_);
  if (require_gate_match_ && !shadow_mode_) {
    const auto gate_status_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - gate_status.received_at).count();
    // The gate intentionally rate-limits its final Twist, so compare the
    // envelope identity with the gate's recorded input. Its output is then
    // checked against the immutable actuator bounds.
    if (!gate_status.valid ||
        (gate_status.emergency_stop && !gate_status.failure_latched)) {
      // A deliberate operator e-stop is the pre-arm state for HW1. Keep the
      // Nav2 action alive with a zero observed command until explicit release.
      // The first relay cycle has no source metadata yet, which is also safe
      // only while this operator lock remains asserted.
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      command.header.stamp = node_->now();
      publishDiagnostic("info", "awaiting_gate_release");
      return command;
    }
    if (!gate_command_received || !gate_status.valid ||
        gate_status_age > envelope_timeout_sec_ || gate_status.failure_latched ||
        std::abs(gate_status.input_linear_x - envelope.linear_x) > command_match_epsilon_ ||
        std::abs(gate_status.input_angular_z - envelope.angular_z) > command_match_epsilon_ ||
        std::abs(gate_status.planner_result_monotonic - envelope.planner_result_monotonic) > command_match_epsilon_ ||
        gate_status.path_version != envelope.path_version ||
        gate_status.goal_version != envelope.goal_version ||
        !finite(gate_status.output_linear_x) || !finite(gate_status.output_angular_z) ||
        gate_status.output_linear_x < minimum_linear - kActuatorLimitRepresentationEpsilon ||
        gate_status.output_linear_x > max_linear_ + kActuatorLimitRepresentationEpsilon ||
        std::abs(gate_status.output_angular_z) > max_angular_ + kActuatorLimitRepresentationEpsilon) {
      throw nav2_core::PlannerException("NeuPAN gate envelope mismatch");
    }
  }

  if (!shadow_mode_) {
    const auto guard_now = std::chrono::steady_clock::now();
    const auto plan_age = std::chrono::duration<double>(guard_now - plan_published_at).count();
    const auto guard_age = std::chrono::duration<double>(guard_now - guard_status.received_at).count();
    if (!guard_status.valid) {
      if (plan_age > guard_startup_timeout_sec_) {
        throw nav2_core::PlannerException("R1 collision guard status unavailable");
      }
    } else if (guard_age > guard_status_timeout_sec_) {
      throw nav2_core::PlannerException("R1 collision guard status stale");
    } else if (guard_status.state == "FAULT") {
      // Before the first controller command the guard correctly reports a
      // command timeout. In smoothed mode it also independently watches the
      // raw source. These two faults are caused by this controller not yet
      // publishing, so treating them as an upstream fault creates a circular
      // stop: the guard waits for a command while the controller waits for a
      // healthy guard. The envelope watchdog remains authoritative for stale
      // planner output and the guard still fails closed on every other fault.
      const bool awaiting_first_command =
        guard_status.reason == "command_stale" ||
        guard_status.reason == "source_command_stale";
      // A brief sensor timestamp gap holds a fresh zero while the guard
      // waits for valid data. The odometry watchdog stays at 150 ms; only
      // task termination is delayed, by at most two seconds.
      const auto sensor_recovery_timeout =
        guardSensorRecoveryTimeout(guard_status.reason, guard_stop_timeout_sec_);
      if (sensor_recovery_timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!guard_stop_active_) {
          guard_stop_active_ = true;
          guard_stop_started_at_ = guard_now;
        } else if (std::chrono::duration<double>(guard_now - guard_stop_started_at_).count() >=
          *sensor_recovery_timeout)
        {
          throw nav2_core::PlannerException("R1 collision guard sensor recovery timeout");
        }
        geometry_msgs::msg::TwistStamped command;
        command.header = pose.header;
        command.header.stamp = node_->now();
        publishDiagnostic("info", "waiting_for_guard_sensor_recovery:" + guard_status.reason);
        return command;
      }
      if (!awaiting_first_command) {
        throw nav2_core::PlannerException("R1 collision guard fault:" + guard_status.reason);
      }
    } else if (guard_status.state == "STOP") {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!guard_stop_active_) {
        guard_stop_active_ = true;
        guard_stop_started_at_ = guard_now;
      } else if (std::chrono::duration<double>(guard_now - guard_stop_started_at_).count() >=
        guard_stop_timeout_sec_)
      {
        throw nav2_core::PlannerException("R1 collision guard blocking timeout");
      }
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      guard_stop_active_ = false;
    }
  }

  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  command.header.stamp = node_->now();
  if (!shadow_mode_) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (execution_->faulted()) {throw nav2_core::PlannerException(execution_->fault());}
    if (!ownerReadyLocked(monotonicNow()) || !execution_->permitted(monotonicNow()) || !plan_valid_ ||
      path_version_ != expected_path || goal_version_ != expected_goal ||
      !envelope_.valid || envelope_.planner_result_monotonic != envelope.planner_result_monotonic)
    {return command;}
    command.twist.linear.x = bounded_linear;
    command.twist.angular.z = bounded_angular;
  }
  publishDiagnostic("info", shadow_mode_ ? "shadow_envelope_valid" : "envelope_valid");
  return command;
}

void NeuPANController::setSpeedLimit(const double &, const bool &)
{
  // NeuPAN's frozen safety gate owns the final speed and acceleration limits.
}

void NeuPANController::envelopeCallback(const std_msgs::msg::String::SharedPtr message)
{
  if (!shadow_mode_) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      const auto payload = nlohmann::json::parse(message->data);
      const double now = monotonicNow();
      if (execution_->source(payload, now)) {
        const auto source = r1_neupan_runtime::parseSource(payload, now);
        if (!envelope_.valid || source.published != envelope_.planner_result_monotonic) {
          envelope_.linear_x = source.linear; envelope_.angular_z = source.angular;
          envelope_.planner_result_monotonic = source.published;
          envelope_.worker_generation = source.context.generation;
          envelope_.path_version = source.context.path; envelope_.goal_version = source.context.goal;
          envelope_.valid = true; envelope_.received_at = std::chrono::steady_clock::now();
        }
      }
      if (execution_->faulted()) {envelope_ = Envelope{};}
    } catch (const std::exception &) {
      if (execution_->active()) {execution_->fail("invalid_planner_source"); envelope_ = Envelope{};}
    }
    return;
  }
  Envelope candidate;
  try {
    const auto payload = nlohmann::json::parse(message->data);
    if (payload.value("schema", "") != kEnvelopeSchema || !payload.value("finite", false)) {
      throw std::runtime_error("schema_or_finite_invalid");
    }
    const auto & command = payload.at("command");
    candidate.linear_x = command.at("linear_x").get<double>();
    candidate.angular_z = command.at("angular_z").get<double>();
    candidate.planner_result_monotonic = payload.at("planner_result_monotonic").get<double>();
    candidate.worker_generation = payload.at("worker_generation").get<std::int64_t>();
    candidate.path_version = payload.at("path_version").get<int>();
    candidate.goal_version = payload.at("goal_version").get<int>();
    if (!finite(candidate.linear_x) || !finite(candidate.angular_z) ||
        !finite(candidate.planner_result_monotonic) || candidate.planner_result_monotonic <= 0.0) {
      throw std::runtime_error("nonfinite_or_invalid_timestamp");
    }
    candidate.valid = true;
    candidate.received_at = std::chrono::steady_clock::now();
  } catch (const std::exception & error) {
    publishDiagnostic("error", std::string("invalid_envelope:") + error.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // A path update deliberately invalidates the previous context.  Late worker
  // results for that context must not turn the new plan's zero-command startup
  // window into a controller failure.
  if (!plan_valid_ || candidate.path_version != path_version_ ||
      candidate.goal_version != goal_version_) {
    publishDiagnostic("info", "discarded_envelope_context_mismatch");
    return;
  }
  if (worker_generation_ != -1 && candidate.worker_generation != worker_generation_) {
    envelope_ = Envelope{};
    publishDiagnostic("error", "worker_generation_changed");
    return;
  }
  worker_generation_ = candidate.worker_generation;
  envelope_ = candidate;
  envelope_recovery_active_ = false;
}

void NeuPANController::gateCommandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gate_command_ = *message;
  gate_command_received_ = true;
}

void NeuPANController::gateStatusCallback(const std_msgs::msg::String::SharedPtr message)
{
  GateStatus candidate;
  try {
    const auto payload = nlohmann::json::parse(message->data);
    if (payload.value("schema", "") != "r1_n2_pre_safety_gate_v2") {
      throw std::runtime_error("schema_invalid");
    }
    candidate.input_linear_x = payload.at("input_linear_x").get<double>();
    candidate.input_angular_z = payload.at("input_angular_z").get<double>();
    candidate.output_linear_x = payload.at("output_linear_x").get<double>();
    candidate.output_angular_z = payload.at("output_angular_z").get<double>();
    candidate.emergency_stop = payload.at("emergency_stop").get<bool>();
    candidate.failure_latched = payload.at("failure_latched").get<bool>();
    const auto & metadata = payload.at("freshness").at("source_metadata");
    // Before the first valid planner result the gate has no envelope identity.
    // That is a valid state only while the operator e-stop is asserted; a
    // released gate must always provide the full identity below.
    if (metadata.empty()) {
      if (!candidate.emergency_stop || candidate.failure_latched) {
        throw std::runtime_error("missing_source_metadata");
      }
    } else {
      candidate.planner_result_monotonic = metadata.at("planner_result_monotonic").get<double>();
      candidate.path_version = metadata.at("path_version").get<int>();
      candidate.goal_version = metadata.at("goal_version").get<int>();
    }
    if (!finite(candidate.input_linear_x) || !finite(candidate.input_angular_z) ||
        !finite(candidate.output_linear_x) || !finite(candidate.output_angular_z) ||
        (!metadata.empty() && !finite(candidate.planner_result_monotonic))) {
      throw std::runtime_error("nonfinite");
    }
    candidate.valid = true;
    candidate.received_at = std::chrono::steady_clock::now();
  } catch (const std::exception & error) {
    publishDiagnostic("error", std::string("invalid_gate_status:") + error.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  gate_status_ = candidate;
}

void NeuPANController::guardStatusCallback(const std_msgs::msg::String::SharedPtr message)
{
  if (!shadow_mode_) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {execution_->guard(nlohmann::json::parse(message->data), monotonicNow());}
    catch (const std::exception &) {if (execution_->active()) {execution_->fail("invalid_guard_status");}}
  }
  GuardStatus candidate;
  try {
    const auto payload = nlohmann::json::parse(message->data);
    if (payload.value("schema", "") != "r1_collision_guard_status_v1") {
      throw std::runtime_error("schema_invalid");
    }
    candidate.state = payload.at("state").get<std::string>();
    candidate.reason = payload.at("reason").get<std::string>();
    if (candidate.state != "PASS" && candidate.state != "SLOW" &&
      candidate.state != "STOP" && candidate.state != "FAULT")
    {
      throw std::runtime_error("state_invalid");
    }
    candidate.valid = true;
    candidate.received_at = std::chrono::steady_clock::now();
  } catch (const std::exception & error) {
    publishDiagnostic("error", std::string("invalid_collision_guard_status:") + error.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  guard_status_ = std::move(candidate);
}

void NeuPANController::localizationHealthCallback(const std_msgs::msg::String::SharedPtr message)
{
  LocalizationHealth candidate;
  try {
    const auto payload = nlohmann::json::parse(message->data);
    if (payload.value("schema", "") != "r1_localization_health_v1") {
      throw std::runtime_error("schema_invalid");
    }
    candidate.state = payload.at("state").get<std::string>();
    candidate.reason = payload.at("reason").get<std::string>();
    if (candidate.state != "BOOTSTRAP" && candidate.state != "READY" &&
      candidate.state != "FAULT")
    {
      throw std::runtime_error("state_invalid");
    }
    candidate.valid = true;
    candidate.received_at = std::chrono::steady_clock::now();
  } catch (const std::exception & error) {
    publishDiagnostic("error", std::string("invalid_localization_health:") + error.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  localization_health_ = std::move(candidate);
}

void NeuPANController::publishDiagnostic(const std::string & level, const std::string & reason) const
{
  if (!diagnostics_pub_) {
    return;
  }
  std_msgs::msg::String message;
  message.data = nlohmann::json{{"schema", "r1_neupan_controller_status_v1"},
    {"level", level}, {"reason", reason}, {"monotonic", monotonicNow()}}.dump();
  diagnostics_pub_->publish(message);
}

void NeuPANController::rejectPlan(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_valid_ = false;
    envelope_ = Envelope{};
  }
  publishDiagnostic("error", reason);
}

double NeuPANController::monotonicNow() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool NeuPANController::finite(double value)
{
  return std::isfinite(value);
}

}  // namespace navigation_neupan_bridge

PLUGINLIB_EXPORT_CLASS(navigation_neupan_bridge::NeuPANController, nav2_core::Controller)
