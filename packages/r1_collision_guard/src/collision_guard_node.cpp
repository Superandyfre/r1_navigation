// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#include "r1_collision_guard/guard_logic.hpp"
#include "r1_neupan_runtime/motion_authorization.hpp"
#include "r1_neupan_runtime/command_barrier.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace r1_collision_guard
{

namespace
{
using SteadyTime = std::chrono::steady_clock::time_point;

struct OdomSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double linear_x{0.0};
  double angular_z{0.0};
  std::string frame;
  std::string child_frame;
  SteadyTime received{};
};

struct LocalizationHealth
{
  std::string state;
  std::string reason;
  SteadyTime received{};
  bool available{false};
};

struct ArmState
{
  bool available{false};
  bool valid{false};
  bool stowed{false};
  bool moving{false};
  double max_position_error{0.0};
  SteadyTime received{};
  SteadyTime oldest_joint_received{};
  SteadyTime stable_since{};
};

}  // namespace

class CollisionGuardNode : public rclcpp::Node
{
public:
  CollisionGuardNode()
  : Node("r1_collision_guard")
  {
    max_linear_mps_ = declare_parameter<double>("max_linear_mps", 0.15);
    max_reverse_mps_ = declare_parameter<double>("max_reverse_mps", 0.15);
    max_angular_radps_ = declare_parameter<double>("max_angular_radps", 0.4);
    // There is no runtime bypass for motion authorization. Old producer stacks
    // without the new grant and source contract remain disarmed.
    r1_neupan_runtime::AuthorizationConfig authorization_config;
    authorization_config.health_timeout = declare_parameter<double>("worker_health_timeout_sec", 0.50);
    authorization_config.source_timeout = declare_parameter<double>("planner_source_timeout_sec", 0.50);
    authorization_config.input_timeout = declare_parameter<double>("planner_input_timeout_sec", 0.75);
    authorization_config.authority_timeout = declare_parameter<double>("navigation_authority_timeout_sec", 0.50);
    authorization_config.recovery_timeout = declare_parameter<double>("planner_recovery_timeout_sec", 5.0);
    authorization_ = std::make_unique<r1_neupan_runtime::MotionAuthorization>(
      r1_neupan_runtime::newBootId(), authorization_config);
    const auto worker_health_topic = declare_parameter<std::string>("worker_health_topic", "/neupan_worker_health");
    const auto planner_source_topic = declare_parameter<std::string>("planner_source_topic", "/neupan_planner_result");
    const auto motion_grant_topic = declare_parameter<std::string>("motion_grant_topic", "/neupan_motion_grant");
    control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.15);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.20);
    source_command_timeout_sec_ = declare_parameter<double>("source_command_timeout_sec", command_timeout_sec_);
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto command_topic = declare_parameter<std::string>(
      "cmd_vel_in_topic", "/neupan_nav2_cmd_vel");
    const auto source_command_topic = declare_parameter<std::string>(
      "source_cmd_vel_topic", command_topic);
    same_command_topics_ = command_topic == source_command_topic;
    const auto output_topic = declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel");
    const auto smoother_provenance_topic = declare_parameter<std::string>(
      "smoother_provenance_topic", "/neupan_nav2_smoothed_provenance");
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/r1_collision_guard/status");
    const auto localization_health_topic = declare_parameter<std::string>(
      "localization_health_topic", "/r1_localization_health");
    localization_health_timeout_sec_ = declare_parameter<double>(
      "localization_health_timeout_sec", 0.50);
    require_localization_ready_ = declare_parameter<bool>(
      "require_localization_ready", true);
    require_arms_stowed_ = declare_parameter<bool>("require_arms_stowed", true);
    joint_state_timeout_sec_ = declare_parameter<double>("joint_state_timeout_sec", 0.50);
    arm_stow_tolerance_rad_ = declare_parameter<double>("arm_stow_tolerance_rad", 0.035);
    arm_motion_tolerance_radps_ = declare_parameter<double>(
      "arm_motion_tolerance_radps", 0.020);
    arm_settle_time_sec_ = declare_parameter<double>("arm_settle_time_sec", 1.0);
    arm_joint_names_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names", std::vector<std::string>{});
    arm_stowed_positions_ = declare_parameter<std::vector<double>>(
      "arm_stowed_positions", std::vector<double>{});
    arm_joint_state_topics_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_state_topics", std::vector<std::string>{});

    if (max_linear_mps_ <= 0.0 || max_reverse_mps_ <= 0.0 || max_angular_radps_ <= 0.0 ||
      control_frequency_ <= 0.0 || odom_timeout_sec_ <= 0.0 || command_timeout_sec_ <= 0.0 ||
      source_command_timeout_sec_ <= 0.0 || localization_health_timeout_sec_ <= 0.0 ||
      joint_state_timeout_sec_ <= 0.0 || arm_stow_tolerance_rad_ <= 0.0 ||
      arm_motion_tolerance_radps_ < 0.0 || arm_settle_time_sec_ < 0.0)
    {throw std::runtime_error("invalid R1 command guard parameters");}
    if (require_arms_stowed_ && (arm_joint_names_.empty() ||
      arm_joint_names_.size() != arm_stowed_positions_.size() ||
      arm_joint_state_topics_.empty()))
    {
      throw std::runtime_error("arm stow guard requires matching joint names and positions");
    }
    recovery_stop_ = create_service<std_srvs::srv::Trigger>("~/stop_for_recovery",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        authorization_->latch("controlled_runtime_recovery");
        response->success = true;
        response->message = "motion_revoked_restart_required";
      });
    worker_health_sub_ = create_subscription<std_msgs::msg::String>(
      worker_health_topic, rclcpp::QoS(20), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        authorization_->health(parseProtocol(msg->data), monotonicNow());
      });
    planner_source_sub_ = create_subscription<std_msgs::msg::String>(
      planner_source_topic, rclcpp::QoS(20), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        authorization_->source(parseProtocol(msg->data), monotonicNow());
      });
    motion_grant_sub_ = create_subscription<std_msgs::msg::String>(
      motion_grant_topic, rclcpp::QoS(20), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto previous_grant = authorization_->grantId();
        const bool was_committed = authorization_->committed();
        auto request = parseProtocol(msg->data);
        const bool seamless = seamlessHandoverReady(monotonicNow());
        if (request.is_object() && request.contains("state") && request["state"] == "COMMIT" &&
          !previous_grant.empty() && request.contains("grant_id") && request["grant_id"] == previous_grant &&
          !was_committed && !seamless &&
          !command_barrier_.ready(monotonicNow(), source_command_timeout_sec_, command_timeout_sec_))
        {
          // Refresh the preparation lease, but never commit a queued old tail.
          request["state"] = "PREPARE";
        }
        authorization_->authorization(request, monotonicNow());
        if (authorization_->grantId() != previous_grant) {
          command_barrier_.reset(); smoother_provenance_.clear();
        }
        if (authorization_->grantId() != previous_grant ||
          (!was_committed && authorization_->committed())) {
          ++command_epoch_;
          const bool retaining_old = authorization_->handoverFresh(monotonicNow());
          if (!retaining_old && !seamless) {
            command_available_ = false;
            source_command_available_ = false;
          }
        }
      });
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::commandCallback, this, std::placeholders::_1));
    source_command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      source_command_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::sourceCommandCallback, this, std::placeholders::_1));
    if (!same_command_topics_) {
      smoother_provenance_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        smoother_provenance_topic, rclcpp::QoS(20),
        std::bind(&CollisionGuardNode::smootherProvenanceCallback, this, std::placeholders::_1));
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50),
      std::bind(&CollisionGuardNode::odomCallback, this, std::placeholders::_1));
    localization_health_sub_ = create_subscription<std_msgs::msg::String>(
      localization_health_topic, rclcpp::QoS(20),
      std::bind(&CollisionGuardNode::localizationHealthCallback, this, std::placeholders::_1));
    for (const auto & topic : arm_joint_state_topics_) {
      arm_joint_state_subs_.push_back(create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::QoS(20),
        std::bind(&CollisionGuardNode::jointStateCallback, this, std::placeholders::_1)));
    }
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(20));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(20));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&CollisionGuardNode::tick, this));
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          const auto & name = parameter.get_name();
          if (name == "worker_health_timeout_sec" || name == "planner_source_timeout_sec" ||
            name == "planner_input_timeout_sec" || name == "navigation_authority_timeout_sec" ||
            name == "planner_recovery_timeout_sec" || name == "worker_health_topic" ||
            name == "planner_source_topic" || name == "motion_grant_topic")
          {
            result.successful = false;
            result.reason = "authorization configuration requires controlled restart";
            return result;
          }
        }
        return result;
      });
    RCLCPP_INFO(get_logger(), "R1 collision guard active: %s (source %s) -> %s at %.1f Hz",
      command_topic.c_str(), source_command_topic.c_str(), output_topic.c_str(), control_frequency_);
  }

private:
  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_barrier_.observe(false,
      {message->linear.x, message->linear.y, message->linear.z,
        message->angular.x, message->angular.y, message->angular.z}, monotonicNow());
    command_ = *message;
    command_received_ = std::chrono::steady_clock::now();
    command_available_ = true;

  }

  void sourceCommandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_barrier_.observe(true,
      {message->linear.x, message->linear.y, message->linear.z,
        message->angular.x, message->angular.y, message->angular.z}, monotonicNow());
    source_command_ = *message;
    source_command_received_ = std::chrono::steady_clock::now();
    source_command_available_ = true;
    source_command_valid_ = std::isfinite(message->linear.x) && std::isfinite(message->angular.z);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    OdomSample sample;
    sample.stamp = message->header.stamp;
    sample.linear_x = message->twist.twist.linear.x;
    sample.angular_z = message->twist.twist.angular.z;
    sample.frame = message->header.frame_id;
    sample.child_frame = message->child_frame_id;
    sample.received = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    odom_history_.push_back(sample);
    while (odom_history_.size() > 240) {
      odom_history_.pop_front();
    }
  }

  void smootherProvenanceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (message->layout.dim.empty() || message->layout.dim[0].label != "r1_smoother_provenance_v1" ||
      message->data.size() != 5 ||
      !std::all_of(message->data.begin(), message->data.end(), [](double x) {return std::isfinite(x);})) {return;}
    const double now = monotonicNow();
    if (message->data[0] <= 0.0 || message->data[0] > now ||
      now - message->data[0] > command_timeout_sec_) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    smoother_provenance_.push_back({message->data[0], message->data[1], message->data[2],
      message->data[3], message->data[4], now});
    while (smoother_provenance_.size() > 16) {smoother_provenance_.pop_front();}
  }

  // mutex_ held. Smoothed mode requires evidence emitted by the smoother
  // itself after consuming a new-route input and computing this exact output.
  bool seamlessHandoverReady(double now) const
  {
    const double raw_time = std::chrono::duration<double>(source_command_received_.time_since_epoch()).count();
    const double smooth_time = std::chrono::duration<double>(command_received_.time_since_epoch()).count();
    if (!command_available_ || !source_command_available_ || !source_command_valid_ ||
      now < smooth_time || now - smooth_time > command_timeout_sec_ ||
      now < raw_time || now - raw_time > source_command_timeout_sec_ ||
      !authorization_->handoverMatchesNewCommand(source_command_.linear.x, source_command_.angular.z, raw_time, now)) {
      return false;
    }
    if (same_command_topics_) {
      return command_.linear.x == source_command_.linear.x && command_.angular.z == source_command_.angular.z;
    }
    for (auto it = smoother_provenance_.rbegin(); it != smoother_provenance_.rend(); ++it) {
      if (now >= it->received && now - it->received <= command_timeout_sec_ &&
        it->output_linear == command_.linear.x && it->output_angular == command_.angular.z &&
        authorization_->handoverMatchesNewCommand(it->input_linear, it->input_angular, it->input_time, now)) {
        return true;
      }
    }
    return false;
  }

  void localizationHealthCallback(const std_msgs::msg::String::SharedPtr message)
  {
    LocalizationHealth health;
    try {
      const auto payload = nlohmann::json::parse(message->data);
      if (payload.value("schema", "") != "r1_localization_health_v1") {
        throw std::runtime_error("schema_invalid");
      }
      health.state = payload.at("state").get<std::string>();
      health.reason = payload.at("reason").get<std::string>();
      if (health.state != "BOOTSTRAP" && health.state != "READY" && health.state != "HOLD" && health.state != "FAULT") {
        throw std::runtime_error("state_invalid");
      }
      health.available = true;
      health.received = std::chrono::steady_clock::now();
    } catch (const std::exception &) {
      health.state = "FAULT";
      health.reason = "status_parse_error";
      health.available = true;
      health.received = std::chrono::steady_clock::now();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (health.state == "HOLD" && !localization_hold_seen_) {
      localization_hold_seen_ = true;
      localization_recovery_zero_seen_ = false;
      command_barrier_.reset();
    }
    if (health.state == "READY") {
      localization_ready_seen_ = true;
      if (localization_hold_seen_) {
        localization_recovery_after_ = monotonicNow();
        localization_hold_seen_ = false;
      }
    }
    localization_health_ = std::move(health);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    ArmState arm;
    arm.available = true;
    arm.received = std::chrono::steady_clock::now();
    std::unordered_map<std::string, size_t> indices;
    for (size_t index = 0; index < message->name.size(); ++index) {
      indices.emplace(message->name[index], index);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    bool message_valid = message->name.size() == message->position.size();
    for (const auto & joint_name : arm_joint_names_) {
      const auto iterator = indices.find(joint_name);
      if (iterator == indices.end()) {
        continue;
      }
      const double position = message->position[iterator->second];
      if (!std::isfinite(position)) {
        message_valid = false;
        continue;
      }
      const auto prior = arm_positions_.find(joint_name);
      const auto prior_received = arm_joint_received_.find(joint_name);
      if (prior != arm_positions_.end() && prior_received != arm_joint_received_.end()) {
        const double elapsed = std::chrono::duration<double>(
          arm.received - prior_received->second).count();
        if (elapsed > 1e-4 &&
          std::abs(position - prior->second) / elapsed > arm_motion_tolerance_radps_)
        {
          arm.moving = true;
        }
      }
      if (message->velocity.size() == message->name.size()) {
        const double velocity = message->velocity[iterator->second];
        if (!std::isfinite(velocity)) {
          message_valid = false;
        } else if (std::abs(velocity) > arm_motion_tolerance_radps_) {
          arm.moving = true;
        }
      }
      arm_positions_[joint_name] = position;
      arm_joint_received_[joint_name] = arm.received;
    }
    arm.valid = message_valid;
    for (size_t index = 0; index < arm_joint_names_.size(); ++index) {
      const auto position = arm_positions_.find(arm_joint_names_[index]);
      const auto received = arm_joint_received_.find(arm_joint_names_[index]);
      if (position == arm_positions_.end() || received == arm_joint_received_.end()) {
        arm.valid = false;
        break;
      }
      arm.max_position_error = std::max(
        arm.max_position_error, std::abs(position->second - arm_stowed_positions_[index]));
      if (arm.oldest_joint_received == SteadyTime{} ||
        received->second < arm.oldest_joint_received)
      {
        arm.oldest_joint_received = received->second;
      }
    }
    arm.stowed = arm.valid && arm.max_position_error <= arm_stow_tolerance_rad_;
    if (arm.valid && arm.stowed && !arm.moving) {
      arm.stable_since = (arm_state_.valid && arm_state_.stowed && !arm_state_.moving)
        ? arm_state_.stable_since : arm.received;
    }
    arm_state_ = std::move(arm);
  }

  Decision fault(const std::string & reason) const
  {
    Decision decision;
    decision.state = Decision::State::FAULT;
    decision.reason = reason;
    return decision;
  }

  Decision stop(const std::string & reason) const
  {
    Decision decision;
    decision.state = Decision::State::STOP;
    decision.reason = reason;
    return decision;
  }

  static double monotonicNow()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static nlohmann::json parseProtocol(const std::string & data)
  {
    const auto parsed = nlohmann::json::parse(data, nullptr, false);
    return parsed.is_discarded() ? nlohmann::json::object() : parsed;
  }

  void tick()
  {
    geometry_msgs::msg::Twist command;
    std::deque<OdomSample> odom_history;
    SteadyTime command_received{};
    SteadyTime source_command_received{};
    std::uint64_t command_epoch = 0;
    bool command_available = false;
    bool source_command_available = false;
    bool source_command_valid = false;
    bool localization_ready_seen = false;
    LocalizationHealth localization_health;
    ArmState arm_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_epoch = command_epoch_;
      command = command_;
      odom_history = odom_history_;
      command_received = command_received_;
      source_command_received = source_command_received_;
      command_available = command_available_;
      source_command_available = source_command_available_;
      source_command_valid = source_command_valid_;
      localization_ready_seen = localization_ready_seen_;
      localization_health = localization_health_;
      arm_state = arm_state_;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    const auto ros_now = now();
    Decision decision;
    const bool localization_health_stale = !localization_health.available ||
      std::chrono::duration<double>(steady_now - localization_health.received).count() >
      localization_health_timeout_sec_;
    const bool arm_state_stale = !arm_state.available ||
      arm_state.oldest_joint_received == SteadyTime{} ||
      std::chrono::duration<double>(steady_now - arm_state.oldest_joint_received).count() >
      joint_state_timeout_sec_;
    if (require_localization_ready_ && !localization_ready_seen &&
      (localization_health_stale || localization_health.state == "BOOTSTRAP"))
    {
      // A map-to-odom transform is not expected until the operator supplies an
      // initial pose. Keep the base stopped, but do not turn that startup state
      // into an irreversible task fault.
      decision = stop("waiting_for_localization_ready");
    } else if (require_localization_ready_ && localization_health_stale) {
      decision = fault("localization_status_stale");
    } else if (require_localization_ready_ && localization_health.state == "HOLD") {
      decision = stop("localization_hold:" + localization_health.reason);
    } else if (require_localization_ready_ && localization_health.state != "READY") {
      decision = fault("localization_" + localization_health.state + ":" + localization_health.reason);
    } else if (require_arms_stowed_ && arm_state_stale) {
      decision = fault("arm_joint_state_stale");
    } else if (require_arms_stowed_ && !arm_state.valid) {
      decision = fault("arm_joint_state_invalid");
    } else if (require_arms_stowed_ && !arm_state.stowed) {
      decision = fault("arm_not_stowed");
    } else if (require_arms_stowed_ && arm_state.moving) {
      decision = fault("arm_moving");
    } else if (require_arms_stowed_ &&
      std::chrono::duration<double>(steady_now - arm_state.stable_since).count() <
      arm_settle_time_sec_)
    {
      decision = stop("arm_stabilizing");
    } else if (!source_command_available ||
      std::chrono::duration<double>(steady_now - source_command_received).count() > source_command_timeout_sec_)
    {
      decision = fault("source_command_stale");
    } else if (!source_command_valid) {
      decision = fault("source_command_nonfinite");
    } else if (!command_available || std::chrono::duration<double>(steady_now - command_received).count() > command_timeout_sec_) {
      decision = fault("command_stale");
    } else if (odom_history.empty() ||
      std::chrono::duration<double>(steady_now - odom_history.back().received).count() > odom_timeout_sec_) {
      decision = fault("odom_stale");
    } else if (odom_history.back().stamp.nanoseconds() == 0 ||
      (ros_now - odom_history.back().stamp).seconds() < -0.02 ||
      (ros_now - odom_history.back().stamp).seconds() > odom_timeout_sec_)
    {
      decision = fault("odom_stamp_stale_or_future");
    } else if (odom_history.back().frame != odom_frame_) {
      decision = fault("odom_frame_mismatch");
    } else if (odom_history.back().child_frame != base_frame_) {
      decision = fault("odom_child_frame_mismatch");
    } else if (!std::isfinite(command.linear.x) || !std::isfinite(command.angular.z) ||
      !std::isfinite(odom_history.back().linear_x) || !std::isfinite(odom_history.back().angular_z))
    {
      decision = fault("nonfinite_input");
    } else {
      decision.state = Decision::State::PASS;
      decision.reason = "planner_command";
      decision.linear_x = std::clamp(command.linear.x, -max_reverse_mps_, max_linear_mps_);
      decision.angular_z = std::clamp(command.angular.z, -max_angular_radps_, max_angular_radps_);
    }
    // Recheck command ownership at the actuator boundary.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (command_epoch != command_epoch_) {decision = stop("command_epoch_changed");}
      // A HOLD can arrive after the timer copied its inputs; do not let that
      // stale snapshot emit an additional command at the actuator boundary.
      if (require_localization_ready_ && localization_health_.state == "HOLD") {
        decision = stop("localization_hold:" + localization_health_.reason);
      }
      physical_safety_state_ = stateName(decision.state);
      authorization_decision_ = authorization_->evaluate(monotonicNow());
      if (authorization_decision_.state == "ROUTE_HANDOVER") {
        const auto & old = authorization_->handoverSource();
        const double raw_time = std::chrono::duration<double>(source_command_received_.time_since_epoch()).count();
        const bool old_raw = old && source_command_.linear.x == old->linear &&
          source_command_.angular.z == old->angular;
        const bool new_raw = authorization_->handoverMatchesNewCommand(
          source_command_.linear.x, source_command_.angular.z, raw_time, monotonicNow());
        if (!source_command_available_ || !source_command_valid_ || (!old_raw && !new_raw)) {
          decision = stop("route_handover_waiting_for_verified_command");
        }
      }
      if (localization_hold_seen_ || localization_recovery_after_ > 0.0) {
        if (command_barrier_.ready(monotonicNow(), source_command_timeout_sec_, command_timeout_sec_)) {
          localization_recovery_zero_seen_ = true;
        }
      }
      if (localization_recovery_after_ > 0.0) {
        const auto source = authorization_->latestSource();
        const double raw_time = std::chrono::duration<double>(source_command_received_.time_since_epoch()).count();
        const double output_time = std::chrono::duration<double>(command_received_.time_since_epoch()).count();
        if (!localization_recovery_zero_seen_ || !source || source->input <= localization_recovery_after_ ||
          raw_time <= source->published || output_time <= localization_recovery_after_)
        {
          decision = stop("awaiting_post_localization_recovery_command");
        } else {
          localization_recovery_after_ = 0.0;
        }
      }
      if (!authorization_decision_.allowed) {
        decision = authorization_decision_.fault ? fault(authorization_decision_.reason) :
          stop(authorization_decision_.reason);
      }
      geometry_msgs::msg::Twist output;
      output.linear.x = decision.linear_x;
      output.angular.z = decision.angular_z;
      command_pub_->publish(output);
    }
    publishStatus(
      decision, command, odom_history, localization_health, command_available, command_received,
      source_command_available, source_command_valid, source_command_received, arm_state,
      steady_now, command_epoch);
  }

  void publishStatus(const Decision & decision, const geometry_msgs::msg::Twist & command,
    const std::deque<OdomSample> & odom_history, const LocalizationHealth & localization_health,
    const bool command_available, const SteadyTime & command_received,
    const bool source_command_available, const bool source_command_valid,
    const SteadyTime & source_command_received, const ArmState & arm_state,
    const SteadyTime & steady_now, const std::uint64_t command_epoch)
  {
    std_msgs::msg::String message;
    // JSON serialization escapes diagnostic strings and emits null for
    // nonfinite requested input, so the safety status itself stays parseable.
    nlohmann::json payload = {
      {"schema", "r1_collision_guard_status_v1"},
      {"state", stateName(decision.state)}, {"reason", decision.reason},
      {"requested_linear_x", command.linear.x}, {"requested_angular_z", command.angular.z},
      {"output_linear_x", decision.linear_x}, {"output_angular_z", decision.angular_z},
      {"localization_state", localization_health.state}, {"localization_reason", localization_health.reason},
      {"arm_state", !require_arms_stowed_ ? "NOT_REQUIRED" :
        (!arm_state.available ? "MISSING" : !arm_state.valid ? "INVALID" :
        !arm_state.stowed ? "NOT_STOWED" : arm_state.moving ? "MOVING" : "STOWED")},
      {"arm_max_position_error_rad", arm_state.max_position_error},
      {"source_command_ready", source_command_available && source_command_valid}};
    if (command_available) {
      payload["command_age_sec"] = std::chrono::duration<double>(steady_now - command_received).count();
    }
    if (source_command_available) {
      payload["source_command_age_sec"] =
        std::chrono::duration<double>(steady_now - source_command_received).count();
    }
    if (!odom_history.empty()) {
      payload["actual_linear_x"] = odom_history.back().linear_x;
      payload["actual_angular_z"] = odom_history.back().angular_z;
      payload["odom_stamp_ns"] = rclcpp::Time(odom_history.back().stamp).nanoseconds();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Do not acknowledge a new grant using an output decision from an old
      // command snapshot. The next tick produces a coherent zero/status pair.
      if (command_epoch != command_epoch_) {return;}
      payload.update(nlohmann::json{
        {"authorization_protocol", "r1_neupan_motion_grant_v1"},
        {"health_protocol", r1_neupan_runtime::kHealthSchema}, {"source_protocol", "r1_neupan_planner_source_v2"}, {"build_revision", R1_FAILSTOP_BUILD_REVISION},
        {"route_handover_seamless_ready", seamlessHandoverReady(monotonicNow())},
        {"command_barrier_ready", seamlessHandoverReady(monotonicNow()) ||
          command_barrier_.ready(monotonicNow(), source_command_timeout_sec_, command_timeout_sec_)},
        {"sequence", ++status_sequence_}, {"monotonic", monotonicNow()},
        {"worker_health_age_sec", authorization_->healthAge(monotonicNow())},
        {"worker_health_bridged_by_source", authorization_->healthBridged(monotonicNow())},
        {"guard_boot_id", authorization_->bootId()}, {"authorization_state", authorization_decision_.state},
        {"authorization_reason", authorization_decision_.reason}, {"physical_safety_state", physical_safety_state_},
        {"grant_id", authorization_->grantId()}, {"navigation_attempt_id", authorization_->attemptId()}});
      if (const auto & context = authorization_->grantContext()) {
        payload.update(nlohmann::json{
          {"authority_boot_id", context->authority_boot}, {"controller_boot_id", context->controller_boot},
          {"worker_boot_id", context->worker_boot}, {"worker_generation", context->generation},
          {"path_version", context->path}, {"goal_version", context->goal}});
      }
    }
    message.data = payload.dump();
    status_pub_->publish(message);
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_stop_;
  r1_neupan_runtime::CommandBarrier command_barrier_;
  std::uint64_t command_epoch_{0};
  std::int64_t status_sequence_{0};
  std::unique_ptr<r1_neupan_runtime::MotionAuthorization> authorization_;
  r1_neupan_runtime::AuthorizationDecision authorization_decision_;
  std::string physical_safety_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr worker_health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_source_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_grant_sub_;
  double max_linear_mps_{0.15};
  double max_reverse_mps_{0.15};
  double max_angular_radps_{0.4};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  double control_frequency_{50.0};
  double odom_timeout_sec_{0.15};
  double command_timeout_sec_{0.20};
  double source_command_timeout_sec_{0.20};
  double localization_health_timeout_sec_{0.50};
  double joint_state_timeout_sec_{0.50};
  double arm_stow_tolerance_rad_{0.035};
  double arm_motion_tolerance_radps_{0.020};
  double arm_settle_time_sec_{1.0};
  bool require_localization_ready_{true};
  bool require_arms_stowed_{true};
  std::string base_frame_;
  std::string odom_frame_;
  std::vector<std::string> arm_joint_names_;
  std::vector<double> arm_stowed_positions_;
  std::vector<std::string> arm_joint_state_topics_;
  std::mutex mutex_;
  geometry_msgs::msg::Twist command_;
  geometry_msgs::msg::Twist source_command_;
  std::deque<OdomSample> odom_history_;
  bool same_command_topics_{false};
  struct SmootherProvenance {
    double input_time, input_linear, input_angular, output_linear, output_angular, received;
  };
  std::deque<SmootherProvenance> smoother_provenance_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr smoother_provenance_sub_;
  bool localization_hold_seen_{false};
  bool localization_recovery_zero_seen_{false};
  double localization_recovery_after_{0.0};
  SteadyTime command_received_{};
  SteadyTime source_command_received_{};
  bool command_available_{false};
  bool source_command_available_{false};
  bool source_command_valid_{false};
  bool localization_ready_seen_{false};
  LocalizationHealth localization_health_;
  ArmState arm_state_;
  std::unordered_map<std::string, double> arm_positions_;
  std::unordered_map<std::string, SteadyTime> arm_joint_received_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr source_command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_health_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> arm_joint_state_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace r1_collision_guard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r1_collision_guard::CollisionGuardNode>());
  rclcpp::shutdown();
  return 0;
}
