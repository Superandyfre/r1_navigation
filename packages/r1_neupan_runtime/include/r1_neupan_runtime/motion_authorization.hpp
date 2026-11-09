// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <vector>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <nlohmann/json.hpp>

namespace r1_neupan_runtime
{
using Json = nlohmann::json;
inline constexpr char kHealthSchema[] = "r1_neupan_worker_health_v2";
inline constexpr char kSourceSchema[] = "r1_neupan_planner_source_v2";
inline constexpr char kGrantSchema[] = "r1_neupan_motion_grant_v1";

inline std::string requiredString(const Json & j, const char * key)
{
  const auto & value = j.at(key);
  if (!value.is_string() || value.get<std::string>().empty()) {
    throw std::runtime_error(std::string("invalid_") + key);
  }
  return value.get<std::string>();
}
inline std::int64_t requiredInt(const Json & j, const char * key, std::int64_t minimum = 0)
{
  const auto & v = j.at(key);
  if (!v.is_number_integer() || (v.is_number_unsigned() &&
    v.get<std::uint64_t>() > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())))
  {
    throw std::runtime_error(std::string("invalid_") + key);
  }
  const auto result = v.get<std::int64_t>();
  if (result < minimum) {throw std::runtime_error(std::string("invalid_") + key);}
  return result;
}
inline double requiredNumber(const Json & j, const char * key, bool positive = true)
{
  const auto & v = j.at(key);
  if (!v.is_number()) {throw std::runtime_error(std::string("invalid_") + key);}
  const double result = v.get<double>();
  if (!std::isfinite(result) || (positive && result <= 0.0)) {
    throw std::runtime_error(std::string("invalid_") + key);
  }
  return result;
}
inline void requireSchema(const Json & j, const char * schema)
{
  if (!j.is_object() || requiredString(j, "schema") != schema) {
    throw std::runtime_error("protocol_version_mismatch");
  }
}
inline std::string newBootId()
{
  std::random_device random;
  std::ostringstream id;
  id << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) {id << std::setw(8) << random();}
  return id.str();
}
struct Context
{
  std::string authority_boot, controller_boot, attempt, worker_boot;
  std::int64_t generation{-1}, path{-1}, goal{-1};
  bool operator==(const Context & b) const
  {
    return authority_boot == b.authority_boot && controller_boot == b.controller_boot &&
           attempt == b.attempt && worker_boot == b.worker_boot && generation == b.generation &&
           path == b.path && goal == b.goal;
  }
};
inline Context parseContext(const Json & j)
{
  return {requiredString(j, "authority_boot_id"), requiredString(j, "controller_boot_id"),
    requiredString(j, "navigation_attempt_id"), requiredString(j, "worker_boot_id"),
    requiredInt(j, "worker_generation", 1), requiredInt(j, "path_version"),
    requiredInt(j, "goal_version")};
}
struct Source
{
  Context context;
  std::int64_t request{}, cycle{}, scan{};
  double input{}, dispatched{}, solved{}, published{}, linear{}, angular{}, received{};
  double trajectory_step{};
  std::vector<std::array<double,2>> trajectory;
  bool sameIdentity(const Source & b) const
  {
    return context == b.context && request == b.request && cycle == b.cycle && scan == b.scan;
  }
  bool sameContents(const Source & b) const
  {
    return sameIdentity(b) && input == b.input && dispatched == b.dispatched && solved == b.solved &&
           published == b.published && linear == b.linear && angular == b.angular &&
           trajectory_step == b.trajectory_step && trajectory == b.trajectory;
  }
};
inline Source parseSource(const Json & j, double received)
{
  requireSchema(j, kSourceSchema);
  if (!j.at("finite").is_boolean() || !j.at("finite").get<bool>()) {
    throw std::runtime_error("source_nonfinite");
  }
  Source s;
  s.context = parseContext(j);
  s.request = requiredInt(j, "request_id", 1);
  s.cycle = requiredInt(j, "cycle_id", 1);
  s.scan = requiredInt(j, "scan_counter", 1);
  s.input = requiredNumber(j, "scan_receive_monotonic");
  s.dispatched = requiredNumber(j, "request_dispatched_monotonic");
  s.solved = requiredNumber(j, "worker_solve_end_monotonic");
  s.published = requiredNumber(j, "planner_result_monotonic");
  s.linear = requiredNumber(j.at("command"), "linear_x", false);
  s.angular = requiredNumber(j.at("command"), "angular_z", false);
  if (j.contains("trajectory")) {
    const auto & t=j.at("trajectory");
    s.trajectory_step=requiredNumber(t,"step_sec");
    const auto & controls=t.at("controls");
    if (!controls.is_array() || controls.empty() || controls.size()>1000) {
      throw std::runtime_error("invalid_source_trajectory");
    }
    for (const auto & u:controls) {
      if (!u.is_array() || u.size()!=2 || !u[0].is_number() || !u[1].is_number()) {
        throw std::runtime_error("invalid_source_trajectory_action");
      }
      const double linear=u[0].get<double>(),angular=u[1].get<double>();
      if (!std::isfinite(linear) || !std::isfinite(angular)) {
        throw std::runtime_error("invalid_source_trajectory_action");
      }
      s.trajectory.push_back({linear,angular});
    }
    if (std::abs(s.trajectory[0][0]-s.linear)>1e-7 ||
        std::abs(s.trajectory[0][1]-s.angular)>1e-7) {
      throw std::runtime_error("source_trajectory_action_mismatch");
    }
  }
  s.received = received;
  if (s.input > s.dispatched || s.dispatched > s.solved || s.solved > s.published ||
    s.published > received)
  {
    throw std::runtime_error("source_timestamp_order");
  }
  return s;
}
inline constexpr char kTrajectoryProofSchema[] = "r1_neupan_trajectory_proof_v1";

// This verifies provenance only. Motion still requires the live task grant,
// worker health, fresh pose/scan and checks at the final actuator boundary.
class TrajectoryProofVerifier
{
public:
  void install(const Source & source, double step, const std::vector<std::array<double,2>> & controls)
  {
    if (!std::isfinite(step) || step<=0. || controls.empty() || controls.size()>1000) {
      throw std::invalid_argument("invalid_trajectory_horizon");
    }
    for (const auto & u:controls) {
      if (!std::isfinite(u[0]) || !std::isfinite(u[1])) {throw std::invalid_argument("invalid_trajectory_action");}
    }
    // The first planned control and published source must be the same action.
    if (std::abs(controls[0][0]-source.linear)>1e-7 || std::abs(controls[0][1]-source.angular)>1e-7) {
      throw std::invalid_argument("trajectory_source_mismatch");
    }
    source_=source;step_=step;controls_=controls;sequence_=0;scan_=source.scan;
    observation_time_=source.input;last_.reset();proofs_.clear();
  }
  void clear() {source_.reset();controls_.clear();last_.reset();proofs_.clear();sequence_=0;scan_=0;observation_time_=0.;}

  bool accept(const Json & j, double now, double input_timeout)
  {
    if (!source_ || !std::isfinite(now) || !std::isfinite(input_timeout) || input_timeout<=0.) {return false;}
    try {
      requireSchema(j,kTrajectoryProofSchema);
      const auto context=parseContext(j);
      if (!(context==source_->context) || requiredInt(j,"parent_request_id",1)!=source_->request ||
          requiredInt(j,"parent_cycle_id",1)!=source_->cycle ||
          requiredInt(j,"parent_scan_counter",1)!=source_->scan) {return false;}
      const auto sequence=requiredInt(j,"sequence",1), scan=requiredInt(j,"scan_counter",1);
      if (sequence<=sequence_ || scan<=scan_) {return false;}
      const double plan_start=requiredNumber(j,"plan_started_monotonic"),
        scan_time=requiredNumber(j,"scan_receive_monotonic"), pose_time=requiredNumber(j,"pose_receive_monotonic"),
        started=requiredNumber(j,"validation_started_monotonic"), done=requiredNumber(j,"validated_monotonic"),
        issued=requiredNumber(j,"published_monotonic");
      if (plan_start!=source_->published || scan_time<=plan_start || pose_time>started || scan_time>started ||
          started>done || done>issued || issued>now || now-scan_time>input_timeout ||
          now-pose_time>input_timeout || now-issued>input_timeout ||
          !j.at("valid").is_boolean()) {return false;}
      if (!j.at("valid").get<bool>()) {
        // A newer failed revalidation revokes an earlier proof immediately.
        sequence_=sequence;scan_=scan;observation_time_=scan_time;last_.reset();proofs_.clear();return true;
      }
      const double at_start=(started-plan_start)/step_, at_end=(done-plan_start)/step_,
        at_now=(now-plan_start)/step_;
      if (at_start<0. || at_end<0. || at_now<0. || at_start>=controls_.size() ||
          at_end>=controls_.size() || at_now>=controls_.size()) {return false;}
      const auto & expected=controls_[static_cast<std::size_t>(at_start)];
      const auto & command=j.at("command");
      const double linear=requiredNumber(command,"linear_x",false), angular=requiredNumber(command,"angular_z",false);
      if (std::abs(linear-expected[0])>1e-7 || std::abs(angular-expected[1])>1e-7) {return false;}
      sequence_=sequence;scan_=scan;observation_time_=scan_time;last_=j;proofs_.push_back(j);
      while (proofs_.size()>32) {proofs_.pop_front();}
      return true;
    } catch (const std::exception &) {return false;}
  }
  const std::optional<Json> & latest() const {return last_;}
  double observationTime() const {return observation_time_;}
  std::int64_t observationScan() const {return scan_;}
  std::optional<std::array<double,2>> action(double now, double input_timeout) const
  {return last_ ? actionFrom(*last_,now,input_timeout) : std::nullopt;}
  std::optional<std::array<double,2>> acknowledgedAction(
    double now, double input_timeout, std::int64_t acknowledged_scan) const
  {
    // Guard and Controller receive the same proof on different callbacks.
    // Reuse an earlier, still-fresh proof that Guard has actually acknowledged
    // while a newer proof is in flight; never extend its original timestamps.
    for (auto it=proofs_.rbegin();it!=proofs_.rend();++it) {
      if (it->at("scan_counter").get<std::int64_t>() <= acknowledged_scan) {
        const auto value=actionFrom(*it,now,input_timeout);
        if (value) {return value;}
      }
    }
    return std::nullopt;
  }
  bool commandMatches(double linear, double angular, double raw_time, double now, double timeout) const
  {
    if (!action(now, timeout) || !source_ || !std::isfinite(raw_time) || raw_time > now) {return false;}
    const double knot = (raw_time - source_->published) / step_;
    if (knot < 0. || knot >= controls_.size()) {return false;}
    const auto & expected = controls_[static_cast<std::size_t>(knot)];
    return std::abs(linear - expected[0]) <= 1e-7 && std::abs(angular - expected[1]) <= 1e-7;
  }
private:
  std::optional<std::array<double,2>> actionFrom(const Json & proof, double now, double input_timeout) const
  {
    if (!source_ || !std::isfinite(now) || !std::isfinite(input_timeout) || input_timeout<=0.) {
      return std::nullopt;
    }
    const double scan=proof.at("scan_receive_monotonic").get<double>();
    const double pose=proof.at("pose_receive_monotonic").get<double>();
    const double issued=proof.at("published_monotonic").get<double>();
    if (now<issued || now-scan>input_timeout || now-pose>input_timeout || now-issued>input_timeout) {
      return std::nullopt;
    }
    const double knot=(now-source_->published)/step_;
    if (knot<0. || knot>=controls_.size()) {return std::nullopt;}
    return controls_[static_cast<std::size_t>(knot)];
  }
  std::optional<Source> source_;
  double step_{0.}, observation_time_{0.};
  std::vector<std::array<double,2>> controls_;
  std::int64_t sequence_{0},scan_{0};
  std::optional<Json> last_;
  std::deque<Json> proofs_;
};

struct AuthorizationConfig
{
  double health_timeout{0.5};
  double source_timeout{0.5};
  double input_timeout{0.75};
  double authority_timeout{0.5};
  double recovery_timeout{5.0};
  // Opt-in until validator, controller and final guard are wired together.
  bool continuation_enabled{false};
};
struct AuthorizationDecision
{
  bool allowed{false};
  bool fault{false};
  std::string state{"DISARMED"};
  std::string reason{"navigation_not_armed"};
};

// No ROS, wall clock, callbacks or IO. Caller owns synchronization; all times
// are verified same-host CLOCK_MONOTONIC seconds. Restart means a new object
// and a fresh externally generated guard boot challenge, never a reset method.
class MotionAuthorization
{
public:
  explicit MotionAuthorization(std::string guard_boot, AuthorizationConfig config = {})
  : boot_(std::move(guard_boot)), config_(config)
  {
    if (boot_.empty()) {throw std::invalid_argument("missing_guard_boot");}
    for (const auto v : {config_.health_timeout, config_.source_timeout,
      config_.input_timeout, config_.authority_timeout, config_.recovery_timeout})
    {
      if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("invalid_authorization_timeout");}
    }
  }

  void health(const Json & j, double now)
  {
    expireExistingGrant(now);
    if (faulted()) {return;}
    try {
      requireSchema(j, kHealthSchema);
      const auto boot = requiredString(j, "boot_id");
      const auto generation = requiredInt(j, "worker_generation", 1);
      const auto sequence = requiredInt(j, "sequence", 1);
      const auto state = requiredString(j, "state");
      const auto timestamp = requiredNumber(j, "monotonic");
      if (timestamp > now || now - timestamp > config_.health_timeout) {
        protocolFailure("worker_health_stale"); return;
      }
      if (ever_prepared_ && (boot != health_boot_ || generation != health_generation_)) {
        latch("unauthorized_worker_generation"); return;
      }
      if (boot == health_boot_ && sequence <= health_sequence_) {
        if (sequence == health_sequence_ && j != health_payload_) {
          protocolFailure("worker_health_identity_conflict");
        }
        return;
      }
      if (state != "STARTING" && state != "RUNNING" && state != "FAULT_LATCHED" &&
        state != "SHUTTING_DOWN") {throw std::runtime_error("invalid_health_state");}
      if (state == "FAULT_LATCHED") {
        requiredString(j, "fault_id"); requiredString(j, "fault_code");
        failed_worker_boots_.insert(boot);
        if (ever_prepared_) {latch("worker_fault_latched"); return;}
      } else if (failed_worker_boots_.count(boot)) {return;}
      const auto planner_state = requiredString(j, "planner_state");
      if (planner_state != "ready" && planner_state != "retry" && planner_state != "exhausted") {
        throw std::runtime_error("invalid_planner_state");
      }
      requiredInt(j, "planner_failed_request_id", -1);
      health_payload_ = j;
      health_boot_ = boot;
      health_generation_ = generation;
      health_sequence_ = sequence;
      health_time_ = timestamp;  // receive time must not launder delayed health
      health_state_ = state;
      health_attempt_ = j.at("navigation_attempt_id").is_null() ? "" :
        requiredString(j, "navigation_attempt_id");
      if (ever_prepared_ && state == "SHUTTING_DOWN") {latch("worker_shutting_down");}
    } catch (const std::exception &) {protocolFailure("invalid_worker_health");}
  }

  void source(const Json & j, double now)
  {
    expireExistingGrant(now);
    if (faulted()) {return;}
    try {
      const auto candidate = parseSource(j, now);
      if (retired_.count(candidate.context.attempt)) {return;}
      if (grant_ && !(candidate.context == *grant_)) {
        if (candidate.context.attempt == grant_->attempt &&
          candidate.context.path == grant_->path && candidate.context.goal == grant_->goal &&
          (candidate.context.generation != grant_->generation ||
          candidate.context.worker_boot != grant_->worker_boot))
        {latch("unauthorized_worker_generation");}
        return;  // unrelated/old route/attempt does not poison the current grant
      }
      if (candidate.context.worker_boot != health_boot_ ||
        candidate.context.generation != health_generation_) {return;}
      if (handover_source_ && !(candidate.context == handover_source_->context) &&
        (candidate.request <= handover_source_->request ||
        candidate.cycle <= handover_source_->cycle || candidate.scan <= handover_source_->scan ||
        candidate.input < handover_source_->input || candidate.solved <= handover_source_->solved)) {
        return;  // an earlier computation cannot become the new route result
      }
      // A new route solve can also predate the newest observation already
      // used to validate the old route. Stage it until its own proof catches up.
      if (!source_ && handover_source_ && config_.continuation_enabled &&
          (candidate.scan < validated_scan_ || candidate.input < validated_input_)) {
        if (candidate.trajectory.empty()) {return;}
        if (pending_source_ && (candidate.request <= pending_source_->request ||
            candidate.cycle <= pending_source_->cycle || candidate.scan <= pending_source_->scan ||
            candidate.solved <= pending_source_->solved)) {return;}
        pending_source_ = candidate;
        pending_continuation_.install(candidate,candidate.trajectory_step,candidate.trajectory);
        return;
      }
      if (source_ && candidate.context == source_->context) {
        // A later revalidation has seen a newer scene. A solve computed from
        // an older scan must never overwrite it, especially after wall denial.
        if (config_.continuation_enabled && committed_ && !source_->trajectory.empty() &&
            (candidate.scan < validated_scan_ || candidate.input < validated_input_)) {
          // Keep the current certified action while the candidate is checked
          // against a scan at least as new as the last observed scene.
          if (candidate.trajectory.empty() || candidate.request <= source_->request ||
              candidate.cycle <= source_->cycle || candidate.solved <= source_->solved) {return;}
          if (pending_source_) {
            if (candidate.sameIdentity(*pending_source_)) {
              if (!candidate.sameContents(*pending_source_)) {latch("duplicate_command_mismatch");}
              return;
            }
            if (candidate.request <= pending_source_->request || candidate.cycle <= pending_source_->cycle ||
                candidate.scan <= pending_source_->scan || candidate.solved <= pending_source_->solved) {return;}
          }
          pending_source_=candidate;
          pending_continuation_.install(candidate,candidate.trajectory_step,candidate.trajectory);
          return;
        }
        if (candidate.sameIdentity(*source_)) {
          if (!candidate.sameContents(*source_)) {latch("duplicate_command_mismatch");}
          return;  // even exact duplicates do not refresh time or recovery
        }
        if (candidate.request <= source_->request || candidate.cycle <= source_->cycle ||
          candidate.scan <= source_->scan || candidate.solved <= source_->solved ||
          candidate.published <= source_->published)
        {protocolFailure("source_identity_regressed"); return;}
      }
      source_ = candidate;
      validated_scan_=candidate.scan;validated_input_=candidate.input;
      pending_source_.reset();pending_continuation_.clear();
      continuation_rejected_ = false;
      if (config_.continuation_enabled && !candidate.trajectory.empty()) {
        continuation_.install(candidate, candidate.trajectory_step, candidate.trajectory);
      } else {continuation_.clear();}
      if (handover_source_) {
        handover_new_sources_.push_back(candidate);
        while (handover_new_sources_.size() > 16) {handover_new_sources_.pop_front();}
      }
    } catch (const std::exception &) {protocolFailure("invalid_planner_source");}
  }

  void continuation(const Json & j, double now)
  {
    if (!config_.continuation_enabled || faulted() || !grant_) {return;}
    if (!committed_ && handover_source_ && handover_pending_source_ && j.is_object() &&
        j.contains("parent_request_id") && j["parent_request_id"].is_number_integer() &&
        j["parent_request_id"] == handover_pending_source_->request &&
        j.contains("scan_counter") && j["scan_counter"].is_number_integer() &&
        j["scan_counter"].get<std::int64_t>() >= validated_scan_ &&
        j.contains("scan_receive_monotonic") && j["scan_receive_monotonic"].is_number() &&
        j["scan_receive_monotonic"].get<double>() >= validated_input_ &&
        handover_pending_continuation_.accept(j,now,config_.input_timeout)) {
      validated_scan_=handover_pending_continuation_.observationScan();
      validated_input_=handover_pending_continuation_.observationTime();
      if (j.at("valid").get<bool>()) {
        // Guard may receive this proof just before Controller. Retain the
        // previously certified command only for its unextended validity.
        handover_previous_source_=handover_source_;
        handover_previous_continuation_=handover_continuation_;
        handover_source_=*handover_pending_source_;
        handover_continuation_=std::move(handover_pending_continuation_);
        handover_rejected_=false;
      } else {
        handover_continuation_.clear();handover_previous_source_.reset();
        handover_previous_continuation_.clear();handover_rejected_=true;
      }
      handover_pending_source_.reset();handover_pending_continuation_.clear();
      if (source_ && (source_->scan < validated_scan_ || source_->input < validated_input_)) {
        if (!source_->trajectory.empty()) {
          pending_source_=*source_;
          pending_continuation_.install(*source_,source_->trajectory_step,source_->trajectory);
        } else {pending_source_.reset();pending_continuation_.clear();}
        source_.reset();handover_new_sources_.clear();
      }
      return;
    }
    if (!committed_ && handover_source_ && j.is_object() &&
        j.contains("scan_counter") && j["scan_counter"].is_number_integer() &&
        j["scan_counter"].get<std::int64_t>() >= validated_scan_ &&
        j.contains("scan_receive_monotonic") && j["scan_receive_monotonic"].is_number() &&
        j["scan_receive_monotonic"].get<double>() >= validated_input_ &&
        handover_continuation_.accept(j,now,config_.input_timeout)) {
      validated_scan_ = handover_continuation_.observationScan();
      validated_input_ = handover_continuation_.observationTime();
      handover_rejected_ = !j.at("valid").get<bool>();
      if (handover_rejected_) {handover_previous_source_.reset();handover_previous_continuation_.clear();}
      // Delayed old-route proof may carry a newer scene than an already
      // received new-route solve. Require that solve to catch up as well.
      if (source_ && (source_->scan < validated_scan_ || source_->input < validated_input_)) {
        if (!source_->trajectory.empty()) {
          pending_source_ = *source_;
          pending_continuation_.install(*source_,source_->trajectory_step,source_->trajectory);
        } else {pending_source_.reset();pending_continuation_.clear();}
        source_.reset();handover_new_sources_.clear();
      }
      return;
    }
    if (!committed_ && !handover_source_) {return;}
    if (committed_ && (!source_ || !(source_->context == *grant_))) {return;}
    if (pending_source_ && j.is_object() && j.contains("parent_request_id") &&
        j["parent_request_id"].is_number_integer() && j["parent_request_id"]==pending_source_->request &&
        j.contains("scan_counter") && j["scan_counter"].is_number_integer() &&
        j["scan_counter"].get<std::int64_t>() >= validated_scan_ &&
        j.contains("scan_receive_monotonic") && j["scan_receive_monotonic"].is_number() &&
        j["scan_receive_monotonic"].get<double>() >= validated_input_ &&
        pending_continuation_.accept(j,now,config_.input_timeout)) {
      validated_scan_=pending_continuation_.observationScan();
      validated_input_=pending_continuation_.observationTime();
      if (j.at("valid").get<bool>()) {
        source_=*pending_source_;
        continuation_=std::move(pending_continuation_);
        continuation_rejected_=false;
        if (handover_source_) {
          handover_new_sources_.push_back(*source_);
          while (handover_new_sources_.size() > 16) {handover_new_sources_.pop_front();}
        }
      } else {
        // A newly seen wall invalidates both the new plan and old proof.
        continuation_.clear();continuation_rejected_=true;
        handover_continuation_.clear();handover_previous_source_.reset();
        handover_previous_continuation_.clear();handover_rejected_=true;
      }
      pending_source_.reset();pending_continuation_.clear();
      return;
    }
    if (committed_ && continuation_.accept(j,now,config_.input_timeout)) {
      validated_scan_=continuation_.observationScan();
      validated_input_=continuation_.observationTime();
      continuation_rejected_ = !j.at("valid").get<bool>();
    }
  }

  std::optional<std::array<double,2>> continuationAction(double now) const
  {
    if (!config_.continuation_enabled || continuation_rejected_ || !committed_ || !grant_ ||
        !source_ || !(source_->context == *grant_)) {return std::nullopt;}
    return continuation_.action(now,config_.input_timeout);
  }

  std::optional<std::array<double,2>> handoverAction(double now) const
  {
    if (!config_.continuation_enabled || handover_rejected_ || !handover_source_ || !grant_ ||
        committed_ || handover_source_->context.attempt != grant_->attempt) {return std::nullopt;}
    return handover_continuation_.action(now,config_.input_timeout);
  }
  bool handoverMatchesOldCommand(double linear, double angular, double raw_time, double now) const
  {
    if (!handoverFresh(now) || !handover_source_) {return false;}
    if (sourceFresh(*handover_source_,now) && linear == handover_source_->linear &&
        angular == handover_source_->angular) {return true;}
    if (handoverAction(now) && handover_continuation_.commandMatches(
        linear,angular,raw_time,now,config_.input_timeout)) {return true;}
    if (!handover_previous_source_) {return false;}
    if (sourceFresh(*handover_previous_source_,now) && linear == handover_previous_source_->linear &&
        angular == handover_previous_source_->angular) {return true;}
    return handover_previous_continuation_.commandMatches(
      linear,angular,raw_time,now,config_.input_timeout);
  }
  std::optional<std::array<double,2>> continuationAcknowledgedAction(double now, std::int64_t scan) const
  {
    if (!config_.continuation_enabled || continuation_rejected_ || !committed_ || !grant_ ||
        !source_ || !(source_->context == *grant_)) {return std::nullopt;}
    return continuation_.acknowledgedAction(now,config_.input_timeout,scan);
  }
  std::optional<std::array<double,2>> handoverAcknowledgedAction(double now, std::int64_t scan) const
  {
    if (!handoverAction(now)) {return std::nullopt;}
    return handover_continuation_.acknowledgedAction(now,config_.input_timeout,scan);
  }
  std::int64_t continuationScan(double now) const
  {return (continuationAction(now) || handoverAction(now)) ? validated_scan_ : 0;}
  std::int64_t continuationParent(double now) const
  {
    if (continuationAction(now) && source_) {return source_->request;}
    if (handoverAction(now) && handover_source_) {return handover_source_->request;}
    return 0;
  }

  void authorization(const Json & j, double now)
  {
    expireExistingGrant(now);
    if (faulted()) {return;}
    try {
      requireSchema(j, kGrantSchema);
      if (requiredString(j, "guard_boot_id") != boot_) {return;}
      const auto state = requiredString(j, "state");
      const auto context = parseContext(j);
      const auto id = requiredString(j, "grant_id");
      const auto sequence = requiredInt(j, "sequence", 1);
      const double timestamp = requiredNumber(j, "monotonic");
      if (timestamp > now) {return;}
      if (state == "REVOKE") {
        if (task_failed_attempt_ == context.attempt) {task_failed_attempt_.clear();}
        retired_.insert(context.attempt);  // revoke before a late PREPARE is permanent
        if (grant_ && grant_->attempt == context.attempt) {
          retired_grants_.insert(grant_id_);
          grant_.reset(); committed_ = false; grant_id_.clear(); source_stale_since_.reset();
          handover_source_.reset(); handover_continuation_.clear(); handover_pending_source_.reset(); handover_pending_continuation_.clear(); handover_previous_source_.reset(); handover_previous_continuation_.clear(); handover_rejected_=false; handover_new_sources_.clear();
          pending_source_.reset();pending_continuation_.clear();continuation_.clear();
        }
        return;
      }
      if (now - timestamp > config_.authority_timeout) {return;}
      if (state != "PREPARE" && state != "COMMIT") {throw std::runtime_error("invalid_grant_state");}
      if (retired_.count(context.attempt) || retired_grants_.count(id)) {return;}
      if (ever_prepared_ && (context.authority_boot != authority_boot_ ||
        context.controller_boot != controller_boot_)) {latch("authority_instance_changed"); return;}
      if (context.authority_boot == authority_boot_ && sequence <= authority_sequence_) {return;}
      if (!healthReady(now)) {
        // Keep the authority lease alive during a brief health gap. This
        // never grants motion or admits a new context while health is absent.
        if (grant_ && context == *grant_ && id == grant_id_) {
          authority_sequence_ = sequence;authority_time_ = timestamp;
        }
        return;
      }
      if (context.worker_boot != health_boot_ || context.generation != health_generation_ ||
          context.attempt != health_attempt_) {return;}
      if (state == "PREPARE") {
        if (!task_failed_attempt_.empty()) {return;}
        if (grant_ && !(context == *grant_)) {
          // A route revision keeps the same attempt/worker/boots and requires
          // a strictly new grant and fresh source. It cannot replace an action.
          if (context.attempt != grant_->attempt || context.worker_boot != grant_->worker_boot ||
            context.generation != grant_->generation || context.path <= grant_->path ||
            context.goal <= grant_->goal || id == grant_id_)
          {return;}
          // Same-attempt revision may temporarily keep the already-authorized
          // source, without changing or refreshing its original timestamps.
          if (committed_ && freshSource(now)) {
            handover_pending_source_.reset();handover_pending_continuation_.clear();
            handover_previous_source_.reset();handover_previous_continuation_.clear();
            handover_source_ = source_;
            handover_continuation_ = continuation_;
            handover_rejected_ = continuation_rejected_;
          } else if (!handoverFresh(now)) {
            handover_source_.reset();handover_continuation_.clear();handover_pending_source_.reset();handover_pending_continuation_.clear();handover_previous_source_.reset();handover_previous_continuation_.clear();handover_rejected_=false;
          }
          handover_new_sources_.clear();
          // The previous route may already have a newer solve awaiting its
          // first proof. Preserve that pending plan across the revision.
          if (pending_source_ && handover_source_ && pending_source_->context == handover_source_->context) {
            handover_pending_source_=pending_source_;
            handover_pending_continuation_=pending_continuation_;
          }
          pending_source_.reset();pending_continuation_.clear();
          retired_grants_.insert(grant_id_);
          committed_ = false;
          source_stale_since_.reset();
          source_.reset();
        } else if (grant_ && id != grant_id_) {return;}
        grant_ = context;
        grant_id_ = id;
        authority_boot_ = context.authority_boot;
        controller_boot_ = context.controller_boot;
        ever_prepared_ = true;
      } else {
        if (!grant_ || !(context == *grant_) || id != grant_id_) {return;}
        // Authority liveness is independent of solving. Missing source holds
        // zero, but must not turn a short same-generation interruption into
        // an authority timeout while its valid heartbeat continues.
        if (freshSource(now)) {committed_ = true;}
      }
      authority_sequence_ = sequence;
      authority_time_ = timestamp;
    } catch (const std::exception &) {protocolFailure("invalid_motion_grant");}
  }

  AuthorizationDecision evaluate(double now)
  {
    expireExistingGrant(now);
    if (faulted()) {return {false, true, "FAULT_LATCHED", fault_};}
    if (!task_failed_attempt_.empty()) {return {false, true, "TASK_FAILED", "planner_result_recovery_timeout"};}
    if (!grant_) {return {};}
    if (!healthReady(now)) {
      if (!source_stale_since_) {source_stale_since_ = now;}
      return {false, false, "WORKER_HEALTH_HOLD", "worker_health_stale"};
    }
    if (now < authority_time_ || now - authority_time_ > config_.authority_timeout) {
      latch("navigation_authority_stale"); return evaluate(now);
    }
    if (grant_->attempt != health_attempt_) {
      return {false, false, "DISARMED", "worker_attempt_not_ready"};
    }
    // During a recoverable solver retry retain only the already committed,
    // same-context source while its original input and result stamps remain
    // fresh. Health does not renew either timestamp. Exhaustion still stops.
    const auto planner_state = health_payload_.value("planner_state", "ready");
    const bool planning_failed = planner_state != "ready" ||
      (source_ && source_->request <= health_payload_.value("planner_failed_request_id", -1LL));
    if (planning_failed && planner_state != "exhausted" && committed_ && freshSource(now)) {
      return {true, false, "ARMED", "planner_retry_previous_fresh_source"};
    }
    if (!committed_ && !planning_failed && handoverFresh(now)) {
      return {true, false, "ROUTE_HANDOVER", "previous_source_still_valid"};
    }
    if (planning_failed || !freshSource(now)) {
      if (committed_ && !source_stale_since_) {source_stale_since_ = now;}
      return {false, false, "SOURCE_STALE_HOLD", "planner_result_stale"};
    }
    source_stale_since_.reset();
    if (!committed_) {return {false, false, "PREPARED", "awaiting_motion_commit"};}
    return {true, false, "ARMED", "motion_authorized"};
  }

  double healthAge(double now) const
  {return health_time_ > 0.0 && now >= health_time_ ? now - health_time_ : -1.0;}
  bool healthBridged(double now) const
  {return healthAge(now) > config_.health_timeout && healthReady(now);}
  bool faulted() const {return !fault_.empty();}
  const std::string & faultReason() const {return fault_;}
  const std::string & bootId() const {return boot_;}
  const std::string & grantId() const {return grant_id_;}
  bool committed() const {return committed_;}
  const std::optional<Context> & grantContext() const {return grant_;}
  std::string attemptId() const {return grant_ ? grant_->attempt : "";}
  const std::optional<Source> & latestSource() const {return source_;}
  const std::optional<Source> & handoverSource() const {return handover_source_;}
  bool handoverFresh(double now) const
  {
    return handover_source_ && !handover_rejected_ &&
      (sourceFresh(*handover_source_, now) || handoverAction(now).has_value()) &&
      health_payload_.value("planner_state", "ready") == "ready" &&
      handover_source_->request > health_payload_.value("planner_failed_request_id", -1LL);
  }
  bool handoverNewSourceFresh(double now) const
  {return !committed_ && source_ && grant_ && source_->context == *grant_ && sourceFresh(*source_,now);}
  std::optional<std::array<double,2>> handoverNewAction(double now) const
  {
    if (!config_.continuation_enabled || committed_ || continuation_rejected_ || !source_ || !grant_ ||
        !(source_->context == *grant_)) {return std::nullopt;}
    return continuation_.action(now,config_.input_timeout);
  }
  std::optional<std::array<double,2>> handoverNewAcknowledgedAction(double now, std::int64_t scan) const
  {
    if (!handoverNewAction(now)) {return std::nullopt;}
    return continuation_.acknowledgedAction(now,config_.input_timeout,scan);
  }
  std::int64_t handoverNewParent(double now) const
  {return handoverNewAction(now) && source_ ? source_->request : 0;}
  std::int64_t handoverNewScan(double now) const
  {return handoverNewAction(now) ? continuation_.observationScan() : 0;}
  bool handoverMatchesNewCommand(double linear, double angular, double raw_time, double now) const
  {
    if (!handoverFresh(now) || !grant_ || !std::isfinite(linear) || !std::isfinite(angular) ||
      !std::isfinite(raw_time) || raw_time > now) {return false;}
    for (auto it = handover_new_sources_.rbegin(); it != handover_new_sources_.rend(); ++it) {
      const bool original = sourceFresh(*it, now) && it->linear == linear && it->angular == angular;
      const bool certified = source_ && it->sameContents(*source_) && handoverNewAction(now) &&
        continuation_.commandMatches(linear,angular,raw_time,now,config_.input_timeout);
      if (it->context == *grant_ && (original || certified) && it->published <= raw_time &&
        it->request > handover_source_->request && it->cycle > handover_source_->cycle &&
        it->scan > handover_source_->scan && it->input >= handover_source_->input &&
        it->solved > handover_source_->solved &&
        it->request > health_payload_.value("planner_failed_request_id", -1LL)) {return true;}
    }
    return false;
  }
  void latch(const std::string & reason) {if (fault_.empty()) {fault_ = reason;}}

private:
  // Enforce the previous lease before consuming an arriving renewal. A delayed
  // timer callback must never let an expired execution become authorized again.
  void expireExistingGrant(double now)
  {
    if (faulted() || !grant_) {return;}
    if (now < authority_time_ || now - authority_time_ > config_.authority_timeout) {
      latch("navigation_authority_stale");
    } else if (source_stale_since_ && now - *source_stale_since_ > config_.recovery_timeout) {
      // Retire this attempt and its grant, not the healthy Worker. A late
      // result or grant cannot revive it; the action must explicitly revoke.
      task_failed_attempt_ = grant_->attempt;
      retired_.insert(grant_->attempt);
      retired_grants_.insert(grant_id_);
      grant_.reset(); source_.reset(); handover_source_.reset(); handover_continuation_.clear(); handover_pending_source_.reset(); handover_pending_continuation_.clear(); handover_previous_source_.reset(); handover_previous_continuation_.clear(); handover_rejected_=false; handover_new_sources_.clear(); source_stale_since_.reset();
      grant_id_.clear(); committed_ = false;
    }
  }
  bool healthReady(double now) const
  {
    if (health_state_ != "RUNNING" || now < health_time_) {return false;}
    const double age = now - health_time_;
    if (age <= config_.health_timeout) {return true;}
    // A freshly computed, strictly sequenced result from the same authorized
    // Worker is independent evidence that it is running. Bridge only a
    // bounded heartbeat gap; duplicates cannot refresh freshSource().
    return age <= config_.health_timeout + config_.source_timeout &&
      source_ && source_->context.worker_boot == health_boot_ &&
      source_->context.generation == health_generation_ && sourceFresh(*source_, now) &&
      health_payload_.value("planner_state", "ready") == "ready" &&
      source_->request > health_payload_.value("planner_failed_request_id", -1LL);
  }
  bool freshSource(double now) const
  {
    return source_ && grant_ && source_->context == *grant_ && !continuation_rejected_ &&
      (sourceFresh(*source_, now) || continuationAction(now).has_value() ||
       handoverNewAction(now).has_value());
  }
  bool sourceFresh(const Source & source, double now) const
  {
    return now >= source.published && now - source.published <= config_.source_timeout &&
           now >= source.solved && now - source.solved <= config_.source_timeout &&
           now >= source.input && now - source.input <= config_.input_timeout;
  }
  void protocolFailure(const std::string & reason)
  {
    if (grant_) {latch(reason);}
    else if (reason == "invalid_worker_health") {health_state_ = "INVALID";}
    else if (reason == "invalid_planner_source") {source_.reset();}
    // Idle garbage cannot create authorization or a permanent startup fault.
  }
  std::string boot_, fault_, health_boot_, health_state_, health_attempt_;
  std::string task_failed_attempt_;
  std::string authority_boot_, controller_boot_, grant_id_;
  AuthorizationConfig config_;
  Json health_payload_;
  std::int64_t health_generation_{-1}, health_sequence_{0}, authority_sequence_{0};
  double health_time_{0}, authority_time_{0};
  bool ever_prepared_{false}, committed_{false};
  std::optional<Context> grant_;
  std::optional<Source> source_;
  TrajectoryProofVerifier continuation_, pending_continuation_, handover_continuation_, handover_pending_continuation_, handover_previous_continuation_;
  std::optional<Source> pending_source_, handover_pending_source_, handover_previous_source_;
  std::int64_t validated_scan_{0};
  double validated_input_{0.};
  bool continuation_rejected_{false}, handover_rejected_{false};
  std::optional<Source> handover_source_;
  std::deque<Source> handover_new_sources_;
  std::optional<double> source_stale_since_;
  std::set<std::string> retired_, retired_grants_, failed_worker_boots_;
};
}  // namespace r1_neupan_runtime
