// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <cstdint>
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
inline constexpr char kHealthSchema[] = "r1_neupan_worker_health_v1";
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
  bool sameIdentity(const Source & b) const
  {
    return context == b.context && request == b.request && cycle == b.cycle && scan == b.scan;
  }
  bool sameContents(const Source & b) const
  {
    return sameIdentity(b) && input == b.input && dispatched == b.dispatched && solved == b.solved &&
           published == b.published && linear == b.linear && angular == b.angular;
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
  s.received = received;
  if (s.input > s.dispatched || s.dispatched > s.solved || s.solved > s.published ||
    s.published > received)
  {
    throw std::runtime_error("source_timestamp_order");
  }
  return s;
}
struct AuthorizationConfig
{
  double health_timeout{0.5};
  double source_timeout{0.5};
  double input_timeout{0.75};
  double authority_timeout{0.5};
  double recovery_timeout{5.0};
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
      if (source_ && candidate.context == source_->context) {
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
    } catch (const std::exception &) {protocolFailure("invalid_planner_source");}
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
        retired_.insert(context.attempt);  // revoke before a late PREPARE is permanent
        if (grant_ && grant_->attempt == context.attempt) {
          retired_grants_.insert(grant_id_);
          grant_.reset(); committed_ = false; grant_id_.clear(); source_stale_since_.reset();
        }
        return;
      }
      if (now - timestamp > config_.authority_timeout) {return;}
      if (state != "PREPARE" && state != "COMMIT") {throw std::runtime_error("invalid_grant_state");}
      if (retired_.count(context.attempt) || retired_grants_.count(id)) {return;}
      if (ever_prepared_ && (context.authority_boot != authority_boot_ ||
        context.controller_boot != controller_boot_)) {latch("authority_instance_changed"); return;}
      if (context.authority_boot == authority_boot_ && sequence <= authority_sequence_) {return;}
      if (!healthReady(now) || context.worker_boot != health_boot_ ||
        context.generation != health_generation_ || context.attempt != health_attempt_)
      {return;}
      if (state == "PREPARE") {
        if (grant_ && !(context == *grant_)) {
          // A route revision keeps the same attempt/worker/boots and requires
          // a strictly new grant and fresh source. It cannot replace an action.
          if (context.attempt != grant_->attempt || context.worker_boot != grant_->worker_boot ||
            context.generation != grant_->generation || context.path <= grant_->path ||
            context.goal <= grant_->goal || id == grant_id_)
          {return;}
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
    if (!grant_) {return {};}
    if (!healthReady(now)) {latch("worker_health_stale"); return evaluate(now);}
    if (now < authority_time_ || now - authority_time_ > config_.authority_timeout) {
      latch("navigation_authority_stale"); return evaluate(now);
    }
    if (grant_->attempt != health_attempt_) {
      return {false, false, "DISARMED", "worker_attempt_not_ready"};
    }
    if (source_stale_since_ && now - *source_stale_since_ > config_.recovery_timeout) {
      latch("planner_result_recovery_timeout"); return evaluate(now);
    }
    if (!freshSource(now)) {
      if (committed_ && !source_stale_since_) {source_stale_since_ = now;}
      return {false, false, "SOURCE_STALE_HOLD", "planner_result_stale"};
    }
    source_stale_since_.reset();
    if (!committed_) {return {false, false, "PREPARED", "awaiting_motion_commit"};}
    return {true, false, "ARMED", "motion_authorized"};
  }

  bool faulted() const {return !fault_.empty();}
  const std::string & faultReason() const {return fault_;}
  const std::string & bootId() const {return boot_;}
  const std::string & grantId() const {return grant_id_;}
  bool committed() const {return committed_;}
  const std::optional<Context> & grantContext() const {return grant_;}
  std::string attemptId() const {return grant_ ? grant_->attempt : "";}
  const std::optional<Source> & latestSource() const {return source_;}
  void latch(const std::string & reason) {if (fault_.empty()) {fault_ = reason;}}

private:
  // Enforce the previous lease before consuming an arriving renewal. A delayed
  // timer callback must never let an expired execution become authorized again.
  void expireExistingGrant(double now)
  {
    if (faulted() || !grant_) {return;}
    if (source_stale_since_ && now - *source_stale_since_ > config_.recovery_timeout) {
      latch("planner_result_recovery_timeout");
    } else if (!healthReady(now)) {
      latch("worker_health_stale");
    } else if (now < authority_time_ || now - authority_time_ > config_.authority_timeout) {
      latch("navigation_authority_stale");
    }
  }
  bool healthReady(double now) const
  {
    return health_state_ == "RUNNING" && now >= health_time_ &&
           now - health_time_ <= config_.health_timeout;
  }
  bool freshSource(double now) const
  {
    return source_ && grant_ && source_->context == *grant_ &&
           now >= source_->published && now - source_->published <= config_.source_timeout &&
           now >= source_->solved && now - source_->solved <= config_.source_timeout &&
           now >= source_->input && now - source_->input <= config_.input_timeout;
  }
  void protocolFailure(const std::string & reason)
  {
    if (grant_) {latch(reason);}
    else if (reason == "invalid_worker_health") {health_state_ = "INVALID";}
    else if (reason == "invalid_planner_source") {source_.reset();}
    // Idle garbage cannot create authorization or a permanent startup fault.
  }
  std::string boot_, fault_, health_boot_, health_state_, health_attempt_;
  std::string authority_boot_, controller_boot_, grant_id_;
  AuthorizationConfig config_;
  Json health_payload_;
  std::int64_t health_generation_{-1}, health_sequence_{0}, authority_sequence_{0};
  double health_time_{0}, authority_time_{0};
  bool ever_prepared_{false}, committed_{false};
  std::optional<Context> grant_;
  std::optional<Source> source_;
  std::optional<double> source_stale_since_;
  std::set<std::string> retired_, retired_grants_, failed_worker_boots_;
};
}  // namespace r1_neupan_runtime
