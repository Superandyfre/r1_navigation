// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "r1_neupan_runtime/motion_authorization.hpp"

namespace r1_neupan_runtime
{
// No ROS/IO/locks. The plugin serializes every call with its output snapshot.
// Local MotionAuthorization uses its own challenge: it is a second monitor,
// never a substitute for a matching ACK from the real final Guard.
class ControllerExecution
{
public:
  explicit ControllerExecution(
    std::string boot = newBootId(), AuthorizationConfig config = {},
    double startup_timeout = 40.)
  : boot_(std::move(boot)), config_(config), startup_timeout_(startup_timeout),
    monitor_(boot_, config)
  {
    if (!std::isfinite(startup_timeout_) || startup_timeout_ <= 0) {
      throw std::invalid_argument("invalid_execution_startup_timeout");
    }
  }

  void guard(const Json & j, double now)
  {
    try {
      requireSchema(j, "r1_collision_guard_status_v1");
      if (requiredString(j, "authorization_protocol") != kGrantSchema ||
        requiredString(j, "source_protocol") != kSourceSchema)
      {throw std::runtime_error("guard_protocol_mismatch");}
      const auto boot = requiredString(j, "guard_boot_id");
      const auto sequence = requiredInt(j, "sequence", 1);
      const auto stamp = requiredNumber(j, "monotonic");
      if (stamp > now || now - stamp > config_.authority_timeout) {return;}
      if (!guard_boot_.empty() && boot != guard_boot_) {
        if (armed_) {fail("guard_instance_changed"); return;}
        guard_sequence_ = 0;
      }
      if (boot == guard_boot_ && sequence <= guard_sequence_) {return;}
      const auto state = requiredString(j, "authorization_state");
      requiredString(j, "physical_safety_state");
      const auto linear = requiredNumber(j, "output_linear_x", false);
      const auto angular = requiredNumber(j, "output_angular_z", false);
      if (state == "FAULT_LATCHED") {fail("guard_fault_latched");}
      guard_boot_ = boot;
      guard_sequence_ = sequence;
      guard_time_ = stamp;
      guard_payload_ = j;
      guard_zero_ = linear == 0.0 && angular == 0.0;
    } catch (const std::exception &) {
      guard_time_ = 0;
      if (active_) {fail("invalid_guard_status");}
    }
  }

  void health(const Json & j, double now)
  {
    if (faulted()) {return;}
    try {
      requireSchema(j, kHealthSchema);
      const auto boot = requiredString(j, "boot_id");
      const auto sequence = requiredInt(j, "sequence", 1);
      const auto stamp = requiredNumber(j, "monotonic");
      const auto state = requiredString(j, "state");
      const auto generation = requiredInt(j, "worker_generation", -1);
      if (stamp > now || now - stamp > config_.health_timeout) {return;}
      if (boot == health_boot_ && sequence <= health_sequence_) {
        if (sequence == health_sequence_ && j != health_payload_) {
          fail("worker_health_identity_conflict");
        }
        return;
      }
      if (!worker_boot_.empty() && (boot != worker_boot_ || generation != worker_generation_)) {
        fail("unauthorized_worker_generation"); return;
      }
      if (state == "FAULT_LATCHED") {
        requiredString(j, "fault_id"); requiredString(j, "fault_code");
        fail("worker_fault_latched"); return;
      }
      if (state != "RUNNING" && state != "STARTING" && state != "SHUTTING_DOWN") {
        throw std::runtime_error("invalid_worker_state");
      }
      if (active_ && state == "SHUTTING_DOWN") {fail("worker_shutting_down"); return;}
      // The first STARTING heartbeat may precede a lazy Worker's generation.
      if (state == "RUNNING" && generation < 1) {
        throw std::runtime_error("invalid_worker_generation");
      }
      health_boot_ = boot; health_sequence_ = sequence; health_time_ = stamp; health_payload_ = j;
      if (state == "RUNNING" && active_ && j.at("navigation_attempt_id") == attempt_) {
        if (worker_boot_.empty()) {worker_boot_ = boot; worker_generation_ = generation;}
      }
      monitor_.health(j, now);
      syncFault();
    } catch (const std::exception &) {if (active_) {fail("invalid_worker_health");}}
  }

  bool arm(double now, bool stopped_odom)
  {
    if (faulted() || active_ || !stopped_odom || guard_boot_.empty() ||
      !freshGuard(now) || !guard_zero_ ||
      guard_payload_.value("authorization_state", "") != "DISARMED")
    {return false;}
    armed_ = true;
    return true;
  }

  void begin(const std::string & authority, const std::string & attempt, double now)
  {
    checkFault();
    if (!armed_ || active_ || authority.empty() || attempt.empty() || retired_.count(attempt) ||
      !freshGuard(now)) {throw std::runtime_error("execution_not_admitted");}
    if (!authority_.empty() && authority != authority_) {
      fail("authority_instance_changed"); checkFault();
    }
    authority_ = authority; attempt_ = attempt; active_ = true;
    started_ = now; last_check_ = now; path_ = -1; goal_ = -1; route_wait_since_.reset();
    has_plan_ = false; prepared_ = false; committed_ = false; first_source_ = false;
  }

  void plan(std::int64_t path, std::int64_t goal, double now)
  {
    checkFault();
    if (!active_ || path < 0 || goal < 0 || (has_plan_ && (path <= path_ || goal <= goal_))) {
      throw std::runtime_error("invalid_execution_plan");
    }
    if (has_plan_) {
      if (!route_wait_since_) {route_wait_since_ = now;}
    } else {started_ = now;}
    path_ = path; goal_ = goal; has_plan_ = true;
    prepared_ = false; committed_ = false;
    grant_id_ = newBootId();
  }

  bool source(const Json & j, double now)
  {
    if (!active_ || !has_plan_ || faulted()) {return false;}
    try {
      const auto s = parseSource(j, now);
      if (s.context.authority_boot != authority_ || s.context.controller_boot != boot_ ||
        s.context.attempt != attempt_ || s.context.path != path_ || s.context.goal != goal_)
      {return false;}
      if (!worker_boot_.empty() && (s.context.worker_boot != worker_boot_ ||
        s.context.generation != worker_generation_))
      {
        fail("unauthorized_worker_generation"); return false;
      }
      monitor_.source(j, now); syncFault();
      const auto & accepted = monitor_.latestSource();
      if (faulted() || !accepted || !accepted->sameContents(s)) {return false;}
      // A duplicate is accepted for observation, never given a new timestamp.
      const bool fresh = freshSource(*accepted, now);
      first_source_ = first_source_ || fresh;
      if (fresh) {route_wait_since_.reset();}
      return true;
    } catch (const std::exception &) {fail("invalid_planner_source"); return false;}
  }

  void check(const std::string & authority, const std::string & attempt, double now)
  {
    checkFault();
    if (!active_ || authority != authority_ || attempt != attempt_) {
      throw std::runtime_error("execution_identity_not_current");
    }
    watchdog(now);
    checkFault();
    last_check_ = now;  // only the synchronous Action loop calls check()
  }

  std::optional<Json> tick(double now)
  {
    if (!active_) {return std::nullopt;}
    watchdog(now);
    if (faulted() || !has_plan_ || worker_boot_.empty() || !freshGuard(now)) {return std::nullopt;}
    // The current Worker/Attempt binding is enough to invalidate an old route.
    // Waiting for new route health here leaves the previous external grant live.
    if (!prepared_ && (health_payload_.value("state", "") != "RUNNING" ||
      health_payload_.value("navigation_attempt_id", Json()) != attempt_)) {return std::nullopt;}
    const bool matched = matchingGuardContext();
    const auto & current_source = monitor_.latestSource();
    if (prepared_ && matched && current_source && freshSource(*current_source, now) &&
      guard_payload_.value("command_barrier_ready", false) &&
      guard_payload_.value("authorization_state", "") == "PREPARED" &&
      (guard_payload_.value("physical_safety_state", "") == "PASS" ||
      guard_payload_.value("physical_safety_state", "") == "SLOW")) {committed_ = true;}
    auto grant = grantMessage(committed_ ? "COMMIT" : "PREPARE", now);
    auto local = grant; local["guard_boot_id"] = boot_;
    monitor_.authorization(local, now);
    prepared_ = monitor_.grantId() == grant_id_;
    syncFault();
    if (faulted()) {return std::nullopt;}
    return grant;
  }

  bool permitted(double now)
  {
    watchdog(now);
    const auto decision = monitor_.evaluate(now); syncFault();
    return active_ && !faulted() && decision.allowed && freshGuard(now) &&
           guard_payload_.value("authorization_state", "") == "ARMED" &&
           matchingGuardContext();
  }

  std::optional<Json> end(double now)
  {
    if (!active_) {return std::nullopt;}
    std::optional<Json> revoke;
    if (has_plan_ && !worker_boot_.empty() && !grant_id_.empty()) {
      revoke = grantMessage("REVOKE", now);
      auto local = *revoke; local["guard_boot_id"] = boot_;
      monitor_.authorization(local, now);
    }
    retired_.insert(attempt_); active_ = false; has_plan_ = false;
    prepared_ = false; committed_ = false;
    return revoke;
  }
  void fail(const std::string & reason)
  {
    if (fault_.empty()) {
      fault_ = reason; armed_ = false; monitor_.latch(reason);
    }
  }
  bool retryAllowed(double now)
  {
    // Readiness only: never arm, clear a fault, or allocate an execution/grant.
    syncFault();
    return !active_ && armed_ && !faulted() && freshGuard(now) &&
           health_time_ > 0 && now >= health_time_ && now - health_time_ <= config_.health_timeout &&
           health_payload_.value("state", "") == "RUNNING" && !worker_boot_.empty();
  }
  void disarm() {armed_ = false;}
  bool faulted() const {return !fault_.empty();}
  bool active() const {return active_;}
  const std::string & fault() const {return fault_;}
  const std::string & boot() const {return boot_;}
  const std::string & authority() const {return authority_;}
  const std::string & attempt() const {return attempt_;}
  const std::string & grantId() const {return grant_id_;}
  std::int64_t generation() const {return worker_generation_;}

private:
  bool matchingGuardContext() const
  {
    try {
      const Context expected{authority_, boot_, attempt_, worker_boot_, worker_generation_, path_,
        goal_};
      return requiredString(guard_payload_, "grant_id") == grant_id_ &&
             parseContext(guard_payload_) == expected;
    } catch (const std::exception &) {return false;}
  }
  void syncFault() {if (monitor_.faulted()) {fail(monitor_.faultReason());}}
  void checkFault() const {if (faulted()) {throw std::runtime_error(fault_);}}
  bool freshGuard(double now) const
  {
    return guard_time_ > 0 && now >= guard_time_ && now - guard_time_ <= config_.authority_timeout;
  }
  bool freshSource(const Source & s, double now) const
  {
    return now >= s.published && now - s.published <= config_.source_timeout &&
           now >= s.solved && now - s.solved <= config_.source_timeout &&
           now >= s.input && now - s.input <= config_.input_timeout;
  }
  void watchdog(double now)
  {
    if (!active_ || faulted()) {return;}
    if (now < last_check_ || now - last_check_ > config_.authority_timeout) {
      fail("action_loop_stale");
    } else if (!freshGuard(now)) {
      fail("guard_status_stale");
    } else if (now - started_ > config_.health_timeout &&
      (health_time_ <= 0 || now - health_time_ > config_.health_timeout))
    {
      fail("worker_health_stale");
    } else if (route_wait_since_ && now - *route_wait_since_ > config_.recovery_timeout) {
      fail("route_source_timeout");
    } else if (!first_source_ && now - started_ > startup_timeout_) {
      fail("initial_source_timeout");
    }
    monitor_.evaluate(now); syncFault();
  }
  Json grantMessage(const char * state, double now)
  {
    return {{"schema", kGrantSchema}, {"state", state}, {"guard_boot_id", guard_boot_},
      {"grant_id", grant_id_}, {"sequence", ++grant_sequence_}, {"monotonic", now},
      {"authority_boot_id", authority_}, {"controller_boot_id", boot_},
      {"navigation_attempt_id", attempt_}, {"worker_boot_id", worker_boot_},
      {"worker_generation", worker_generation_}, {"path_version", path_}, {"goal_version", goal_}};
  }
  std::string boot_, authority_, attempt_, fault_, guard_boot_, health_boot_, worker_boot_,
    grant_id_;
  AuthorizationConfig config_;
  std::optional<double> route_wait_since_;
  double startup_timeout_, guard_time_{0}, health_time_{0}, started_{0}, last_check_{0};
  MotionAuthorization monitor_;
  Json health_payload_, guard_payload_;
  std::int64_t guard_sequence_{0}, health_sequence_{0}, grant_sequence_{0};
  std::int64_t worker_generation_{-1}, path_{-1}, goal_{-1};
  bool armed_{false}, active_{false}, has_plan_{false}, prepared_{false}, committed_{false},
  first_source_{false};
  std::set<std::string> retired_;
  bool guard_zero_{false};
};
}  // namespace r1_neupan_runtime
