// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace r1_neupan_runtime
{
// Optional synchronous extension to nav2_core::Controller. Only the real
// ControllerServer action loop may supply this liveness proof. Implementations
// serialize these calls with health callbacks and reject retired identities.
class ExecutionObserver
{
public:
  virtual ~ExecutionObserver() = default;
  virtual void beginExecution(const std::string & authority, const std::string & attempt) = 0;
  virtual void checkExecution(const std::string & authority, const std::string & attempt) = 0;
  virtual void endExecution(
    const std::string & authority, const std::string & attempt,
    const std::string & reason) noexcept = 0;
};

// Exact-identity, at-most-once cleanup, including exceptions during begin or
// path installation. No locks or ROS callbacks held by this adapter.
class ExecutionSession
{
public:
  ExecutionSession(ExecutionObserver * observer, std::string authority, std::string attempt)
  : observer_(observer), authority_(std::move(authority)), attempt_(std::move(attempt))
  {
    if (!observer_ || authority_.empty() || attempt_.empty()) {
      throw std::invalid_argument("invalid_execution_identity");
    }
    try {observer_->beginExecution(authority_, attempt_);} catch (...) {end("begin_failed"); throw;}
  }
  ~ExecutionSession() {end("execution_scope_exited");}
  ExecutionSession(const ExecutionSession &) = delete;
  ExecutionSession & operator=(const ExecutionSession &) = delete;
  void check()
  {
    if (!observer_) {throw std::runtime_error("execution_already_ended");}
    observer_->checkExecution(authority_, attempt_);
  }
  void end(const std::string & reason) noexcept
  {
    auto * observer = observer_;
    observer_ = nullptr;
    if (observer) {observer->endExecution(authority_, attempt_, reason);}
  }

private:
  ExecutionObserver * observer_;
  std::string authority_, attempt_;
};
}  // namespace r1_neupan_runtime
