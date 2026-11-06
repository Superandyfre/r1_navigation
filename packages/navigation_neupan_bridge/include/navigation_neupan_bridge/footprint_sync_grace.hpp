#pragma once
#include <chrono>
#include <optional>
#include <string>

namespace navigation_neupan_bridge
{
// This permits a stop-only grace period for otherwise fresh footprint mismatch.
// The deadline is not restarted by alternating good and bad samples.
class FootprintSyncGrace
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Result {Healthy, Hold, Fault};
  explicit FootprintSyncGrace(double timeout_seconds) : timeout_seconds_(timeout_seconds) {}

  Result observe(bool ever_ready, bool healthy, const std::string & reason,
    bool stabilized, Clock::time_point now)
  {
    if (!ever_ready) {reset(); return healthy ? Result::Healthy : Result::Fault;}
    if (!healthy && reason != "costmap_footprint_not_synchronized") {
      reset(); return Result::Fault;
    }
    if (!healthy && !started_) {started_ = now;}
    if (!started_) {return Result::Healthy;}
    if (healthy && stabilized) {reset(); return Result::Healthy;}
    if (!healthy && std::chrono::duration<double>(now - *started_).count() >= timeout_seconds_) {
      return Result::Fault;
    }
    return Result::Hold;
  }
  void reset() {started_.reset();}
private:
  double timeout_seconds_;
  std::optional<Clock::time_point> started_;
};
}
