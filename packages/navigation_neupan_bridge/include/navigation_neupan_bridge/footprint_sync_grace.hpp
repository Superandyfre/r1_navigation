#pragma once
#include <algorithm>
#include <chrono>
#include <optional>
#include <string>

namespace navigation_neupan_bridge
{
// Transient localization input gaps may recover; permanent errors fault.
// A single deadline covers changing or alternating transient reasons.
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
    if (!healthy && !transient(reason)) {reset(); return Result::Fault;}
    if (!healthy && !started_) {started_ = now;}
    if (!started_) {return Result::Healthy;}
    if (healthy && stabilized) {reset(); return Result::Healthy;}
    if (!healthy && reason != "localization_status_stale" &&
      std::chrono::duration<double>(now - *started_).count() >= timeout_seconds_) {
      return Result::Fault;
    }
    // The localization status stream has its own absolute stale timeout in the
    // monitor; HOLD persists until that stricter source-specific limit expires.
    return Result::Hold;
  }
  // These failures report availability, not a proven pose jump or a genuine
  // localization LOST report. They can recover after all sources stabilize.
  static bool recoverableFault(const std::string & reason) {return transient(reason);}
  void reset() {started_.reset();}
private:
  static bool transient(const std::string & reason)
  {
    return reason == "dynamic_footprint_unavailable_or_stale" ||
      reason == "global_costmap_unavailable_or_stale" ||
      reason == "map_to_base_unavailable" || reason == "tf_stale" ||
      reason == "odom_stale" || reason == "localization_status_stale";
  }
  double timeout_seconds_;
  std::optional<Clock::time_point> started_;
};
}
