#pragma once
#include <algorithm>
#include <chrono>
#include <optional>
#include <string>

namespace navigation_neupan_bridge
{
// Transient input gaps hold zero; permanent/unknown localization errors fault.
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
    if (!healthy && !started_) {
      started_ = now;
      geometry_only_ = reason == "costmap_footprint_not_synchronized";
    } else if (!healthy && reason != "costmap_footprint_not_synchronized") {
      geometry_only_ = false;
    }
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
  bool geometryOnly() const {return started_.has_value() && geometry_only_;}
  void reset() {started_.reset(); geometry_only_ = false;}
private:
  static bool transient(const std::string & reason)
  {
    return reason == "costmap_footprint_not_synchronized" ||
      reason == "costmap_footprint_unavailable_or_stale" ||
      reason == "dynamic_footprint_unavailable_or_stale" ||
      reason == "costmap_footprint_transform_unavailable" ||
      reason == "global_costmap_unavailable_or_stale" ||
      reason == "map_to_base_unavailable" || reason == "tf_stale" ||
      reason == "odom_stale" || reason == "localization_status_stale";
  }
  double timeout_seconds_;
  std::optional<Clock::time_point> started_;
  bool geometry_only_{false};
};
}
