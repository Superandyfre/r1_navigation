#pragma once
#include <optional>
#include <string>
namespace navigation_neupan_bridge {
// These deadlines keep an action alive at zero velocity, never authorize motion.
// Permanent geometry, localization, nonfinite and frame faults remain fatal.
inline std::optional<double> guardSensorRecoveryTimeout(
  const std::string & reason, double scan_timeout)
{
  if (reason == "scan_stamp_stale_or_future") { return scan_timeout; }
  if (reason == "odom_stale" || reason == "odom_stamp_stale_or_future") {
    return 2.0;
  }
  return std::nullopt;
}
}
