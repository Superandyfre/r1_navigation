// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace navigation_neupan_bridge
{
// A bin is anchored at scan start; each transform is requested at the
// midpoint. max_span bounds corrupted point timestamps and resource use.
struct SensorTimeBin
{
  std::uint32_t index;
  std::int64_t representative_ns;
};
inline std::optional<SensorTimeBin> sensorTimeBin(
  std::int64_t scan_start_ns, std::int64_t sample_ns,
  std::int64_t bin_width_ns, std::int64_t max_span_ns)
{
  if (scan_start_ns <= 0 || bin_width_ns <= 0 || max_span_ns < 0 ||
    sample_ns < scan_start_ns || sample_ns - scan_start_ns > max_span_ns) {return std::nullopt;}
  const auto index=(sample_ns-scan_start_ns)/bin_width_ns;
  if (index > std::numeric_limits<std::uint32_t>::max() ||
    scan_start_ns > std::numeric_limits<std::int64_t>::max() -
      index*bin_width_ns - bin_width_ns/2) {return std::nullopt;}
  return SensorTimeBin{static_cast<std::uint32_t>(index),
    scan_start_ns+index*bin_width_ns+bin_width_ns/2};
}
inline std::optional<std::int64_t> absoluteNanoseconds(double value)
{
  if (!std::isfinite(value) || value < 1.0 ||
    value >= static_cast<double>(std::numeric_limits<std::int64_t>::max())) {return std::nullopt;}
  return static_cast<std::int64_t>(value);
}
inline std::optional<std::int64_t> scanBeamTime(
  std::int64_t scan_start_ns, std::size_t index, double time_increment_sec)
{
  if (scan_start_ns <= 0 || !std::isfinite(time_increment_sec) || time_increment_sec < 0.) {
    return std::nullopt;
  }
  const long double offset=static_cast<long double>(index)*time_increment_sec*1000000000.0L;
  if (offset > std::numeric_limits<std::int64_t>::max()-scan_start_ns) {return std::nullopt;}
  return scan_start_ns+static_cast<std::int64_t>(offset);
}
}
