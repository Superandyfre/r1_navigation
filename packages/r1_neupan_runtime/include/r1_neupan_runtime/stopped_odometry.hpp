// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace r1_neupan_runtime
{
// Same-host steady receipt time and ROS source time are checked separately.
// This is arm evidence only; Guard's continuous odometry checks remain required.
class StoppedOdometry
{
public:
  void reset()
  {
    valid_ = false;
    first_receipt_ = last_receipt_ = 0.;
    first_stamp_ = 0;
    samples_ = 0;
    // Keep the accepted stamp high-water mark: reset cannot accept a replay.
  }

  void observe(
    std::int64_t stamp, std::int64_t ros_now, double received,
    const std::string & frame, const std::string & child,
    const std::string & expected_frame, const std::string & expected_child,
    const std::array<double, 6> & velocity)
  {
    if (!std::isfinite(received) || received <= 0. || !freshStamp(stamp, ros_now) ||
      expected_frame.empty() || expected_child.empty() || frame != expected_frame ||
      child != expected_child)
    {reset(); return;}
    for (std::size_t i = 0; i < velocity.size(); ++i) {
      if (!std::isfinite(velocity[i]) || std::abs(velocity[i]) > (i < 3 ? .01 : .02)) {
        reset(); return;
      }
    }
    // Repeated or reversed timestamps never refresh receipt age/observation.
    if (stamp <= last_stamp_) {return;}
    if (!valid_ || received <= last_receipt_ || received - last_receipt_ > kMaxAge ||
      stamp - last_stamp_ > kMaxAgeNs)
    {
      reset(); first_receipt_ = received; first_stamp_ = stamp;
    }
    last_stamp_ = stamp; last_receipt_ = received; valid_ = true; ++samples_;
  }

  bool stopped(double now, std::int64_t ros_now) const
  {
    return valid_ && samples_ >= 3 && std::isfinite(now) && now >= last_receipt_ &&
           now - last_receipt_ <= kMaxAge && freshStamp(last_stamp_, ros_now) &&
           now - first_receipt_ >= kObservation && last_stamp_ - first_stamp_ >= kObservationNs;
  }

private:
  static bool freshStamp(std::int64_t stamp, std::int64_t now)
  {
    // Zero ROS/simulation time and future stamps are not usable arm evidence.
    return stamp > 0 && now > 0 && stamp <= now && now - stamp <= kMaxAgeNs;
  }
  static constexpr double kMaxAge = .15, kObservation = .25;
  static constexpr std::int64_t kMaxAgeNs = 150000000, kObservationNs = 250000000;
  bool valid_{false};
  std::int64_t first_stamp_{0}, last_stamp_{0};
  double first_receipt_{0}, last_receipt_{0};
  std::size_t samples_{0};
};
}  // namespace r1_neupan_runtime
