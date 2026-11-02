// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <optional>

namespace r1_neupan_runtime
{
// Detect physical nonresponse only when the actual final output is nonzero.
// Normal waiting, legal turning and stale odometry cannot become escape motion.
class StallMonitor
{
public:
  bool observe(double now, bool authorized_nonzero, double odom_stamp,
    double x, double y, double yaw)
  {
    if (!authorized_nonzero || !std::isfinite(now) || !std::isfinite(odom_stamp) ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      odom_stamp <= 0 || (last_odom_ && odom_stamp < *last_odom_))
    {reset(); return false;}
    if (last_odom_ && odom_stamp == *last_odom_) {return false;}
    last_odom_ = odom_stamp;
    const double angular = std::atan2(std::sin(yaw - yaw_), std::cos(yaw - yaw_));
    if (!since_ || now < *since_ || std::hypot(x - x_, y - y_) > .01 ||
      std::abs(angular) > .02)
    {since_ = now; x_ = x; y_ = y; yaw_ = yaw; return false;}
    return now - *since_ >= 3.;
  }
  void reset() {since_.reset(); last_odom_.reset();}
private:
  std::optional<double> since_, last_odom_;
  double x_{0}, y_{0}, yaw_{0};
};
}  // namespace r1_neupan_runtime
