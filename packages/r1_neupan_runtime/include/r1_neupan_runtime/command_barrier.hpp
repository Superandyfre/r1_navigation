// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cmath>

namespace r1_neupan_runtime
{
// A new grant must observe zero on BOTH ordered command streams before COMMIT.
// This drains the preceding smoother tail without identifying a Twist as a
// planner result. Requires a single trusted publisher per ordered stream.
class CommandBarrier
{
public:
  void reset() {raw_ = {}; smooth_ = {};}
  void observe(bool raw, const std::array<double, 6> & command, double now)
  {
    auto & state = raw ? raw_ : smooth_;
    state.zero = std::isfinite(now) && now > 0;
    for (double value : command) {state.zero = state.zero && std::isfinite(value) && value == 0.;}
    state.stamp = now;
  }
  bool ready(double now, double raw_timeout, double smooth_timeout) const
  {
    return fresh(raw_, now, raw_timeout) && fresh(smooth_, now, smooth_timeout);
  }

private:
  struct Sample {bool zero{false}; double stamp{0};};
  static bool fresh(const Sample & s, double now, double timeout)
  {
    return s.zero && std::isfinite(now) && now >= s.stamp && now - s.stamp <= timeout;
  }
  Sample raw_, smooth_;
};
}  // namespace r1_neupan_runtime
