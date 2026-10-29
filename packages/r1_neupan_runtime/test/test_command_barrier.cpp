// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#include <limits>
#include "gtest/gtest.h"
#include "r1_neupan_runtime/command_barrier.hpp"

TEST(CommandBarrier, RequiresFreshZerosOnBothStreamsAfterReset)
{
  r1_neupan_runtime::CommandBarrier barrier;
  std::array<double, 6> zero{};
  EXPECT_FALSE(barrier.ready(100., .35, .3));
  barrier.observe(true, zero, 100.);
  EXPECT_FALSE(barrier.ready(100., .35, .3));
  barrier.observe(false, zero, 100.);
  EXPECT_TRUE(barrier.ready(100., .35, .3));
  EXPECT_FALSE(barrier.ready(100.31, .35, .3));
  EXPECT_FALSE(barrier.ready(99., .35, .3));
  barrier.reset(); EXPECT_FALSE(barrier.ready(100., .35, .3));
}
TEST(CommandBarrier, AnyNonzeroOrNonfiniteAxisClosesBarrier)
{
  for (bool raw : {true, false}) {
    for (std::size_t i = 0; i < 6; ++i) {
      r1_neupan_runtime::CommandBarrier barrier;
      std::array<double, 6> value{};
      barrier.observe(true, value, 100.); barrier.observe(false, value, 100.);
      value[i] = .01; barrier.observe(raw, value, 100.01);
      EXPECT_FALSE(barrier.ready(100.01, .35, .3));
      value[i] = std::numeric_limits<double>::quiet_NaN();
      barrier.observe(raw, value, 100.02);
      EXPECT_FALSE(barrier.ready(100.02, .35, .3));
    }
  }
}
