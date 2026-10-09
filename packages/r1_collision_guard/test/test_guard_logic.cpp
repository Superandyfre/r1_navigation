// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "r1_collision_guard/guard_logic.hpp"
#include "r1_collision_guard/scan_filter.hpp"

namespace r1_collision_guard
{
namespace
{
GuardLogic makeGuard()
{
  GuardConfig config;
  config.reaction_delay_sec = 0.10;
  config.geometry_margin_m = 0.0;
  config.binary_search_iterations = 12;
  return GuardLogic(config, {{0.341, 0.266}, {0.341, -0.266}, {-0.341, -0.266}, {-0.341, 0.266}});
}

TEST(GuardLogic, SideWallOutsideFootprintPasses)
{
  auto guard = makeGuard();
  const auto decision = guard.evaluate(0.15, 0.0, 0.15, 0.0, {{0.40, 0.40}});
  EXPECT_EQ(decision.state, Decision::State::PASS);
}

TEST(GuardLogic, FrontalObstacleStops)
{
  auto guard = makeGuard();
  const auto decision = guard.evaluate(0.15, 0.0, 0.15, 0.0, {{0.37, 0.0}});
  EXPECT_NE(decision.state, Decision::State::PASS);
  EXPECT_GE(decision.linear_x, 0.0);
}

TEST(GuardLogic, ReverseIsRejectedWhenDisabled)
{
  auto guard = makeGuard();
  const auto decision = guard.evaluate(-0.01, 0.0, 0.0, 0.0, {});
  EXPECT_EQ(decision.state, Decision::State::FAULT);
  EXPECT_EQ(decision.linear_x, 0.0);
}

TEST(GuardLogic, ReversePassesWhenRearTrajectoryIsClear)
{
  GuardConfig config;
  config.allow_reverse = true;
  config.reaction_delay_sec = 0.10;
  config.geometry_margin_m = 0.0;
  auto guard = GuardLogic(
    config, {{0.341, 0.266}, {0.341, -0.266}, {-0.341, -0.266}, {-0.341, 0.266}});
  const auto decision = guard.evaluate(-0.10, 0.0, 0.0, 0.0, {{0.40, 0.0}});
  EXPECT_EQ(decision.state, Decision::State::PASS);
  EXPECT_LT(decision.linear_x, 0.0);
}

TEST(GuardLogic, RearObstacleLimitsReverse)
{
  GuardConfig config;
  config.allow_reverse = true;
  config.reaction_delay_sec = 0.10;
  config.geometry_margin_m = 0.0;
  auto guard = GuardLogic(
    config, {{0.341, 0.266}, {0.341, -0.266}, {-0.341, -0.266}, {-0.341, 0.266}});
  const auto decision = guard.evaluate(-0.15, 0.0, 0.0, 0.0, {{-0.37, 0.0}});
  EXPECT_EQ(decision.state, Decision::State::SLOW);
  EXPECT_LT(decision.linear_x, 0.0);
  EXPECT_GT(decision.linear_x, -0.15);
}

TEST(ScanFilter, RemovesOnlyStrictlyInteriorSelfReturns)
{
  const std::vector<Point2D> footprint{
    {0.80, 0.40}, {0.80, -0.40}, {-0.40, -0.40}, {-0.40, 0.40}};
  EXPECT_TRUE(strictlyInsidePolygon({0.52, -0.35}, footprint));
  EXPECT_FALSE(strictlyInsidePolygon({0.80, -0.35}, footprint));
  EXPECT_FALSE(strictlyInsidePolygon({0.81, -0.35}, footprint));
}
}  // namespace
}  // namespace r1_collision_guard
