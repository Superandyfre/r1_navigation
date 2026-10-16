// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "r1_collision_guard/guard_logic.hpp"
#include "r1_collision_guard/mesh_components.hpp"
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

TEST(GuardLogic, ExactArmComponentsLeaveTheSpaceBetweenArmsTraversable)
{
  GuardConfig config;
  config.geometry_margin_m = 0.02;
  const std::vector<Point2D> base{
    {0.341, 0.266}, {0.341, -0.266}, {-0.341, -0.266}, {-0.341, 0.266}};
  const std::vector<Point2D> convex_hull{
    {-0.341, -0.266}, {0.634, -0.38}, {0.634, 0.25},
    {0.397, 0.437}, {0.023, 0.438}, {-0.341, 0.266}};
  GuardLogic guard(config, convex_hull);
  const Point2D gap_point{0.3475, 0.4448};
  EXPECT_EQ(guard.evaluate(0.05, 0.0, 0.0, 0.0, {gap_point}).state, Decision::State::STOP);
  guard.setFootprints({
    base,
    {{0.45, 0.30}, {0.63, 0.30}, {0.63, 0.40}, {0.45, 0.40}},
    {{-0.05, 0.30}, {0.05, 0.30}, {0.05, 0.40}, {-0.05, 0.40}},
  });
  EXPECT_EQ(guard.evaluate(0.05, 0.0, 0.0, 0.0, {gap_point}).state, Decision::State::PASS);
  EXPECT_EQ(guard.evaluate(0.05, 0.0, 0.0, 0.0, {{0.49, 0.41}}).state, Decision::State::STOP);
  EXPECT_EQ(guard.evaluate(0.05, 0.0, 0.0, 0.0, {{0.25, 0.0}}).state, Decision::State::STOP);
}

TEST(GuardLogic, MeshComponentDecoderRejectsMalformedData)
{
  const auto components = decodeMeshComponents({3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0});
  ASSERT_EQ(components.size(), 1U);
  EXPECT_EQ(components[0].size(), 3U);
  EXPECT_THROW(decodeMeshComponents({3.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(decodeMeshComponents({2.5, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(decodeMeshComponents({3.0, 0.0, 0.0, 1.0, 0.0, NAN, 1.0}), std::invalid_argument);
}

TEST(GuardLogic, RawMeshFilteringRetainsTheProtectedToleranceBand)
{
  const auto pair = decodeGuardMeshComponents({
    1.0,
    4.0, 0.40, 0.30, 0.50, 0.30, 0.50, 0.40, 0.40, 0.40,
    4.0, 0.34, 0.24, 0.56, 0.24, 0.56, 0.46, 0.34, 0.46,
  }, 1U);
  const Point2D near_arm{0.53, 0.35};
  EXPECT_FALSE(strictlyInsidePolygon(near_arm, pair.raw[0]));
  auto guard = makeGuard();
  guard.setFootprints(pair.expanded);
  EXPECT_EQ(guard.evaluate(0.05, 0.0, 0.0, 0.0, {near_arm}).state, Decision::State::STOP);
  EXPECT_THROW(decodeGuardMeshComponents({1.0, 3.0, 0.0, 0.0}, 1U), std::invalid_argument);
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
