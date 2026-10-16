#include <chrono>

#include <gtest/gtest.h>

#include "navigation_neupan_bridge/static_map_overlap_state.hpp"

namespace
{
using navigation_neupan_bridge::StaticMapOverlapState;
using navigation_neupan_bridge::StaticMapOverlapTransition;
using Clock = std::chrono::steady_clock;

Clock::time_point at(int seconds)
{
  return Clock::time_point{} + std::chrono::seconds(seconds);
}

TEST(StaticMapOverlapState, TransientOverlapDoesNotLatch)
{
  StaticMapOverlapState state(3.0);
  EXPECT_EQ(state.observe(true, true, true, at(0)), StaticMapOverlapTransition::None);
  EXPECT_EQ(state.observe(true, true, false, at(1)), StaticMapOverlapTransition::None);
  EXPECT_EQ(state.observe(true, true, true, at(2)), StaticMapOverlapTransition::None);
}

TEST(StaticMapOverlapState, RequiresFreshStableClearSamplesToRecover)
{
  StaticMapOverlapState state(3.0);
  state.observe(true, true, true, at(0));
  EXPECT_EQ(state.observe(true, true, true, at(1)), StaticMapOverlapTransition::Latched);
  EXPECT_EQ(state.observe(true, true, false, at(2)), StaticMapOverlapTransition::None);
  EXPECT_EQ(state.observe(true, false, false, at(6)), StaticMapOverlapTransition::None);
  EXPECT_EQ(state.observe(true, true, false, at(6)), StaticMapOverlapTransition::Recovered);
  EXPECT_EQ(state.observe(true, true, true, at(7)), StaticMapOverlapTransition::None);
  EXPECT_EQ(state.observe(true, true, true, at(8)), StaticMapOverlapTransition::Latched);
}

TEST(StaticMapOverlapState, OverlapOrUnhealthyInputRestartsRecoveryWindow)
{
  StaticMapOverlapState state(3.0);
  state.observe(true, true, true, at(0));
  state.observe(true, true, true, at(1));
  state.observe(true, true, false, at(2));
  state.observe(true, true, true, at(3));
  state.observe(true, true, false, at(4));
  EXPECT_EQ(state.observe(true, true, false, at(5)), StaticMapOverlapTransition::None);
  state.observe(false, false, false, at(6));
  state.observe(true, true, false, at(7));
  EXPECT_EQ(state.observe(true, true, false, at(9)), StaticMapOverlapTransition::None);
  EXPECT_EQ(state.observe(true, true, false, at(10)), StaticMapOverlapTransition::Recovered);
}
}  // namespace
