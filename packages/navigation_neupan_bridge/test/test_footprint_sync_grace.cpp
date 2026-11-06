#include <chrono>
#include <gtest/gtest.h>
#include "navigation_neupan_bridge/footprint_sync_grace.hpp"
using navigation_neupan_bridge::FootprintSyncGrace;
using Result = FootprintSyncGrace::Result;
using Clock = FootprintSyncGrace::Clock;
static Clock::time_point at(int ms) {return Clock::time_point{} + std::chrono::milliseconds(ms);}
TEST(FootprintSyncGrace, MismatchStopsUntilStableAndOtherFaultsAreImmediate) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "costmap_footprint_not_synchronized", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", false, at(500)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", true, at(3500)), Result::Healthy);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(3600)), Result::Fault);
}
TEST(FootprintSyncGrace, AlternationDoesNotResetDeadline) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "costmap_footprint_not_synchronized", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", false, at(500)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "costmap_footprint_not_synchronized", false, at(1000)), Result::Fault);
}
TEST(FootprintSyncGrace, BootDoesNotUseTransientGrace) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(false, false, "costmap_footprint_not_synchronized", false, at(0)), Result::Fault);
}
