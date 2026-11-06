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
  EXPECT_TRUE(s.geometryOnly());
  EXPECT_EQ(s.observe(true, true, "", false, at(500)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", true, at(3500)), Result::Healthy);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(3600)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(4600)), Result::Fault);
  EXPECT_EQ(s.observe(true, false, "localization_LOST:tracking", false, at(4601)), Result::Fault);
}
TEST(FootprintSyncGrace, AlternationDoesNotResetDeadline) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "costmap_footprint_not_synchronized", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", false, at(500)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "costmap_footprint_not_synchronized", false, at(1000)), Result::Fault);
}
TEST(FootprintSyncGrace, AlternatingTransientReasonsCannotRenewBudget) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "odom_stale", false, at(0)), Result::Hold);
  EXPECT_FALSE(s.geometryOnly());
  EXPECT_EQ(s.observe(true, false, "tf_stale", false, at(1000)), Result::Fault);
}
TEST(FootprintSyncGrace, LocalizationStatusUsesMonitorAbsoluteDeadline) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "localization_status_stale", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "localization_status_stale", false, at(1500)), Result::Hold);
}
TEST(FootprintSyncGrace, BootDoesNotUseTransientGrace) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(false, false, "costmap_footprint_not_synchronized", false, at(0)), Result::Fault);
}
