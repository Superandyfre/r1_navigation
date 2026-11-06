#include <chrono>
#include <gtest/gtest.h>
#include "navigation_neupan_bridge/footprint_sync_grace.hpp"
using navigation_neupan_bridge::FootprintSyncGrace;
using Result = FootprintSyncGrace::Result;
using Clock = FootprintSyncGrace::Clock;
static Clock::time_point at(int ms) {return Clock::time_point{} + std::chrono::milliseconds(ms);}
TEST(FootprintSyncGrace, TransientDropoutWaitsUntilStableAndOtherFaultsAreImmediate) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", false, at(500)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", true, at(3500)), Result::Healthy);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(3600)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(4600)), Result::Fault);
  EXPECT_EQ(s.observe(true, false, "localization_LOST:tracking", false, at(4601)), Result::Fault);
}
TEST(FootprintSyncGrace, AlternationDoesNotResetDeadline) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, true, "", false, at(500)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "dynamic_footprint_unavailable_or_stale", false, at(1000)), Result::Fault);
}
TEST(FootprintSyncGrace, AlternatingTransientReasonsCannotRenewBudget) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "odom_stale", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "tf_stale", false, at(1000)), Result::Fault);
}
TEST(FootprintSyncGrace, LocalizationStatusUsesMonitorAbsoluteDeadline) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(true, false, "localization_status_stale", false, at(0)), Result::Hold);
  EXPECT_EQ(s.observe(true, false, "localization_status_stale", false, at(1500)), Result::Hold);
}
TEST(FootprintSyncGrace, BootDoesNotUseTransientGrace) {
  FootprintSyncGrace s(1.0);
  EXPECT_EQ(s.observe(false, false, "dynamic_footprint_unavailable_or_stale", false, at(0)), Result::Fault);
}

TEST(FootprintSyncGrace, OnlyAvailabilityFaultsCanAutomaticallyRecover) {
  EXPECT_FALSE(FootprintSyncGrace::recoverableFault("costmap_footprint_unavailable_or_stale"));
  EXPECT_TRUE(FootprintSyncGrace::recoverableFault("dynamic_footprint_unavailable_or_stale"));
  EXPECT_TRUE(FootprintSyncGrace::recoverableFault("localization_status_stale"));
  EXPECT_FALSE(FootprintSyncGrace::recoverableFault("localization_jump_hard"));
  EXPECT_FALSE(FootprintSyncGrace::recoverableFault("localization_jump_repeated"));
  EXPECT_FALSE(FootprintSyncGrace::recoverableFault("localization_LOST:tracking"));
  EXPECT_FALSE(FootprintSyncGrace::recoverableFault("start_in_lethal_static_map"));
}
