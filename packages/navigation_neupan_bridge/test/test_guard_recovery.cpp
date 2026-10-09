#include <gtest/gtest.h>
#include "navigation_neupan_bridge/guard_recovery.hpp"
using namespace navigation_neupan_bridge;
TEST(GuardRecovery, OnlyTransientSensorFaultsGetBoundedZeroHold) {
  EXPECT_DOUBLE_EQ(*guardSensorRecoveryTimeout("odom_stale",300.0),2.0);
  EXPECT_DOUBLE_EQ(*guardSensorRecoveryTimeout("odom_stamp_stale_or_future",300.0),2.0);
  EXPECT_DOUBLE_EQ(*guardSensorRecoveryTimeout("scan_stamp_stale_or_future",300.0),300.0);
  for (const auto * reason : {"odom_frame_mismatch", "odom_child_frame_mismatch",
    "nonfinite_input", "dynamic_footprint_stale", "localization_status_stale",
    "arm_not_stowed", "source_command_nonfinite", "unknown"}) {
    EXPECT_FALSE(guardSensorRecoveryTimeout(reason,300.0).has_value()) << reason;
  }
}
