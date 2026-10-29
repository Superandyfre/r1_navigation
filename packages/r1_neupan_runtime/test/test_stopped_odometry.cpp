// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0
#include <limits>
#include "gtest/gtest.h"
#include "r1_neupan_runtime/stopped_odometry.hpp"

namespace
{
class Odom : public ::testing::Test
{
public:
  r1_neupan_runtime::StoppedOdometry evidence;
  std::array<double, 6> velocity{};
  void sample(
    int offset, int lag = 0, const std::string & frame = "odom",
    const std::string & child = "base_link")
  {
    evidence.observe(
      1000000000LL + offset * 1000000LL,
      1000000000LL + (offset + lag) * 1000000LL, 10. + offset / 1000.,
      frame, child, "odom", "base_link", velocity);
  }
  void stopped() {for (int t = 0; t <= 300; t += 50) {sample(t);}}
};
TEST_F(Odom, RequiresFreshContinuousObservationInBothClocks)
{
  sample(0); EXPECT_FALSE(evidence.stopped(10., 1000000000));
  stopped(); EXPECT_TRUE(evidence.stopped(10.3, 1300000000));
  EXPECT_FALSE(evidence.stopped(10.5, 1500000000));
  EXPECT_FALSE(evidence.stopped(10.3, 1600000000));
  EXPECT_FALSE(evidence.stopped(10.2, 1300000000));
}
TEST_F(Odom, DuplicateCannotRefreshReceiptOrAccumulateWindow)
{
  sample(0);
  for (int t = 50; t <= 500; t += 50) {
    evidence.observe(
      1000000000, 1000000000, 10. + t / 1000.,
      "odom", "base_link", "odom", "base_link", velocity);
  }
  EXPECT_FALSE(evidence.stopped(10.5, 1000000000));
  stopped(); EXPECT_TRUE(evidence.stopped(10.3, 1300000000));
  evidence.observe(
    1300000000, 1300000000, 10.6, "odom", "base_link", "odom", "base_link",
    velocity);
  EXPECT_FALSE(evidence.stopped(10.6, 1300000000));
}
TEST_F(Odom, OldFutureZeroAndWrongFramesReject)
{
  stopped(); sample(350, 200); EXPECT_FALSE(evidence.stopped(10.35, 1550000000));
  sample(400, -1); EXPECT_FALSE(evidence.stopped(10.4, 1400000000));
  evidence.observe(0, 0, 11., "odom", "base_link", "odom", "base_link", velocity);
  EXPECT_FALSE(evidence.stopped(11., 0));
  sample(450, 0, "map"); EXPECT_FALSE(evidence.stopped(10.45, 1450000000));
  sample(500, 0, "odom", "other"); EXPECT_FALSE(evidence.stopped(10.5, 1500000000));
}
TEST_F(Odom, NonfiniteAndMotionInAnyAxisBreakObservation)
{
  for (std::size_t i = 0; i < 6; ++i) {
    evidence = r1_neupan_runtime::StoppedOdometry(); velocity.fill(0.); stopped();
    velocity[i] = .1; sample(350); EXPECT_FALSE(evidence.stopped(10.35, 1350000000));
    velocity[i] = std::numeric_limits<double>::quiet_NaN(); sample(400);
    EXPECT_FALSE(evidence.stopped(10.4, 1400000000));
  }
}
TEST_F(Odom, GapsAndResetRequireNewObservationAndDoNotAcceptReplays)
{
  stopped(); sample(500); EXPECT_FALSE(evidence.stopped(10.5, 1500000000));
  evidence.reset(); sample(100); EXPECT_FALSE(evidence.stopped(10.1, 1100000000));
  for (int t = 550; t <= 850; t += 50) {
    sample(t);
  }
  EXPECT_TRUE(evidence.stopped(10.85, 1850000000));
}
}  // namespace
