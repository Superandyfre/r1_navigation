#include <gtest/gtest.h>

#include "navigation_neupan_bridge/passage_clearance_state.hpp"

using navigation_neupan_bridge::passageClearanceState;

TEST(PassageClearanceState, WidthAloneDoesNotHideOffCenterRoute)
{
  EXPECT_EQ(passageClearanceState(true, 1.73, 1.01, 0.03, 0.89, 0.10),
    "ROUTE_TOO_CLOSE");
  EXPECT_EQ(passageClearanceState(true, 1.73, 1.01, 0.30, 0.62, 0.10),
    "CLEAR");
}

TEST(PassageClearanceState, NarrowAndUnmeasuredStayDistinct)
{
  EXPECT_EQ(passageClearanceState(true, 0.90, 1.01, 0.04, 0.05, 0.10),
    "TOO_NARROW");
  EXPECT_EQ(passageClearanceState(false, 0.0, 1.01, 0.0, 0.0, 0.10),
    "INSUFFICIENT_SCAN");
}
