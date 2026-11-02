#include <gtest/gtest.h>
#include "navigation_neupan_bridge/scan_side_clearance.hpp"
using navigation_neupan_bridge::SidePoint;
using navigation_neupan_bridge::sampledSideClearance;
TEST(SampledSideClearance, SymmetricSidesIgnoreFrontObstacle)
{
  const std::vector<SidePoint> polygon{{-.3,-.3},{.3,-.3},{.3,.3},{-.3,.3}};
  const auto value=sampledSideClearance(polygon,{{0,.4},{0,-.4},{.7,.01}});
  EXPECT_NEAR(value.left,.1,1e-9); EXPECT_NEAR(value.right,.1,1e-9);
  EXPECT_EQ(value.left_samples,1U); EXPECT_EQ(value.right_samples,1U);
}
TEST(SampledSideClearance, DoesNotSubtractProtrusionAtAnotherLongitudinalPosition)
{
  const std::vector<SidePoint> polygon{{-.3,-.3},{.3,-.3},{.3,.6},{.2,.6},{0,.3},{-.3,.3}};
  const auto value=sampledSideClearance(polygon,{{-.2,.4},{-.2,-.4}});
  // Whole-polygon maximum would incorrectly report .4-.6=-.2.
  EXPECT_NEAR(value.left,.1,1e-9); EXPECT_NEAR(value.right,.1,1e-9);
}
TEST(SampledSideClearance, MissingSideIsUnknownAndOverlapRemainsSigned)
{
  const std::vector<SidePoint> polygon{{-.3,-.3},{.3,-.3},{.3,.3},{-.3,.3}};
  const auto value=sampledSideClearance(polygon,{{0,.28},{1,-.4}});
  EXPECT_NEAR(value.left,-.02,1e-9); EXPECT_EQ(value.right_samples,0U);
  EXPECT_TRUE(std::isinf(value.right));
}
