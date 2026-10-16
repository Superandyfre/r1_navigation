#include <vector>

#include "gtest/gtest.h"
#include "navigation_neupan_bridge/polygon_boundary_distance.hpp"

using navigation_neupan_bridge::Point2D;
using navigation_neupan_bridge::maxBoundaryDifference;

TEST(PolygonBoundaryDistance, IgnoresVertexCountAndStartIndex)
{
  const std::vector<Point2D> rectangle{{0, 0}, {2, 0}, {2, 1}, {0, 1}};
  const std::vector<Point2D> same_boundary{
    {2, 0}, {2, 0.5}, {2, 1}, {1, 1}, {0, 1}, {0, 0}, {1, 0}};
  EXPECT_NEAR(maxBoundaryDifference(rectangle, same_boundary), 0.0, 1.0e-9);
}

TEST(PolygonBoundaryDistance, DetectsMaterialMovement)
{
  const std::vector<Point2D> rectangle{{0, 0}, {2, 0}, {2, 1}, {0, 1}};
  const std::vector<Point2D> shifted{{0.04, 0}, {2.04, 0}, {2.04, 1}, {0.04, 1}};
  EXPECT_NEAR(maxBoundaryDifference(rectangle, shifted), 0.04, 1.0e-9);
}
