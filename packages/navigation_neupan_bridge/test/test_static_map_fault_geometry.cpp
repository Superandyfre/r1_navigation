#include <gtest/gtest.h>

#include <vector>

#include "navigation_neupan_bridge/static_map_fault_geometry.hpp"

namespace navigation_neupan_bridge
{

TEST(StaticMapFaultGeometry, BoundaryCellDoesNotImplyLocalizationFault)
{
  const std::vector<double> chassis{
    -0.341, -0.266, -0.341, 0.266,
    0.341, 0.266, 0.341, -0.266};

  EXPECT_FALSE(deepInsideFootprint(chassis, -0.339253, 0.269939, 0.05));
  EXPECT_FALSE(deepInsideFootprint(chassis, -0.32, 0.24, 0.05));
  EXPECT_TRUE(deepInsideFootprint(chassis, -0.20, 0.10, 0.05));
  EXPECT_TRUE(deepInsideFootprint(chassis, 0.0, 0.0, 0.05));
}

}  // namespace navigation_neupan_bridge
