#include <gtest/gtest.h>
#include <limits>
#include "navigation_neupan_bridge/convex_footprint_contains.hpp"
using navigation_neupan_bridge::convexFootprintContains;
TEST(ConvexFootprint, AllowsOnlyGeometryInsideOldEnvelope) {
  const std::vector<double> old={-1.,-1.,1.,-1.,1.,1.,-1.,1.};
  const std::vector<double> small={-.5,-.5,.5,-.5,.5,.5,-.5,.5};
  EXPECT_TRUE(convexFootprintContains(old,small,0,0,.7));
  EXPECT_TRUE(convexFootprintContains(old,small,.5,.5,0));
  EXPECT_FALSE(convexFootprintContains(old,small,.51,.5,0));
  EXPECT_FALSE(convexFootprintContains(small,old,0,0,0));
}
TEST(ConvexFootprint, RejectsUntrustedGeometry) {
  const std::vector<double> shape={-1.,-1.,1.,-1.,1.,1.,-1.,1.};
  const std::vector<double> concave={-1.,-1.,1.,-1.,0.,0.,1.,1.,-1.,1.};
  EXPECT_FALSE(convexFootprintContains(concave,shape,0,0,0));
  EXPECT_FALSE(convexFootprintContains(shape,shape,0,0,std::numeric_limits<double>::quiet_NaN()));
}
