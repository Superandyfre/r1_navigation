#include <gtest/gtest.h>
#include "navigation_neupan_bridge/path_replan_distance.hpp"
using namespace navigation_neupan_bridge;
TEST(PathReplanDistance, PrefixProgressDoesNotResetSparseRoute) {
 nav_msgs::msg::Path route;
 geometry_msgs::msg::PoseStamped pose;
 route.poses.push_back(pose); pose.pose.position.x=2; route.poses.push_back(pose);
 geometry_msgs::msg::Point p; p.x=1;
 EXPECT_NEAR(distanceToRoute(p,route),0.0,1e-9);
 p.y=0.10; EXPECT_LT(distanceToRoute(p,route),0.15);
 p.y=0.18; EXPECT_GT(distanceToRoute(p,route),0.15);
}
TEST(PathReplanDistance, DuplicateRotationPosesAndRouteEnd) {
 nav_msgs::msg::Path route; geometry_msgs::msg::PoseStamped pose;
 route.poses.push_back(pose); route.poses.push_back(pose);
 geometry_msgs::msg::Point p; p.x=0.3;
 EXPECT_NEAR(distanceToRoute(p,route),0.3,1e-9);
 pose.pose.position.x=1; route.poses.push_back(pose); p.x=1.2;
 EXPECT_NEAR(distanceToRoute(p,route),0.2,1e-9);
}
