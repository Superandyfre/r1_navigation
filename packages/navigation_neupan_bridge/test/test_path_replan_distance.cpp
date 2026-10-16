#include <gtest/gtest.h>
#include "navigation_neupan_bridge/path_replan_distance.hpp"
#include "navigation_neupan_bridge/route_update_gate.hpp"
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

TEST(PathReplanDistance, DetectsBendAheadOfUnchangedStart) {
 nav_msgs::msg::Path current, candidate;
 geometry_msgs::msg::PoseStamped pose;
 current.poses.push_back(pose);
 pose.pose.position.x=2; current.poses.push_back(pose);
 pose.pose.position.x=0; candidate.poses.push_back(pose);
 pose.pose.position.x=0.4; candidate.poses.push_back(pose);
 pose.pose.position.x=0.6; pose.pose.position.y=0.3; candidate.poses.push_back(pose);
 EXPECT_GT(forwardRouteDeviation(candidate,current),0.15);
}
TEST(PathReplanDistance, PrefixTrimmingAndTinyReplanStayStable) {
 nav_msgs::msg::Path current, candidate;
 geometry_msgs::msg::PoseStamped pose;
 current.poses.push_back(pose);
 pose.pose.position.x=2; current.poses.push_back(pose);
 pose.pose.position.x=0.5; pose.pose.position.y=0.02; candidate.poses.push_back(pose);
 pose.pose.position.x=1.5; candidate.poses.push_back(pose);
 EXPECT_LT(forwardRouteDeviation(candidate,current),0.15);
}

TEST(PathReplanDistance, StableModerateClearanceShiftIsPublished) {
 nav_msgs::msg::Path current, candidate;
 geometry_msgs::msg::PoseStamped pose;
 current.poses.push_back(pose); pose.pose.position.x=2; current.poses.push_back(pose);
 pose.pose.position.x=0; pose.pose.position.y=0.10; candidate.poses.push_back(pose);
 pose.pose.position.x=2; candidate.poses.push_back(pose);
 RouteUpdateGate gate;
 const auto start=RouteUpdateGate::TimePoint{};
 gate.published(start);
 EXPECT_FALSE(gate.consider(candidate,current,start+std::chrono::seconds(1)));
 EXPECT_TRUE(gate.consider(candidate,current,start+std::chrono::seconds(2)));
 gate.published(start+std::chrono::seconds(2));
 EXPECT_FALSE(gate.consider(candidate,candidate,start+std::chrono::seconds(3)));
}

TEST(PathReplanDistance, AlternatingModerateRoutesDoNotResetWorker) {
 nav_msgs::msg::Path current, upper, lower;
 geometry_msgs::msg::PoseStamped pose;
 current.poses.push_back(pose); pose.pose.position.x=2; current.poses.push_back(pose);
 pose.pose.position.x=0; pose.pose.position.y=0.10; upper.poses.push_back(pose);
 pose.pose.position.x=2; upper.poses.push_back(pose);
 pose.pose.position.x=0; pose.pose.position.y=-0.10; lower.poses.push_back(pose);
 pose.pose.position.x=2; lower.poses.push_back(pose);
 RouteUpdateGate gate;
 const auto start=RouteUpdateGate::TimePoint{};
 gate.published(start);
 EXPECT_FALSE(gate.consider(upper,current,start+std::chrono::seconds(2)));
 EXPECT_FALSE(gate.consider(lower,current,start+std::chrono::seconds(3)));
 EXPECT_FALSE(gate.consider(upper,current,start+std::chrono::seconds(4)));
}

TEST(PathReplanDistance, MaterialBendAlsoNeedsRepeatedEvidence) {
 nav_msgs::msg::Path current, candidate;
 geometry_msgs::msg::PoseStamped pose;
 current.poses.push_back(pose); pose.pose.position.x=2; current.poses.push_back(pose);
 pose.pose.position.x=0; candidate.poses.push_back(pose);
 pose.pose.position.x=0.4; candidate.poses.push_back(pose);
 pose.pose.position.x=0.6; pose.pose.position.y=0.3; candidate.poses.push_back(pose);
 RouteUpdateGate gate;
 const auto start=RouteUpdateGate::TimePoint{};
 gate.published(start);
 EXPECT_FALSE(gate.consider(candidate,current,start+std::chrono::seconds(2)));
 EXPECT_TRUE(gate.consider(candidate,current,start+std::chrono::seconds(3)));
}
