#include <gtest/gtest.h>
#include <limits>
#include "navigation_neupan_bridge/mesh_pose_snapshot.hpp"
using navigation_neupan_bridge::MeshPose;
using navigation_neupan_bridge::meshPosesDiffer;

TEST(MeshPoseSnapshot, IncludesAllPartsAndGripperMotion)
{
  std::vector<MeshPose> original(36, {0,0,0,0,0,0,1});
  auto changed=original;
  EXPECT_FALSE(meshPosesDiffer(changed,original,.005,.01));
  changed[35].x=.006;  // Gripper moved, arm links remain identical.
  EXPECT_TRUE(meshPosesDiffer(changed,original,.005,.01));
  changed=original;changed[35].qz=std::sin(.006);changed[35].qw=std::cos(.006);
  EXPECT_TRUE(meshPosesDiffer(changed,original,.005,.01));
}
TEST(MeshPoseSnapshot, SmallNoiseAndQuaternionSignAreEquivalent)
{
  std::vector<MeshPose> original{{0,0,0,0,0,0,1}};
  std::vector<MeshPose> same{{.001,0,0,0,0,0,-1}};
  EXPECT_FALSE(meshPosesDiffer(same,original,.005,.01));
  EXPECT_TRUE(meshPosesDiffer({},original,.005,.01));
  same[0].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(meshPosesDiffer(same,original,.005,.01));
}
