#include <gtest/gtest.h>
#include "navigation_neupan_bridge/arm_mesh_resources.hpp"
using namespace navigation_neupan_bridge;
TEST(ArmMeshResources, AddsExplicitToolOrHeldObject)
{
  auto meshes=armMeshResources();ASSERT_EQ(meshes.size(),36u);
  appendConfiguredMeshes(meshes,{"held_object"},{"package://test/held.stl"},{1.});
  ASSERT_EQ(meshes.size(),37u);EXPECT_EQ(meshes.back().frame,"held_object");
}
TEST(ArmMeshResources, RejectsPartialOrInvalidModelConfiguration)
{
  auto meshes=armMeshResources();
  EXPECT_THROW(appendConfiguredMeshes(meshes,{"a"},{},{}),std::invalid_argument);
  EXPECT_THROW(appendConfiguredMeshes(meshes,{"a"},{"path"},{0.}),std::invalid_argument);
  EXPECT_EQ(meshes.size(),36u);
}
