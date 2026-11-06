#include <gtest/gtest.h>
#include "navigation_neupan_bridge/mesh_surface_query.hpp"
using navigation_neupan_bridge::MeshSurfaceQuery;

TEST(MeshSurfaceQuery, UsesThreeDimensionalTriangleDistance)
{
  const std::vector<fcl::Vector3d> points{{0,0,0},{1,0,0},{0,1,0}};
  const std::vector<fcl::Triangle> triangles{{0,1,2}};
  MeshSurfaceQuery mesh(points,triangles,.02);
  EXPECT_TRUE(mesh.nearSurface({.25,.25,.01}));
  EXPECT_FALSE(mesh.nearSurface({.25,.25,.05}));  // Same XY, different height.
  EXPECT_FALSE(mesh.nearSurface({.9,.9,0.}));  // Within AABB, outside triangle.
  EXPECT_TRUE(mesh.nearSurface({.5,.5,.005}));
}
TEST(MeshSurfaceQuery, RejectsBrokenMesh)
{
  EXPECT_THROW(MeshSurfaceQuery({{0,0,0}},{{0,1,2}},.02),std::invalid_argument);
  EXPECT_THROW(MeshSurfaceQuery({}, {}, 0.),std::invalid_argument);
}
