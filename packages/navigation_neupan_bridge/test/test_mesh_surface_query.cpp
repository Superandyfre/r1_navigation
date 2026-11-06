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

TEST(MeshSurfaceQuery, ClosedMeshInteriorAndOpenMeshAreDistinct)
{
  const std::vector<fcl::Vector3d> v{{0,0,0},{1,0,0},{1,1,0},{0,1,0},
    {0,0,1},{1,0,1},{1,1,1},{0,1,1}};
  const std::vector<fcl::Triangle> t{{0,2,1},{0,3,2},{4,5,6},{4,6,7},
    {0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
  MeshSurfaceQuery cube(v,t,.01);
  EXPECT_TRUE(cube.hasClosedSurface());
  EXPECT_TRUE(cube.nearSurface({.5,.5,.5}));
  EXPECT_FALSE(cube.nearSurface({1.1,.5,.5}));
  auto with_degenerate=t;with_degenerate.emplace_back(0,0,1);
  MeshSurfaceQuery same_closed(v,with_degenerate,.01);
  EXPECT_TRUE(same_closed.hasClosedSurface());
  EXPECT_TRUE(same_closed.nearSurface({.5,.5,.5}));
  auto open=t;open.pop_back();
  MeshSurfaceQuery incomplete(v,open,.01);
  EXPECT_FALSE(incomplete.hasClosedSurface());
  EXPECT_FALSE(incomplete.nearSurface({.5,.5,.5}));
}
