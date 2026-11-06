// Offline benchmark; no ROS nodes, subscriptions or hardware commands.
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include "geometric_shapes/mesh_operations.h"
#include "navigation_neupan_bridge/mesh_surface_query.hpp"
using navigation_neupan_bridge::MeshSurfaceQuery;

int main()
{
  const auto begin=std::chrono::steady_clock::now();
  struct Part {std::unique_ptr<MeshSurfaceQuery> mesh; fcl::Vector3d translation;};
  std::vector<Part> parts;
  auto add=[&](const std::string & uri,double scale,const fcl::Vector3d & t) {
    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(uri,Eigen::Vector3d(scale,scale,scale)));
    if (!mesh || !mesh->vertex_count || !mesh->triangle_count) throw std::runtime_error(uri);
    std::vector<fcl::Vector3d> v;v.reserve(mesh->vertex_count);
    for (unsigned i=0;i<mesh->vertex_count;++i) v.emplace_back(mesh->vertices[i*3],mesh->vertices[i*3+1],mesh->vertices[i*3+2]);
    std::vector<fcl::Triangle> tri;tri.reserve(mesh->triangle_count);
    for (unsigned i=0;i<mesh->triangle_count;++i) tri.emplace_back(mesh->triangles[i*3],mesh->triangles[i*3+1],mesh->triangles[i*3+2]);
    parts.push_back({std::make_unique<MeshSurfaceQuery>(v,tri,.02),t});
  };
  for (int arm=0;arm<2;++arm) {
    const std::string model=arm==0?"AR5-5_08L-W4C1C5":"AR5-5_08R-W4C1C5";
    const std::string root="package://rokae_description/meshes/"+model+"/visual/";
    for (int i=0;i<8;++i) {
      const std::string file=i?model+"_link"+std::to_string(i)+".stl":model+"_base.stl";
      add(root+file,.001,{.04*i,arm==0?.35:-.35,.2+.04*i});
    }
    add("package://r1_description/meshes/moduan_link.STL",1.,{.35,arm==0?.35:-.35,.45});
    const std::string g="package://rpp_components_description/meshes/gripper/agibot/omniPicker/";
    for (const auto & name:{"base_link.STL","narrow1_Link.STL","narrow2_Link.STL","narrow3_Link.STL",
      "narrow_loop_Link.STL","wide1_Link.STL","wide2_Link.STL","wide3_Link.STL","wide_loop_Link.STL"}) {
      add(g+name,1.,{.38,arm==0?.35:-.35,.45});
    }
  }
  const double build=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  std::mt19937 generator(42);std::uniform_real_distribution<double> xy(-2.,2.),z(-.1,.8);
  std::vector<fcl::Vector3d> points;for(int i=0;i<20000;++i)points.emplace_back(xy(generator),xy(generator),z(generator));
  std::size_t hits=0;
  const auto start=std::chrono::steady_clock::now();
  for (int frame=0;frame<3;++frame) for (const auto & p:points) {
    for (const auto & part:parts) if (part.mesh->nearSurface(p-part.translation)) {++hits;break;}
  }
  const double sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cout<<std::fixed<<std::setprecision(3)<<"parts="<<parts.size()<<" build_s="<<build
    <<" points_per_frame="<<points.size()<<" frames=3 mean_frame_ms="<<(sec*1000/3)<<" hits="<<hits<<std::endl;
}
