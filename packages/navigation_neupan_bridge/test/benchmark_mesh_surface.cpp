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
  std::size_t surface_checks=0,surface_misses=0,negative_checks=0,negative_misclassified=0;
  constexpr int pose_count=5;
  auto add=[&](const std::string & uri,double scale,const fcl::Vector3d & t) {
    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(uri,Eigen::Vector3d(scale,scale,scale)));
    if (!mesh || !mesh->vertex_count || !mesh->triangle_count) throw std::runtime_error(uri);
    std::vector<fcl::Vector3d> v;v.reserve(mesh->vertex_count);
    for (unsigned i=0;i<mesh->vertex_count;++i) v.emplace_back(mesh->vertices[i*3],mesh->vertices[i*3+1],mesh->vertices[i*3+2]);
    std::vector<fcl::Triangle> tri;tri.reserve(mesh->triangle_count);
    for (unsigned i=0;i<mesh->triangle_count;++i) tri.emplace_back(mesh->triangles[i*3],mesh->triangles[i*3+1],mesh->triangles[i*3+2]);
    auto query=std::make_unique<MeshSurfaceQuery>(v,tri,.02);
    // Synthetic labelled examples in five distinct 3D poses per real mesh.
    // Ground truth: perturbed sampled triangle surface is self-return; a
    // point above the mesh's entire world-Z range is an external obstacle.
    for (int pose=0;pose<pose_count;++pose) {
      fcl::Transform3d world_from_part=fcl::Transform3d::Identity();
      world_from_part.linear()=(Eigen::AngleAxisd(.23*pose,fcl::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(.17*pose,fcl::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-.11*pose,fcl::Vector3d::UnitX())).toRotationMatrix();
      world_from_part.translation()=t+fcl::Vector3d(.03*pose,-.02*pose,.01*pose);
      const fcl::Transform3d part_from_world=world_from_part.inverse();
      for (std::size_t sample=0;sample<100;++sample) {
        const auto & face=tri[(sample*tri.size())/100];
        const auto & a=v[face[0]],&b=v[face[1]],&c=v[face[2]];
        fcl::Vector3d normal=(b-a).cross(c-a);
        if (normal.norm()<1e-10) {continue;}
        normal.normalize();
        const fcl::Vector3d point=(a+b+c)/3.;
        for (double offset : {-.005,.005}) {
          const auto world=world_from_part*(point+offset*normal);
          ++surface_checks;
          if (!query->nearSurface(part_from_world*world)) {++surface_misses;}
        }
      }
      double top_z=(world_from_part*v.front()).z();
      for (const auto & vertex:v) {
        top_z=std::max(top_z,(world_from_part*vertex).z());
      }
      // Every XY is a transformed mesh vertex, but the obstacle is above
      // the whole mesh. The legacy expanded XY hull covers each coordinate.
      for (std::size_t sample=0;sample<100;++sample) {
        fcl::Vector3d obstacle=world_from_part*v[(sample*v.size())/100];
        for (double clearance : {.03,.10,.30}) {
          obstacle.z()=top_z+clearance;
          ++negative_checks;
          if (query->nearSurface(part_from_world*obstacle)) {++negative_misclassified;}
        }
      }
    }
    parts.push_back({std::move(query),t});
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
  std::size_t closed=0;for (const auto & part:parts) if(part.mesh->hasClosedSurface())++closed;
  std::mt19937 generator(42);std::uniform_real_distribution<double> xy(-2.,2.),z(-.1,.8);
  std::vector<fcl::Vector3d> points;for(int i=0;i<20000;++i)points.emplace_back(xy(generator),xy(generator),z(generator));
  std::size_t hits=0;
  const auto start=std::chrono::steady_clock::now();
  for (int frame=0;frame<3;++frame) for (const auto & p:points) {
    for (const auto & part:parts) if (part.mesh->nearSurface(p-part.translation)) {++hits;break;}
  }
  const double sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cout<<std::fixed<<std::setprecision(3)<<"parts="<<parts.size()<<" build_s="<<build
    <<" closed="<<closed<<" poses_per_part="<<pose_count
    <<" surface_checks="<<surface_checks<<" surface_misses="<<surface_misses
    <<" negative_checks="<<negative_checks<<" negative_misclassified="<<negative_misclassified
    <<" points_per_frame="<<points.size()<<" frames=3 mean_frame_ms="<<(sec*1000/3)<<" hits="<<hits<<std::endl;
  return (surface_misses || negative_misclassified) ? 1 : 0;
}
