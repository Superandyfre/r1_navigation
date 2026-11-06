// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>
#include <vector>
#include <fcl/fcl.h>

namespace navigation_neupan_bridge
{
// Mesh and point are expressed in one link's local coordinates. This answers
// It handles surface proximity and the interior of verified closed meshes;
// non-closed meshes intentionally make no inside/outside claim.
class MeshSurfaceQuery
{
public:
  MeshSurfaceQuery(const std::vector<fcl::Vector3d> & vertices,
    const std::vector<fcl::Triangle> & triangles, double tolerance)
  : model_(std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>()),
    probe_(std::make_shared<fcl::Sphered>(tolerance)), tolerance_(tolerance)
  {
    if (vertices.empty() || triangles.empty() || !std::isfinite(tolerance) || tolerance <= 0.) {
      throw std::invalid_argument("invalid_mesh_surface_query");
    }
    lower_ = vertices.front(); upper_ = vertices.front();
    for (const auto & p : vertices) {
      if (!p.allFinite()) {throw std::invalid_argument("nonfinite_mesh_vertex");}
      lower_ = lower_.cwiseMin(p); upper_ = upper_.cwiseMax(p);
    }
    for (const auto & t : triangles) {
      for (int i=0; i<3; ++i) {
        if (t[i] >= vertices.size()) {throw std::invalid_argument("invalid_mesh_triangle");}
      }
    }
    if (model_->beginModel(static_cast<int>(triangles.size()), static_cast<int>(vertices.size())) ||
      model_->addSubModel(vertices, triangles) || model_->endModel()) {
      throw std::runtime_error("mesh_bvh_build_failed");
    }
    closed_ = isClosed(vertices,triangles);
  }

  bool hasClosedSurface() const {return closed_;}

  bool nearSurface(const fcl::Vector3d & point) const
  {
    if (!point.allFinite()) {return false;}
    if ((point.array() < (lower_.array()-tolerance_)).any() ||
      (point.array() > (upper_.array()+tolerance_)).any()) {return false;}
    fcl::Transform3d probe_pose=fcl::Transform3d::Identity();
    probe_pose.translation()=point;
    const fcl::Transform3d identity=fcl::Transform3d::Identity();
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(model_.get(), identity, probe_.get(), probe_pose, request, result);
    return result.isCollision() || (closed_ && insideClosedMesh(point));
  }

private:
  using Key=std::array<std::int64_t,3>;
  using Edge=std::array<Key,2>;
  static bool isClosed(const std::vector<fcl::Vector3d> & vertices,
    const std::vector<fcl::Triangle> & triangles)
  {
    // STL meshes may repeat the coordinates at each triangle. Weld only for
    // topology checking, at 0.1 micrometre resolution; query geometry is intact.
    std::vector<Key> keys;keys.reserve(vertices.size());
    for (const auto & p:vertices) {
      if ((p.array().abs()>1000.).any()) {return false;}
      keys.push_back({static_cast<std::int64_t>(std::llround(p.x()*1e7)),
        static_cast<std::int64_t>(std::llround(p.y()*1e7)),
        static_cast<std::int64_t>(std::llround(p.z()*1e7))});
    }
    std::vector<Edge> edges;edges.reserve(triangles.size()*3);
    for (const auto & t:triangles) {
      // Duplicate-vertex triangles have zero surface area; retain them in the
      // original BVH but exclude them from the closed-topology edge count.
      if (keys[t[0]]==keys[t[1]] || keys[t[1]]==keys[t[2]] ||
        keys[t[2]]==keys[t[0]]) {continue;}
      for (int j=0;j<3;++j) {
        Key a=keys[t[j]],b=keys[t[(j+1)%3]];
        if (b<a) {std::swap(a,b);}
        edges.push_back({a,b});
      }
    }
    if (edges.empty()) {return false;}
    std::sort(edges.begin(),edges.end());
    for (std::size_t i=0;i<edges.size();) {
      std::size_t j=i+1;while (j<edges.size() && edges[j]==edges[i]) {++j;}
      if (j-i!=2) {return false;}
      i=j;
    }
    return true;
  }
  static bool rayIntersectsBox(const fcl::Vector3d & p,const fcl::Vector3d & d,
    const fcl::OBBd & box)
  {
    const fcl::Vector3d local=box.axis.transpose()*(p-box.To);
    const fcl::Vector3d direction=box.axis.transpose()*d;
    double low=0.,high=std::numeric_limits<double>::infinity();
    for (int i=0;i<3;++i) {
      if (std::abs(direction[i])<1e-12) {
        if (std::abs(local[i])>box.extent[i]+1e-10) {return false;}
        continue;
      }
      double a=(-box.extent[i]-1e-10-local[i])/direction[i];
      double b=(box.extent[i]+1e-10-local[i])/direction[i];
      if (a>b) {std::swap(a,b);}
      low=std::max(low,a);high=std::min(high,b);
      if (low>high) {return false;}
    }
    return true;
  }
  bool insideClosedMesh(const fcl::Vector3d & p) const
  {
    if (model_->getNumBVs()==0) {return false;}
    const fcl::Vector3d d=fcl::Vector3d(1.,.371,.529).normalized();
    std::vector<int> stack{0};std::vector<double> hits;
    while (!stack.empty()) {
      const int index=stack.back();stack.pop_back();
      const auto & node=model_->getBV(index);
      if (!rayIntersectsBox(p,d,node.bv.obb)) {continue;}
      if (!node.isLeaf()) {stack.push_back(node.leftChild());stack.push_back(node.rightChild());continue;}
      const auto & tri=model_->tri_indices[node.primitiveId()];
      const auto & a=model_->vertices[tri[0]];
      const auto edge1=model_->vertices[tri[1]]-a;
      const auto edge2=model_->vertices[tri[2]]-a;
      const auto h=d.cross(edge2);const double det=edge1.dot(h);
      if (std::abs(det)<1e-12) {continue;}
      const double inv=1./det;const auto rel=p-a;
      const double u=inv*rel.dot(h);
      if (u < -1e-10 || u > 1.+1e-10) {continue;}
      const auto q=rel.cross(edge1);const double v=inv*d.dot(q);
      if (v < -1e-10 || u+v > 1.+1e-10) {continue;}
      const double distance=inv*edge2.dot(q);
      if (distance>1e-10) {hits.push_back(distance);}
    }
    std::sort(hits.begin(),hits.end());
    std::size_t count=0;double last=-1.;
    for (double distance:hits) {
      if (count==0 || std::abs(distance-last)>1e-8) {++count;last=distance;}
    }
    return count%2==1;
  }
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> model_;
  std::shared_ptr<fcl::Sphered> probe_;
  double tolerance_;
  bool closed_{false};
  fcl::Vector3d lower_, upper_;
};
}
