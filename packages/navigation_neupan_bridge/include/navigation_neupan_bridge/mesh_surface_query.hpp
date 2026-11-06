// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
#include <fcl/fcl.h>

namespace navigation_neupan_bridge
{
// Mesh and point are expressed in one link's local coordinates. This answers
// a *surface proximity* question; it does not claim inside/outside containment.
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
  }

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
    return result.isCollision();
  }

private:
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> model_;
  std::shared_ptr<fcl::Sphered> probe_;
  double tolerance_;
  fcl::Vector3d lower_, upper_;
};
}
