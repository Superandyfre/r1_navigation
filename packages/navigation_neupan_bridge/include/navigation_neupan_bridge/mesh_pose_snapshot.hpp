// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace navigation_neupan_bridge
{
struct MeshPose
{
  double x, y, z, qx, qy, qz, qw;
};

inline bool meshPosesDiffer(
  const std::vector<MeshPose> & current, const std::vector<MeshPose> & previous,
  double translation_tolerance, double rotation_tolerance)
{
  if (current.empty() || current.size() != previous.size()) {return true;}
  for (std::size_t i = 0; i < current.size(); ++i) {
    const auto & a = current[i]; const auto & b = previous[i];
    const double dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
    const double norm_a = std::sqrt(a.qx*a.qx+a.qy*a.qy+a.qz*a.qz+a.qw*a.qw);
    const double norm_b = std::sqrt(b.qx*b.qx+b.qy*b.qy+b.qz*b.qz+b.qw*b.qw);
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
      !std::isfinite(norm_a) || !std::isfinite(norm_b) || norm_a < 1e-9 || norm_b < 1e-9)
    {return true;}
    if (std::sqrt(dx*dx+dy*dy+dz*dz) >= translation_tolerance) {return true;}
    const double dot = std::clamp(std::abs((a.qx*b.qx+a.qy*b.qy+a.qz*b.qz+a.qw*b.qw) /
      (norm_a*norm_b)), 0.0, 1.0);
    if (2.0*std::acos(dot) >= rotation_tolerance) {return true;}
  }
  return false;
}
}
