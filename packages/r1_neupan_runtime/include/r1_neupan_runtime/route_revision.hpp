#pragma once
#include <cmath>
#include "nav_msgs/msg/path.hpp"

namespace r1_neupan_runtime
{
inline bool validRouteRevision(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::Path & active)
{
  if (path.poses.empty() || active.poses.empty() || path.header.frame_id.empty() ||
    path.header.frame_id != active.header.frame_id) {return false;}
  for (const auto & p : path.poses) {
    const auto & v = p.pose.position;
    const auto & q = p.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if ((!p.header.frame_id.empty() && p.header.frame_id != path.header.frame_id) ||
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
      !std::isfinite(norm) || std::abs(norm - 1.) > 1e-3) {return false;}
  }
  const auto & a = path.poses.back().pose;
  const auto & b = active.poses.back().pose;
  const auto & q = a.orientation;
  const auto & r = b.orientation;
  const double dot = q.x*r.x + q.y*r.y + q.z*r.z + q.w*r.w;
  return std::hypot(a.position.x-b.position.x, a.position.y-b.position.y) < 1e-4 &&
    std::abs(a.position.z-b.position.z) < 1e-4 && std::abs(std::abs(dot)-1.) < 1e-6;
}
}
