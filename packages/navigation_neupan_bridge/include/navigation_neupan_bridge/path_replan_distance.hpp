#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include "nav_msgs/msg/path.hpp"
namespace navigation_neupan_bridge {
inline double distanceToRoute(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & path) {
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[std::min(i + 1, path.poses.size() - 1)].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 1e-12 ?
      std::clamp(((p.x-a.x)*dx + (p.y-a.y)*dy) / length2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(p.x-a.x-t*dx, p.y-a.y-t*dy));
  }
  return best;
}
}
