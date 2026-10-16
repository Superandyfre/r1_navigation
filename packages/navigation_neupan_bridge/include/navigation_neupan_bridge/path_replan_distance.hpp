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

inline double forwardRouteDeviation(
  const nav_msgs::msg::Path & candidate, const nav_msgs::msg::Path & current,
  double lookahead_m = 1.0, double sample_step_m = 0.1)
{
  if (candidate.poses.empty() || current.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double travelled = 0.0;
  double worst = distanceToRoute(candidate.poses.front().pose.position, current);
  for (size_t index = 1; index < candidate.poses.size() && travelled < lookahead_m; ++index) {
    const auto & a = candidate.poses[index - 1].pose.position;
    const auto & b = candidate.poses[index].pose.position;
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double covered = std::min(length, lookahead_m - travelled);
    const size_t steps = std::max<size_t>(1, std::ceil(covered / sample_step_m));
    for (size_t step = 1; step <= steps; ++step) {
      const double fraction = length > 1e-9 ?
        covered * static_cast<double>(step) / static_cast<double>(steps) / length : 0.0;
      geometry_msgs::msg::Point point;
      point.x = a.x + fraction * (b.x - a.x);
      point.y = a.y + fraction * (b.y - a.y);
      worst = std::max(worst, distanceToRoute(point, current));
    }
    travelled += length;
  }
  return worst;
}
}
