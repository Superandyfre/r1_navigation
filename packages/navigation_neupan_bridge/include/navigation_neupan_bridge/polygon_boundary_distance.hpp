#ifndef NAVIGATION_NEUPAN_BRIDGE__POLYGON_BOUNDARY_DISTANCE_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__POLYGON_BOUNDARY_DISTANCE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace navigation_neupan_bridge
{
struct Point2D
{
  double x{};
  double y{};
};

inline double maxBoundaryDifference(
  const std::vector<Point2D> & first, const std::vector<Point2D> & second)
{
  if (first.size() < 3 || second.size() < 3) {
    return std::numeric_limits<double>::infinity();
  }
  const auto directed_distance = [](const std::vector<Point2D> & source,
      const std::vector<Point2D> & target) {
      double maximum = 0.0;
      for (const auto & point : source) {
        double nearest = std::numeric_limits<double>::infinity();
        for (size_t index = 0; index < target.size(); ++index) {
          const auto & start = target[index];
          const auto & end = target[(index + 1) % target.size()];
          const double dx = end.x - start.x;
          const double dy = end.y - start.y;
          const double length_sq = dx * dx + dy * dy;
          const double projection = length_sq > 1.0e-12 ?
            std::clamp(((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq,
            0.0, 1.0) : 0.0;
          nearest = std::min(nearest, std::hypot(
            point.x - start.x - projection * dx,
            point.y - start.y - projection * dy));
        }
        maximum = std::max(maximum, nearest);
      }
      return maximum;
    };
  return std::max(directed_distance(first, second), directed_distance(second, first));
}
}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__POLYGON_BOUNDARY_DISTANCE_HPP_
