// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>
#include <vector>

#include "r1_collision_guard/guard_logic.hpp"

namespace r1_collision_guard
{

// Returns true only for the strict polygon interior. Points on an edge stay in
// the obstacle set so the collision margin continues to protect the boundary.
inline bool strictlyInsidePolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  constexpr double kBoundaryTolerance = 1.0e-6;
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t index = 0, previous = polygon.size() - 1; index < polygon.size(); previous = index++) {
    const auto & a = polygon[previous];
    const auto & b = polygon[index];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double projection = length_squared > 0.0 ?
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared : 0.0;
    if (projection >= 0.0 && projection <= 1.0) {
      const double edge_x = a.x + projection * dx;
      const double edge_y = a.y + projection * dy;
      if (std::hypot(point.x - edge_x, point.y - edge_y) <= kBoundaryTolerance) {
        return false;
      }
    }

    const bool crosses = (a.y > point.y) != (b.y > point.y);
    if (crosses && point.x < a.x + (point.y - a.y) * dx / dy) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace r1_collision_guard
