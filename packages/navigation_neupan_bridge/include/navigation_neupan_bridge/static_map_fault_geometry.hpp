#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace navigation_neupan_bridge
{

inline bool deepInsideFootprint(
  const std::vector<double> & polygon, const double x, const double y,
  const double inset)
{
  const size_t count = polygon.size() / 2;
  if (count < 3 || inset < 0.0) {
    return false;
  }
  bool inside = false;
  for (size_t index = 0, previous = count - 1; index < count; previous = index++) {
    const double current_x = polygon[index * 2];
    const double current_y = polygon[index * 2 + 1];
    const double previous_x = polygon[previous * 2];
    const double previous_y = polygon[previous * 2 + 1];
    if ((current_y > y) != (previous_y > y) &&
      x < (previous_x - current_x) * (y - current_y) /
      (previous_y - current_y) + current_x)
    {
      inside = !inside;
    }
  }
  if (!inside) {
    return false;
  }
  for (size_t index = 0; index < count; ++index) {
    const size_t next = (index + 1) % count;
    const double ax = polygon[index * 2];
    const double ay = polygon[index * 2 + 1];
    const double dx = polygon[next * 2] - ax;
    const double dy = polygon[next * 2 + 1] - ay;
    const double length_squared = dx * dx + dy * dy;
    const double ratio = length_squared > 1e-12 ?
      std::clamp(((x - ax) * dx + (y - ay) * dy) / length_squared, 0.0, 1.0) : 0.0;
    if (std::hypot(x - (ax + ratio * dx), y - (ay + ratio * dy)) < inset) {
      return false;
    }
  }
  return true;
}

}  // namespace navigation_neupan_bridge
