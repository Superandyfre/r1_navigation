#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "r1_collision_guard/guard_logic.hpp"

namespace r1_collision_guard
{

inline std::vector<std::vector<Point2D>> decodeMeshComponents(const std::vector<double> & data)
{
  if (data.empty() || data.size() > 20000U) {
    throw std::invalid_argument("mesh component data size invalid");
  }
  std::vector<std::vector<Point2D>> components;
  size_t offset = 0U;
  while (offset < data.size()) {
    const double count_value = data[offset++];
    if (!std::isfinite(count_value) || count_value < 3.0 || count_value > 4096.0 ||
      std::floor(count_value) != count_value || components.size() >= 128U)
    {
      throw std::invalid_argument("mesh component vertex count invalid");
    }
    const size_t count = static_cast<size_t>(count_value);
    if (count > (data.size() - offset) / 2U) {
      throw std::invalid_argument("mesh component data truncated");
    }
    std::vector<Point2D> polygon;
    polygon.reserve(count);
    for (size_t index = 0U; index < count; ++index) {
      const double x = data[offset++];
      const double y = data[offset++];
      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument("mesh component point invalid");
      }
      polygon.push_back({x, y});
    }
    components.push_back(std::move(polygon));
  }
  return components;
}

struct GuardMeshComponents
{
  std::vector<std::vector<Point2D>> raw;
  std::vector<std::vector<Point2D>> expanded;
};

inline GuardMeshComponents decodeGuardMeshComponents(
  const std::vector<double> & data, size_t expected_count)
{
  if (data.empty() || data.size() > 40001U || expected_count == 0U ||
    expected_count > 128U || !std::isfinite(data[0]) ||
    data[0] != static_cast<double>(expected_count))
  {
    throw std::invalid_argument("guard mesh component count invalid");
  }
  GuardMeshComponents result;
  size_t offset = 1U;
  for (auto * group : {&result.raw, &result.expanded}) {
    for (size_t component = 0U; component < expected_count; ++component) {
      if (offset >= data.size()) {
        throw std::invalid_argument("guard mesh component stream truncated");
      }
      const double count_value = data[offset++];
      if (!std::isfinite(count_value) || count_value < 3.0 || count_value > 4096.0 ||
        std::floor(count_value) != count_value ||
        static_cast<size_t>(count_value) > (data.size() - offset) / 2U)
      {
        throw std::invalid_argument("guard mesh polygon invalid");
      }
      std::vector<Point2D> polygon;
      for (size_t index = 0U; index < static_cast<size_t>(count_value); ++index) {
        const double x = data[offset++];
        const double y = data[offset++];
        if (!std::isfinite(x) || !std::isfinite(y)) {
          throw std::invalid_argument("guard mesh point invalid");
        }
        polygon.push_back({x, y});
      }
      group->push_back(std::move(polygon));
    }
  }
  if (offset != data.size()) {
    throw std::invalid_argument("guard mesh component stream has trailing data");
  }
  return result;
}

}  // namespace r1_collision_guard
