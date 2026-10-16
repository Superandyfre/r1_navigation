#ifndef NAVIGATION_NEUPAN_BRIDGE__SOFT_CLEARANCE_COST_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__SOFT_CLEARANCE_COST_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace navigation_neupan_bridge
{

constexpr double kUnreachableDistanceSquared = 1.0e12;

inline void distanceTransform1D(
  const std::vector<double> & input, std::vector<double> & output)
{
  const int count = static_cast<int>(input.size());
  output.assign(input.size(), kUnreachableDistanceSquared);
  std::vector<int> sites;
  std::vector<double> boundaries;
  sites.reserve(input.size());
  boundaries.reserve(input.size() + 1U);
  for (int position = 0; position < count; ++position) {
    if (input[position] >= kUnreachableDistanceSquared) {
      continue;
    }
    double crossing = -std::numeric_limits<double>::infinity();
    while (!sites.empty()) {
      const int site = sites.back();
      crossing = (input[position] + position * position - input[site] - site * site) /
        (2.0 * (position - site));
      if (crossing > boundaries.back()) {
        break;
      }
      sites.pop_back();
      boundaries.pop_back();
    }
    if (sites.empty()) {
      crossing = -std::numeric_limits<double>::infinity();
    }
    sites.push_back(position);
    boundaries.push_back(crossing);
  }
  if (sites.empty()) {
    return;
  }
  size_t active = 0U;
  for (int position = 0; position < count; ++position) {
    while (active + 1U < sites.size() && boundaries[active + 1U] <= position) {
      ++active;
    }
    const double delta = position - sites[active];
    output[position] = delta * delta + input[sites[active]];
  }
}

inline std::vector<double> squaredDistanceToLethal(
  const unsigned char * costs, unsigned int width, unsigned int height,
  unsigned char lethal_cost)
{
  std::vector<double> horizontal(static_cast<size_t>(width) * height);
  std::vector<double> distances(horizontal.size());
  std::vector<double> input(std::max(width, height));
  std::vector<double> output;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      input[x] = costs[static_cast<size_t>(y) * width + x] == lethal_cost ?
        0.0 : kUnreachableDistanceSquared;
    }
    distanceTransform1D(
      std::vector<double>(input.begin(), input.begin() + width), output);
    std::copy(output.begin(), output.end(), horizontal.begin() + static_cast<size_t>(y) * width);
  }
  for (unsigned int x = 0; x < width; ++x) {
    for (unsigned int y = 0; y < height; ++y) {
      input[y] = horizontal[static_cast<size_t>(y) * width + x];
    }
    distanceTransform1D(
      std::vector<double>(input.begin(), input.begin() + height), output);
    for (unsigned int y = 0; y < height; ++y) {
      distances[static_cast<size_t>(y) * width + x] = output[y];
    }
  }
  return distances;
}

inline unsigned char softClearanceCost(
  double squared_distance_cells, double resolution_m,
  double radius_m, unsigned char maximum_cost)
{
  const double distance_m = std::sqrt(squared_distance_cells) * resolution_m;
  if (distance_m >= radius_m || maximum_cost == 0U) {
    return 0U;
  }
  return static_cast<unsigned char>(std::lround(
      maximum_cost * (1.0 - distance_m / radius_m)));
}

inline void addSoftClearanceCosts(
  unsigned char * costs, unsigned int width, unsigned int height,
  double resolution_m, double radius_m, unsigned char maximum_cost)
{
  const auto distances = squaredDistanceToLethal(
    costs, width, height, nav2_costmap_2d::LETHAL_OBSTACLE);
  const size_t count = static_cast<size_t>(width) * height;
  for (size_t index = 0; index < count; ++index) {
    if (costs[index] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      continue;
    }
    costs[index] = std::max(costs[index], softClearanceCost(
        distances[index], resolution_m, radius_m, maximum_cost));
  }
}

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__SOFT_CLEARANCE_COST_HPP_
