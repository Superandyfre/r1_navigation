#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <vector>

#include "navigation_neupan_bridge/soft_clearance_cost.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"

using navigation_neupan_bridge::softClearanceCost;
using navigation_neupan_bridge::squaredDistanceToLethal;
using navigation_neupan_bridge::addSoftClearanceCosts;

TEST(SoftClearanceCost, ExactEuclideanDistanceAndUnknownNotAnObstacle)
{
  std::vector<unsigned char> costs(7U * 5U, 0U);
  costs[2U * 7U + 1U] = 254U;
  costs[2U * 7U + 6U] = 255U;
  const auto distance = squaredDistanceToLethal(costs.data(), 7U, 5U, 254U);
  EXPECT_DOUBLE_EQ(distance[2U * 7U + 1U], 0.0);
  EXPECT_DOUBLE_EQ(distance[3U * 7U + 3U], 5.0);
  EXPECT_DOUBLE_EQ(distance[2U * 7U + 6U], 25.0);
}

TEST(SoftClearanceCost, OpenSidePreferredWithoutBlockingPassage)
{
  const auto near_wall = softClearanceCost(9.0, 0.05, 0.50, 180U);
  const auto farther_wall = softClearanceCost(64.0, 0.05, 0.50, 180U);
  EXPECT_GT(near_wall, farther_wall);
  EXPECT_GT(farther_wall, 0U);
  EXPECT_LT(near_wall, 253U);
  EXPECT_EQ(softClearanceCost(100.0, 0.05, 0.50, 180U), 0U);
}

TEST(SoftClearanceCost, SoftCostsPreserveHardAndUnknownCells)
{
  std::vector<unsigned char> costs(11U * 5U, 0U);
  costs[2U * 11U] = 254U;
  costs[2U * 11U + 10U] = 254U;
  costs[2U * 11U + 5U] = 255U;
  costs[1U * 11U + 5U] = 253U;
  addSoftClearanceCosts(costs.data(), 11U, 5U, 0.10, 0.50, 180U);
  EXPECT_EQ(costs[2U * 11U], 254U);
  EXPECT_EQ(costs[2U * 11U + 10U], 254U);
  EXPECT_EQ(costs[2U * 11U + 5U], 255U);
  EXPECT_EQ(costs[1U * 11U + 5U], 253U);
  EXPECT_GT(costs[2U * 11U + 1U], costs[2U * 11U + 4U]);
  EXPECT_GT(costs[2U * 11U + 4U], 0U);
  EXPECT_LT(costs[2U * 11U + 4U], 253U);
}

TEST(SoftClearanceCost, MatchesBruteForceWithSeparatedObstacles)
{
  constexpr unsigned int width = 9U;
  constexpr unsigned int height = 7U;
  std::vector<unsigned char> costs(width * height, 0U);
  const std::vector<std::pair<int, int>> seeds{{1, 1}, {7, 5}, {4, 3}};
  for (const auto & [x, y] : seeds) {
    costs[y * width + x] = 254U;
  }
  const auto distance = squaredDistanceToLethal(costs.data(), width, height, 254U);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      double expected = 1.0e12;
      for (const auto & [sx, sy] : seeds) {
        expected = std::min(expected, static_cast<double>(
            (static_cast<int>(x) - sx) * (static_cast<int>(x) - sx) +
            (static_cast<int>(y) - sy) * (static_cast<int>(y) - sy)));
      }
      EXPECT_DOUBLE_EQ(distance[y * width + x], expected);
    }
  }
}

TEST(SoftClearanceCost, FullMapComputationFitsCostmapUpdatePeriod)
{
  constexpr unsigned int width = 631U;
  constexpr unsigned int height = 436U;
  std::vector<unsigned char> costs(width * height, 0U);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if ((x * 17U + y * 23U) % 10U == 0U) {
        costs[y * width + x] = 254U;
      }
    }
  }
  const auto started = std::chrono::steady_clock::now();
  const auto distance = squaredDistanceToLethal(costs.data(), width, height, 254U);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  std::cout << "full_map_distance_transform_ms=" << elapsed_ms << std::endl;
  EXPECT_EQ(distance.size(), costs.size());
  EXPECT_LT(elapsed_ms, 100.0);
}

TEST(SoftClearanceCost, PluginIsDiscoverable)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  EXPECT_TRUE(loader.isClassAvailable("navigation_neupan_bridge::SoftClearanceLayer"));
  EXPECT_NE(loader.createSharedInstance("navigation_neupan_bridge::SoftClearanceLayer"), nullptr);
}
