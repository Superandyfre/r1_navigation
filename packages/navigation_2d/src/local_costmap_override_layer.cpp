#include "navigation_2d/local_costmap_override_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigation_2d
{

void LocalCostmapOverrideLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_topic", rclcpp::ParameterValue("/local_costmap/costmap"));
  declareParameter("max_age", rclcpp::ParameterValue(max_age_));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".map_topic", map_topic_);
  node->get_parameter(name_ + ".max_age", max_age_);
  if (max_age_ <= 0.0) {
    throw std::runtime_error("Local costmap override max_age must be positive");
  }

  rclcpp::QoS map_qos(1);
  map_qos.reliable().transient_local();
  map_sub_ = node->create_subscription<OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&LocalCostmapOverrideLayer::mapCallback, this, std::placeholders::_1));
  current_ = false;
}

void LocalCostmapOverrideLayer::reset()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_.reset();
  current_ = false;
}

bool LocalCostmapOverrideLayer::isClearable()
{
  return false;
}

void LocalCostmapOverrideLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  const auto map = getFreshMap();
  current_ = map != nullptr;
  if (!enabled_ || !map) {
    return;
  }

  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double end_x = origin_x + map->info.width * map->info.resolution;
  const double end_y = origin_y + map->info.height * map->info.resolution;
  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, end_x);
  *max_y = std::max(*max_y, end_y);
}

void LocalCostmapOverrideLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  const auto map = getFreshMap();
  if (!map || map->header.frame_id != layered_costmap_->getGlobalFrameID() ||
    map->info.resolution <= 0.0 ||
    map->data.size() != static_cast<std::size_t>(map->info.width) * map->info.height)
  {
    return;
  }

  const int start_x = std::max(0, min_i);
  const int start_y = std::max(0, min_j);
  const int end_x = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  const int end_y = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);
  for (int master_y = start_y; master_y < end_y; ++master_y) {
    for (int master_x = start_x; master_x < end_x; ++master_x) {
      double world_x = 0.0;
      double world_y = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(master_x), static_cast<unsigned int>(master_y), world_x, world_y);
      const int source_x = static_cast<int>(std::floor(
          (world_x - map->info.origin.position.x) / map->info.resolution));
      const int source_y = static_cast<int>(std::floor(
          (world_y - map->info.origin.position.y) / map->info.resolution));
      if (source_x < 0 || source_y < 0 || source_x >= static_cast<int>(map->info.width) ||
        source_y >= static_cast<int>(map->info.height))
      {
        continue;
      }

      const auto source_index = static_cast<std::size_t>(source_y) * map->info.width + source_x;
      // Only raise costs. The local grid can be empty or blind (stale sensor
      // sources); letting it write free space here would erase static lethal
      // cells and let the global planner route through real obstacles.
      const auto override_cost = occupancyToCost(map->data[source_index]);
      const auto existing_cost = master_grid.getCost(
        static_cast<unsigned int>(master_x), static_cast<unsigned int>(master_y));
      if (override_cost > existing_cost) {
        master_grid.setCost(
          static_cast<unsigned int>(master_x), static_cast<unsigned int>(master_y),
          override_cost);
      }
    }
  }
}

void LocalCostmapOverrideLayer::mapCallback(OccupancyGrid::SharedPtr map)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_ = std::move(map);
  latest_map_time_ = clock_->now();
}

LocalCostmapOverrideLayer::OccupancyGrid::SharedPtr LocalCostmapOverrideLayer::getFreshMap() const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_map_ || (clock_->now() - latest_map_time_).seconds() > max_age_) {
    return nullptr;
  }
  return latest_map_;
}

unsigned char LocalCostmapOverrideLayer::occupancyToCost(const int8_t occupancy)
{
  if (occupancy < 0) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  if (occupancy >= 100) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return static_cast<unsigned char>(occupancy * nav2_costmap_2d::LETHAL_OBSTACLE / 100);
}

}  // namespace navigation_2d

PLUGINLIB_EXPORT_CLASS(navigation_2d::LocalCostmapOverrideLayer, nav2_costmap_2d::Layer)
