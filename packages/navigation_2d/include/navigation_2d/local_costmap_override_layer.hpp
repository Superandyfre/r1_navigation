#ifndef NAVIGATION_2D__LOCAL_COSTMAP_OVERRIDE_LAYER_HPP_
#define NAVIGATION_2D__LOCAL_COSTMAP_OVERRIDE_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace navigation_2d
{

class LocalCostmapOverrideLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void reset() override;
  bool isClearable() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  void mapCallback(OccupancyGrid::SharedPtr map);
  OccupancyGrid::SharedPtr getFreshMap() const;
  static unsigned char occupancyToCost(int8_t occupancy);

  mutable std::mutex map_mutex_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  OccupancyGrid::SharedPtr latest_map_;
  rclcpp::Time latest_map_time_;
  std::string map_topic_;
  double max_age_ = 1.0;
};

}  // namespace navigation_2d

#endif  // NAVIGATION_2D__LOCAL_COSTMAP_OVERRIDE_LAYER_HPP_
