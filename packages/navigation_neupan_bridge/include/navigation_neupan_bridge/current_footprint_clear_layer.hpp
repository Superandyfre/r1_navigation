#ifndef NAVIGATION_NEUPAN_BRIDGE__CURRENT_FOOTPRINT_CLEAR_LAYER_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__CURRENT_FOOTPRINT_CLEAR_LAYER_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace navigation_neupan_bridge
{

// Clears only the robot's current, padded footprint from the planning map.
// It is ordered after inflation so static obstacle costs cannot be written back
// into the space physically occupied by the robot.
class CurrentFootprintClearLayer : public nav2_costmap_2d::Layer
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
  void extendBounds(
    const std::vector<geometry_msgs::msg::Point> & footprint,
    double * min_x, double * min_y, double * max_x, double * max_y) const;

  double quantization_margin_m_{0.05};
  std::vector<geometry_msgs::msg::Point> current_footprint_;
};

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__CURRENT_FOOTPRINT_CLEAR_LAYER_HPP_
