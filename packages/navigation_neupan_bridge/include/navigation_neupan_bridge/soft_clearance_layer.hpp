#ifndef NAVIGATION_NEUPAN_BRIDGE__SOFT_CLEARANCE_LAYER_HPP_
#define NAVIGATION_NEUPAN_BRIDGE__SOFT_CLEARANCE_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"

namespace navigation_neupan_bridge
{

// Adds traversable cost outside the standard inflation radius. The live
// footprint and lethal cells remain the sole global collision boundary.
class SoftClearanceLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void reset() override;
  bool isClearable() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  double radius_m_{0.50};
  int maximum_cost_{180};
};

}  // namespace navigation_neupan_bridge

#endif  // NAVIGATION_NEUPAN_BRIDGE__SOFT_CLEARANCE_LAYER_HPP_
