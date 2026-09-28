#include "navigation_neupan_bridge/current_footprint_clear_layer.hpp"

#include <algorithm>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigation_neupan_bridge
{

void CurrentFootprintClearLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("quantization_margin_m", rclcpp::ParameterValue(quantization_margin_m_));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".quantization_margin_m", quantization_margin_m_);
  if (quantization_margin_m_ < 0.0 || quantization_margin_m_ > 0.05) {
    throw std::runtime_error(
            "Current footprint clearing quantization_margin_m must be within [0.0, 0.05]");
  }
  current_ = true;
}

void CurrentFootprintClearLayer::reset()
{
  current_footprint_.clear();
  current_ = true;
}

bool CurrentFootprintClearLayer::isClearable()
{
  return false;
}

void CurrentFootprintClearLayer::extendBounds(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double * min_x, double * min_y, double * max_x, double * max_y) const
{
  for (const auto & point : footprint) {
    *min_x = std::min(*min_x, point.x);
    *min_y = std::min(*min_y, point.y);
    *max_x = std::max(*max_x, point.x);
    *max_y = std::max(*max_y, point.y);
  }
}

void CurrentFootprintClearLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  current_ = true;
  if (!enabled_) {
    return;
  }

  // Include the previous location so StaticLayer restores it before this
  // layer clears the robot's new location.
  extendBounds(current_footprint_, min_x, min_y, max_x, max_y);

  auto padded_footprint = getFootprint();
  nav2_costmap_2d::padFootprint(padded_footprint, quantization_margin_m_);
  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, padded_footprint, current_footprint_);
  extendBounds(current_footprint_, min_x, min_y, max_x, max_y);
}

void CurrentFootprintClearLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int, int, int, int)
{
  if (enabled_ && current_footprint_.size() >= 3) {
    master_grid.setConvexPolygonCost(current_footprint_, nav2_costmap_2d::FREE_SPACE);
  }
}

}  // namespace navigation_neupan_bridge

PLUGINLIB_EXPORT_CLASS(
  navigation_neupan_bridge::CurrentFootprintClearLayer,
  nav2_costmap_2d::Layer)
