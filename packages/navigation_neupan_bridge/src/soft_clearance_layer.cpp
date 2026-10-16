#include "navigation_neupan_bridge/soft_clearance_layer.hpp"

#include <stdexcept>

#include "navigation_neupan_bridge/soft_clearance_cost.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigation_neupan_bridge
{

void SoftClearanceLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("radius_m", rclcpp::ParameterValue(radius_m_));
  declareParameter("maximum_cost", rclcpp::ParameterValue(maximum_cost_));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".radius_m", radius_m_);
  node->get_parameter(name_ + ".maximum_cost", maximum_cost_);
  if (radius_m_ <= 0.0 || maximum_cost_ < 1 || maximum_cost_ > 252) {
    throw std::runtime_error("Invalid soft clearance layer radius or cost");
  }
  current_ = true;
}

void SoftClearanceLayer::reset()
{
  current_ = true;
}

bool SoftClearanceLayer::isClearable()
{
  return false;
}

void SoftClearanceLayer::updateBounds(
  double, double, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  const auto * grid = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, grid->getOriginX());
  *min_y = std::min(*min_y, grid->getOriginY());
  *max_x = std::max(*max_x, grid->getOriginX() + grid->getSizeInMetersX());
  *max_y = std::max(*max_y, grid->getOriginY() + grid->getSizeInMetersY());
}

void SoftClearanceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int, int, int, int)
{
  if (!enabled_) {
    return;
  }
  const unsigned int width = master_grid.getSizeInCellsX();
  const unsigned int height = master_grid.getSizeInCellsY();
  auto * costs = master_grid.getCharMap();
  addSoftClearanceCosts(costs, width, height, master_grid.getResolution(),
    radius_m_, static_cast<unsigned char>(maximum_cost_));
}

}  // namespace navigation_neupan_bridge

PLUGINLIB_EXPORT_CLASS(
  navigation_neupan_bridge::SoftClearanceLayer,
  nav2_costmap_2d::Layer)
