#include "navigation_2d/clearance_critic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigation_2d
{

namespace
{

struct DistanceCell
{
  double distance;
  unsigned int index;

  bool operator>(const DistanceCell & other) const
  {
    return distance > other.distance;
  }
};

}  // namespace

void ClearanceCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".preferred_clearance",
    rclcpp::ParameterValue(preferred_clearance_));
  nav2_util::declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".penalty_exponent",
    rclcpp::ParameterValue(penalty_exponent_));
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".preferred_clearance", preferred_clearance_);
  node->get_parameter(
    dwb_plugin_name_ + "." + name_ + ".penalty_exponent", penalty_exponent_);

  if (preferred_clearance_ <= 0.0 || penalty_exponent_ <= 0.0) {
    throw std::runtime_error("Clearance critic parameters must be positive");
  }
}

bool ClearanceCritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  footprint_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  buildDistanceField();
  return true;
}

double ClearanceCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!has_obstacles_ || trajectory.poses.empty()) {
    return 0.0;
  }

  double worst_penalty = 0.0;
  for (const auto & pose : trajectory.poses) {
    worst_penalty = std::max(worst_penalty, penaltyForClearance(clearanceAt(pose)));
  }
  return worst_penalty;
}

void ClearanceCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  std::vector<float> values(obstacle_distances_.size(), 0.0F);
  for (std::size_t index = 0; index < obstacle_distances_.size(); ++index) {
    const double clearance = std::max(0.0, obstacle_distances_[index] - footprint_radius_);
    values[index] = static_cast<float>(penaltyForClearance(clearance));
  }
  cost_channels.emplace_back(name_, std::move(values));
}

double ClearanceCritic::clearanceAt(const geometry_msgs::msg::Pose2D & pose) const
{
  unsigned int cell_x = 0;
  unsigned int cell_y = 0;
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    return preferred_clearance_;
  }

  const double center_distance = obstacle_distances_[costmap_->getIndex(cell_x, cell_y)];
  return std::max(0.0, center_distance - footprint_radius_);
}

double ClearanceCritic::penaltyForClearance(const double clearance) const
{
  if (clearance >= preferred_clearance_) {
    return 0.0;
  }

  const double normalized_deficit = (preferred_clearance_ - clearance) / preferred_clearance_;
  return std::pow(normalized_deficit, penalty_exponent_);
}

void ClearanceCritic::buildDistanceField()
{
  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const std::size_t cell_count = static_cast<std::size_t>(width) * height;
  const double infinity = std::numeric_limits<double>::infinity();
  const double resolution = costmap_->getResolution();

  obstacle_distances_.assign(cell_count, infinity);
  std::priority_queue<DistanceCell, std::vector<DistanceCell>, std::greater<DistanceCell>> frontier;
  has_obstacles_ = false;

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const unsigned int index = costmap_->getIndex(x, y);
      if (costmap_->getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        obstacle_distances_[index] = 0.0;
        frontier.push({0.0, index});
        has_obstacles_ = true;
      }
    }
  }

  constexpr std::array<std::pair<int, int>, 8> kNeighbors{{
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1},
  }};

  while (!frontier.empty()) {
    const DistanceCell current = frontier.top();
    frontier.pop();
    if (current.distance != obstacle_distances_[current.index]) {
      continue;
    }

    const int x = static_cast<int>(current.index % width);
    const int y = static_cast<int>(current.index / width);
    for (const auto & offset : kNeighbors) {
      const int next_x = x + offset.first;
      const int next_y = y + offset.second;
      if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(width) ||
        next_y >= static_cast<int>(height))
      {
        continue;
      }

      const double step =
        (offset.first == 0 || offset.second == 0) ? resolution : resolution * std::sqrt(2.0);
      const unsigned int next_index = costmap_->getIndex(
        static_cast<unsigned int>(next_x), static_cast<unsigned int>(next_y));
      const double next_distance = current.distance + step;
      if (next_distance < obstacle_distances_[next_index]) {
        obstacle_distances_[next_index] = next_distance;
        frontier.push({next_distance, next_index});
      }
    }
  }
}

}  // namespace navigation_2d

PLUGINLIB_EXPORT_CLASS(navigation_2d::ClearanceCritic, dwb_core::TrajectoryCritic)
