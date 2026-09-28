#ifndef NAVIGATION_2D__CLEARANCE_CRITIC_HPP_
#define NAVIGATION_2D__CLEARANCE_CRITIC_HPP_

#include <vector>

#include "dwb_core/trajectory_critic.hpp"

namespace navigation_2d
{

class ClearanceCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

  void addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>> & cost_channels) override;

private:
  double clearanceAt(const geometry_msgs::msg::Pose2D & pose) const;
  double penaltyForClearance(double clearance) const;
  void buildDistanceField();

  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::vector<double> obstacle_distances_;
  double preferred_clearance_ = 0.15;
  double penalty_exponent_ = 2.0;
  double footprint_radius_ = 0.0;
  bool has_obstacles_ = false;
};

}  // namespace navigation_2d

#endif  // NAVIGATION_2D__CLEARANCE_CRITIC_HPP_
