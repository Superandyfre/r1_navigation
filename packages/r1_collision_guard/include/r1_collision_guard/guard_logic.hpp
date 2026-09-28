// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <vector>

namespace r1_collision_guard
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GuardConfig
{
  double max_linear_mps{0.15};
  double max_reverse_mps{0.15};
  double max_angular_radps{0.4};
  double control_period_sec{0.02};
  double reaction_delay_sec{0.10};
  double braking_deceleration_mps2{0.30};
  double braking_angular_deceleration_radps2{0.80};
  double geometry_margin_m{0.02};
  double tracking_error_mps{0.0};
  int binary_search_iterations{10};
  bool allow_reverse{false};
};

struct Decision
{
  enum class State {PASS, SLOW, STOP, FAULT};
  State state{State::FAULT};
  std::string reason{"uninitialized"};
  double linear_x{0.0};
  double angular_z{0.0};
  double safe_scale{0.0};
  double stopping_distance_m{0.0};
  double collision_time_sec{-1.0};
  Point2D collision_point{};
};

class GuardLogic
{
public:
  GuardLogic(GuardConfig config, std::vector<Point2D> footprint);
  void setFootprint(std::vector<Point2D> footprint);
  Decision evaluate(
    double requested_linear_x, double requested_angular_z,
    double actual_linear_x, double actual_angular_z,
    const std::vector<Point2D> & obstacle_points) const;

private:
  struct SimulationResult
  {
    bool safe{false};
    double collision_time_sec{-1.0};
    Point2D collision_point{};
  };

  SimulationResult stoppingTrajectorySafe(
    double scale, double requested_linear_x, double requested_angular_z,
    double actual_linear_x, double actual_angular_z,
    const std::vector<Point2D> & obstacle_points) const;
  bool pointCollides(const Point2D & point, const Pose2D & pose) const;
  static bool pointInsidePolygon(const Point2D & point, const std::vector<Point2D> & polygon);
  static double pointToSegmentDistance(const Point2D & point, const Point2D & a, const Point2D & b);
  static void advance(Pose2D & pose, double linear_x, double angular_z, double dt);
  static double clamp(double value, double low, double high);

  GuardConfig config_;
  std::vector<Point2D> footprint_;
};

const char * stateName(Decision::State state);

}  // namespace r1_collision_guard
