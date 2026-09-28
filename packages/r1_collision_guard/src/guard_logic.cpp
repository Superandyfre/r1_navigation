// Copyright 2026 R1 navigation
// SPDX-License-Identifier: Apache-2.0

#include "r1_collision_guard/guard_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace r1_collision_guard
{

namespace
{
constexpr double kEpsilon = 1.0e-9;
}

GuardLogic::GuardLogic(GuardConfig config, std::vector<Point2D> footprint)
: config_(config), footprint_(std::move(footprint))
{
  if (footprint_.size() < 3 || config_.control_period_sec <= 0.0 ||
    config_.braking_deceleration_mps2 <= 0.0 ||
    config_.braking_angular_deceleration_radps2 <= 0.0)
  {
    throw std::invalid_argument("invalid collision guard geometry or braking configuration");
  }
}

void GuardLogic::setFootprint(std::vector<Point2D> footprint)
{
  if (footprint.size() < 3) {
    throw std::invalid_argument("collision guard footprint must have at least three points");
  }
  footprint_ = std::move(footprint);
}

double GuardLogic::clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

void GuardLogic::advance(Pose2D & pose, double linear_x, double angular_z, double dt)
{
  pose.x += linear_x * std::cos(pose.yaw) * dt;
  pose.y += linear_x * std::sin(pose.yaw) * dt;
  pose.yaw += angular_z * dt;
}

bool GuardLogic::pointInsidePolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Point2D & a = polygon[i];
    const Point2D & b = polygon[j];
    const bool crosses = ((a.y > point.y) != (b.y > point.y)) &&
      (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y + kEpsilon) + a.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

double GuardLogic::pointToSegmentDistance(const Point2D & point, const Point2D & a, const Point2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared < kEpsilon) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  const double projection = clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) /
    length_squared, 0.0, 1.0);
  return std::hypot(point.x - (a.x + projection * dx), point.y - (a.y + projection * dy));
}

bool GuardLogic::pointCollides(const Point2D & point, const Pose2D & pose) const
{
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const Point2D local{cosine * dx + sine * dy, -sine * dx + cosine * dy};
  if (pointInsidePolygon(local, footprint_)) {
    return true;
  }
  for (size_t i = 0; i < footprint_.size(); ++i) {
    if (pointToSegmentDistance(local, footprint_[i], footprint_[(i + 1) % footprint_.size()]) <=
      config_.geometry_margin_m)
    {
      return true;
    }
  }
  return false;
}

GuardLogic::SimulationResult GuardLogic::stoppingTrajectorySafe(
  double scale, double requested_linear_x, double requested_angular_z,
  double actual_linear_x, double actual_angular_z,
  const std::vector<Point2D> & obstacle_points) const
{
  const double requested_v = scale * clamp(
    requested_linear_x, -config_.max_reverse_mps, config_.max_linear_mps);
  const double requested_w = scale * clamp(
    requested_angular_z, -config_.max_angular_radps, config_.max_angular_radps);
  const double actual_v = clamp(
    actual_linear_x, -config_.max_reverse_mps, config_.max_linear_mps);
  const double actual_w = clamp(actual_angular_z, -config_.max_angular_radps, config_.max_angular_radps);
  const double brake_direction = std::abs(actual_v) >= std::abs(requested_v) ? actual_v : requested_v;
  const double brake_speed_limit = brake_direction < 0.0 ?
    config_.max_reverse_mps : config_.max_linear_mps;
  const double brake_v_initial = std::copysign(
    std::min(
      brake_speed_limit,
      std::max(std::abs(actual_v), std::abs(requested_v)) + config_.tracking_error_mps),
    brake_direction);
  double brake_w = std::abs(requested_w) >= std::abs(actual_w) ? requested_w : actual_w;
  double brake_v = brake_v_initial;
  Pose2D pose{};
  double elapsed = 0.0;

  const auto collides = [&]() -> SimulationResult {
      for (const auto & point : obstacle_points) {
        if (pointCollides(point, pose)) {
          return {false, elapsed, point};
        }
      }
      return {true, -1.0, {}};
    };

  while (elapsed < config_.reaction_delay_sec) {
    const auto result = collides();
    if (!result.safe) {
      return result;
    }
    const double dt = std::min(config_.control_period_sec, config_.reaction_delay_sec - elapsed);
    advance(pose, actual_v, actual_w, dt);
    elapsed += dt;
  }

  // This rollout follows the proven Collision Monitor footprint-at-pose method,
  // but decelerates rather than projecting a constant command indefinitely.
  while (std::abs(brake_v) > kEpsilon || std::abs(brake_w) > kEpsilon) {
    const auto result = collides();
    if (!result.safe) {
      return result;
    }
    advance(pose, brake_v, brake_w, config_.control_period_sec);
    elapsed += config_.control_period_sec;
    const double v_magnitude = std::max(
      0.0, std::abs(brake_v) - config_.braking_deceleration_mps2 * config_.control_period_sec);
    brake_v = std::copysign(v_magnitude, brake_v);
    const double w_magnitude = std::max(
      0.0, std::abs(brake_w) - config_.braking_angular_deceleration_radps2 * config_.control_period_sec);
    brake_w = std::copysign(w_magnitude, brake_w);
  }
  return collides();
}

Decision GuardLogic::evaluate(
  double requested_linear_x, double requested_angular_z,
  double actual_linear_x, double actual_angular_z,
  const std::vector<Point2D> & obstacle_points) const
{
  Decision decision;
  if (!std::isfinite(requested_linear_x) || !std::isfinite(requested_angular_z) ||
    !std::isfinite(actual_linear_x) || !std::isfinite(actual_angular_z))
  {
    decision.reason = "nonfinite_velocity";
    return decision;
  }
  if (!config_.allow_reverse && requested_linear_x < 0.0) {
    decision.reason = "reverse_command_rejected";
    return decision;
  }
  requested_linear_x = clamp(
    requested_linear_x, -config_.max_reverse_mps, config_.max_linear_mps);
  requested_angular_z = clamp(requested_angular_z, -config_.max_angular_radps, config_.max_angular_radps);
  actual_linear_x = clamp(actual_linear_x, -config_.max_reverse_mps, config_.max_linear_mps);
  actual_angular_z = clamp(actual_angular_z, -config_.max_angular_radps, config_.max_angular_radps);
  const double v_for_distance = std::max(std::abs(actual_linear_x), std::abs(requested_linear_x)) +
    config_.tracking_error_mps;
  decision.stopping_distance_m = v_for_distance * config_.reaction_delay_sec +
    (v_for_distance * v_for_distance) / (2.0 * config_.braking_deceleration_mps2) +
    config_.geometry_margin_m;

  const auto full_scale = stoppingTrajectorySafe(
    1.0, requested_linear_x, requested_angular_z, actual_linear_x, actual_angular_z, obstacle_points);
  if (full_scale.safe) {
    decision.state = Decision::State::PASS;
    decision.reason = "stopping_trajectory_clear";
    decision.linear_x = requested_linear_x;
    decision.angular_z = requested_angular_z;
    decision.safe_scale = 1.0;
    return decision;
  }

  const auto zero_scale = stoppingTrajectorySafe(
    0.0, requested_linear_x, requested_angular_z, actual_linear_x, actual_angular_z, obstacle_points);
  if (!zero_scale.safe) {
    decision.state = Decision::State::STOP;
    decision.reason = "immediate_braking_trajectory_blocked";
    decision.collision_time_sec = zero_scale.collision_time_sec;
    decision.collision_point = zero_scale.collision_point;
    return decision;
  }

  double low = 0.0;
  double high = 1.0;
  for (int iteration = 0; iteration < config_.binary_search_iterations; ++iteration) {
    const double middle = (low + high) * 0.5;
    if (stoppingTrajectorySafe(
        middle, requested_linear_x, requested_angular_z,
        actual_linear_x, actual_angular_z, obstacle_points).safe)
    {
      low = middle;
    } else {
      high = middle;
    }
  }
  decision.state = Decision::State::SLOW;
  decision.reason = "braking_speed_limited";
  decision.linear_x = low * requested_linear_x;
  decision.angular_z = low * requested_angular_z;
  decision.safe_scale = low;
  decision.collision_time_sec = full_scale.collision_time_sec;
  decision.collision_point = full_scale.collision_point;
  return decision;
}

const char * stateName(Decision::State state)
{
  switch (state) {
    case Decision::State::PASS: return "PASS";
    case Decision::State::SLOW: return "SLOW";
    case Decision::State::STOP: return "STOP";
    case Decision::State::FAULT: return "FAULT";
  }
  return "FAULT";
}

}  // namespace r1_collision_guard
