"""Scan-based speed reference for the R1 NeuPAN worker."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class MotionPolicyConfig:
    max_speed_mps: float = 0.20
    braking_mps2: float = 0.20
    reaction_sec: float = 0.30
    stop_margin_m: float = 0.15
    preview_m: float = 0.90
    sample_step_m: float = 0.05
    front_m: float = 0.341
    rear_m: float = 0.341
    left_m: float = 0.266
    right_m: float = 0.266
    steps: int = 6
    step_time_sec: float = 0.25

    def __post_init__(self) -> None:
        positive = (
            self.max_speed_mps, self.braking_mps2, self.reaction_sec,
            self.preview_m, self.sample_step_m, self.front_m, self.rear_m,
            self.left_m, self.right_m, self.step_time_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("motion policy distances, rates and extents must be positive")
        if not math.isfinite(self.stop_margin_m) or self.stop_margin_m < 0.0:
            raise ValueError("stop_margin_m must be finite and nonnegative")
        if self.steps < 1:
            raise ValueError("steps must be positive")


@dataclass(frozen=True)
class MotionDecision:
    speed_profile: np.ndarray
    free_distance_m: float
    raw_distance_m: float
    planner_distance_m: float
    route_blocked: bool
    reason: str


def _points(value: np.ndarray) -> np.ndarray:
    points = np.asarray(value, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] != 2 or not np.isfinite(points).all():
        raise ValueError("obstacle points must be a finite 2xN array")
    return points


def _sample_route(state: np.ndarray, path: np.ndarray, config: MotionPolicyConfig) -> tuple[np.ndarray, np.ndarray]:
    pose = np.asarray(state, dtype=np.float64).reshape(-1)
    route = np.asarray(path, dtype=np.float64)
    if pose.size < 3 or route.ndim != 2 or route.shape[0] < 3 or route.shape[1] < 1:
        raise ValueError("state and path must contain finite x, y and yaw")
    if not np.isfinite(pose[:3]).all() or not np.isfinite(route[:3]).all():
        raise ValueError("state and path must be finite")

    xy = route[:2].T
    arc = np.r_[0.0, np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))]
    closest = int(np.argmin(np.sum((xy - pose[:2]) ** 2, axis=1)))
    forward_arc = arc[closest:] - arc[closest]
    unique_arc, unique_indices = np.unique(forward_arc, return_index=True)
    yaw = np.unwrap(route[2, closest:][unique_indices])
    sample_end = min(config.preview_m, float(unique_arc[-1]))
    distances = np.arange(0.0, sample_end, config.sample_step_m)
    distances = np.unique(np.r_[0.0, distances, sample_end])
    if unique_arc.size == 1:
        samples = np.repeat(pose[:3, None], distances.size, axis=1)
    else:
        samples = np.vstack([
            np.interp(distances, unique_arc, xy[closest + unique_indices, 0]),
            np.interp(distances, unique_arc, xy[closest + unique_indices, 1]),
            np.interp(distances, unique_arc, yaw),
        ])
        samples[:, 0] = pose[:3]
    return samples, distances


def first_swept_collision_distance(
    state: np.ndarray, path: np.ndarray, points: np.ndarray,
    config: MotionPolicyConfig,
) -> float:
    """Distance travelled until the measured points enter the moving base box."""
    cloud = _points(points)
    if cloud.shape[1] == 0:
        return math.inf
    samples, distances = _sample_route(state, path, config)
    def collides(pose: np.ndarray) -> bool:
        x, y, yaw = pose
        dx, dy = cloud[0] - x, cloud[1] - y
        longitudinal = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return bool(np.any(
            (longitudinal >= -config.rear_m)
            & (longitudinal <= config.front_m)
            & (lateral >= -config.right_m)
            & (lateral <= config.left_m)
        ))

    for index, distance in enumerate(distances):
        if collides(samples[:, index]):
            if index == 0:
                return 0.0
            lower = float(distances[index - 1])
            upper = float(distance)
            start = samples[:, index - 1]
            end = samples[:, index]
            # Refine only the collision interval, keeping the normal scan cheap.
            for _ in range(5):
                midpoint = (lower + upper) / 2.0
                fraction = (midpoint - distances[index - 1]) / (
                    distance - distances[index - 1]
                )
                if collides(start + fraction * (end - start)):
                    upper = midpoint
                else:
                    lower = midpoint
            return upper
    return math.inf


def first_command_collision_distance(
    state: np.ndarray, linear_x: float, angular_z: float,
    points: np.ndarray, config: MotionPolicyConfig,
) -> float:
    """Sweep a constant-twist command in its signed travel direction."""
    if not math.isfinite(linear_x) or not math.isfinite(angular_z):
        raise ValueError("command velocity must be finite")
    pose = np.asarray(state, dtype=np.float64).reshape(-1)
    if pose.size < 3 or not np.isfinite(pose[:3]).all():
        raise ValueError("state must contain finite x, y and yaw")
    if abs(linear_x) < 1.0e-6:
        return first_rotation_collision_distance(
            pose, angular_z * config.steps * config.step_time_sec, points, config
        )
    travel = np.arange(0.0, config.preview_m, config.sample_step_m)
    travel = np.unique(np.r_[travel, config.preview_m])
    yaw = pose[2] + (angular_z / abs(linear_x)) * travel
    if abs(angular_z) < 1.0e-6:
        x = pose[0] + np.sign(linear_x) * travel * math.cos(pose[2])
        y = pose[1] + np.sign(linear_x) * travel * math.sin(pose[2])
    else:
        x = pose[0] + (linear_x / angular_z) * (np.sin(yaw) - math.sin(pose[2]))
        y = pose[1] - (linear_x / angular_z) * (np.cos(yaw) - math.cos(pose[2]))
    return first_swept_collision_distance(pose, np.vstack((x, y, yaw)), points, config)


def first_rotation_collision_distance(
    state: np.ndarray, yaw_delta: float, points: np.ndarray,
    config: MotionPolicyConfig,
) -> float:
    cloud = _points(points)
    if cloud.shape[1] == 0:
        return math.inf
    pose = np.asarray(state, dtype=np.float64).reshape(-1)
    if pose.size < 3 or not np.isfinite(pose[:3]).all() or not math.isfinite(yaw_delta):
        raise ValueError("rotation state and angle must be finite")
    count = max(1, math.ceil(abs(yaw_delta) / 0.04))
    dx, dy = cloud[0] - pose[0], cloud[1] - pose[1]
    for yaw in np.linspace(pose[2], pose[2] + yaw_delta, count + 1):
        longitudinal = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if np.any(
            (longitudinal >= -config.rear_m)
            & (longitudinal <= config.front_m)
            & (lateral >= -config.right_m)
            & (lateral <= config.left_m)
        ):
            return 0.0
    return math.inf


def _speed_limit(remaining_m: float, config: MotionPolicyConfig) -> float:
    if math.isinf(remaining_m):
        return config.max_speed_mps
    if remaining_m <= 0.0:
        return 0.0
    a = config.braking_mps2
    return min(
        config.max_speed_mps,
        max(0.0, math.sqrt((a * config.reaction_sec) ** 2 + 2.0 * a * remaining_m)
            - a * config.reaction_sec),
    )


def plan_obstacle_speed(
    state: np.ndarray, path: np.ndarray, raw_points: np.ndarray,
    planner_points: np.ndarray, config: MotionPolicyConfig,
) -> MotionDecision:
    raw_distance = first_swept_collision_distance(state, path, raw_points, config)
    planner_distance = first_swept_collision_distance(state, path, planner_points, config)
    free_distance = min(raw_distance, planner_distance)
    progress = 0.0
    profile = np.empty(config.steps, dtype=np.float64)
    for index in range(config.steps):
        remaining = free_distance - config.stop_margin_m - progress
        speed = _speed_limit(remaining, config)
        if math.isfinite(remaining):
            speed = min(speed, max(0.0, remaining / config.step_time_sec))
        profile[index] = speed
        progress += speed * config.step_time_sec
    blocked = math.isfinite(free_distance)
    reason = "raw_obstacle" if raw_distance <= planner_distance else "projected_obstacle"
    return MotionDecision(
        profile, free_distance, raw_distance, planner_distance, blocked,
        reason if blocked else "route_clear",
    )
