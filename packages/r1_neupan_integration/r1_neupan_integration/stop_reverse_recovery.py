"""Bound a NeuPAN reverse proposal made while its normal action is stopped."""

from __future__ import annotations

import math

import numpy as np


def stopped_reverse_velocity(optimized_velocity: np.ndarray) -> tuple[float, float] | None:
    velocity = np.asarray(optimized_velocity, dtype=np.float64)
    if velocity.ndim != 2 or velocity.shape[0] != 2 or velocity.shape[1] < 1:
        return None
    linear, angular = float(velocity[0, 0]), float(velocity[1, 0])
    if not math.isfinite(linear) or not math.isfinite(angular) or linear >= -0.025:
        return None
    return max(linear, -0.10), max(-0.40, min(angular, 0.40))


class StopReverseRecovery:
    def __init__(self, max_distance_m: float = 0.25,
                 max_duration_sec: float = 4.0,
                 max_gear_wait_sec: float = 8.0,
                 rearm_distance_m: float = 0.35) -> None:
        self.max_distance_m = max_distance_m
        self.max_duration_sec = max_duration_sec
        self.max_gear_wait_sec = max_gear_wait_sec
        self.rearm_distance_m = rearm_distance_m
        self.reset()

    def reset(self) -> None:
        self.goal_pose: tuple[float, float, float] | None = None
        self.started_at: float | None = None
        self.gear_wait_started_at: float | None = None
        self.last_pose: tuple[float, float] | None = None
        self.origin: tuple[float, float] | None = None
        self.distance_m = 0.0
        self.exhausted = False

    def permit(self, *, goal_pose: tuple[float, float, float], pose: tuple[float, float],
               active_gear: int, now: float) -> bool:
        if not all(math.isfinite(value) for value in (*goal_pose, *pose, now)):
            return False
        same_goal = (self.goal_pose is not None and
                     math.dist(goal_pose[:2], self.goal_pose[:2]) <= 1e-4 and
                     abs(math.atan2(math.sin(goal_pose[2] - self.goal_pose[2]),
                                    math.cos(goal_pose[2] - self.goal_pose[2]))) <= 0.003)
        if not same_goal:
            self.reset()
            self.goal_pose = goal_pose
        if self.exhausted:
            if self.origin is None or math.dist(pose, self.origin) < self.rearm_distance_m:
                return False
            self.reset()
            self.goal_pose = goal_pose
        if active_gear != -1:
            if self.gear_wait_started_at is None:
                self.gear_wait_started_at = now
            if now - self.gear_wait_started_at >= self.max_gear_wait_sec:
                self.exhausted = True
                self.origin = pose
                return False
            return True  # GearTransitionGate still requires a confirmed stop.
        if self.started_at is None:
            self.started_at = now
            self.origin = pose
        elif self.last_pose is not None:
            self.distance_m += math.dist(pose, self.last_pose)
        self.last_pose = pose
        if (self.distance_m >= self.max_distance_m or
                now - self.started_at >= self.max_duration_sec):
            self.exhausted = True
            self.origin = pose
            return False
        return True
