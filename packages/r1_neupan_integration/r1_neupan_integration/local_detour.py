"""Stable scan-validated local reference paths for NeuPAN obstacle avoidance."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from r1_neupan_integration.obstacle_motion_policy import (
    MotionDecision, MotionPolicyConfig, first_command_collision_distance,
    first_rotation_collision_distance,
    first_swept_collision_distance,
)


@dataclass(frozen=True)
class LocalDetour:
    path: np.ndarray
    identifier: int
    side: int
    side_xy: np.ndarray
    pass_xy: np.ndarray
    join_xy: np.ndarray
    origin_xy: np.ndarray
    direction_xy: np.ndarray
    obstacle_far_m: float
    created_at: float


def _sample_waypoints(waypoints: list[tuple[float, float, float]], gear: int) -> np.ndarray:
    columns = []
    for index, (x, y, yaw) in enumerate(waypoints):
        if index == 0:
            columns.append((x, y, yaw))
            continue
        x0, y0, yaw0 = waypoints[index - 1]
        distance = math.hypot(x - x0, y - y0)
        count = max(1, math.ceil(distance / 0.10))
        dyaw = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
        for step in range(1, count + 1):
            t = step / count
            columns.append((x0 + t * (x - x0), y0 + t * (y - y0), yaw0 + t * dyaw))
    return np.vstack((np.asarray(columns).T, np.full(len(columns), gear)))


class LocalDetourPlanner:
    def __init__(self, config: MotionPolicyConfig, clearance_m: float = 0.18) -> None:
        self.config = config
        self.clearance_m = clearance_m
        self.active: LocalDetour | None = None
        self.path_version = -1
        self.target_pose: np.ndarray | None = None
        self.next_identifier = 1
        self.last_status: dict[str, object] = {"state": "idle", "sides": {}}

    def reset(self) -> None:
        self.active = None
        self.path_version = -1
        self.target_pose = None
        self.last_status = {"state": "idle", "sides": {}}

    def _candidate(
        self, state: np.ndarray, route: np.ndarray, cloud: np.ndarray,
        direction: np.ndarray, side: int, route_distance: float, now: float,
    ) -> tuple[LocalDetour | None, str]:
        origin = state[:2, 0]
        lateral = np.asarray([-direction[1], direction[0]])
        relative = cloud - origin[:, None]
        forward = direction @ relative
        across = lateral @ relative
        # Size the bypass around points that obstruct the route. Points in an
        # adjacent lane still participate in the full collision validation.
        blockers = (
            (forward >= -0.05) & (forward <= self.config.preview_m + self.config.front_m)
            & (across >= -self.config.right_m - 0.05)
            & (across <= self.config.left_m + 0.05)
        )
        if not np.any(blockers):
            return None, "no_route_blocker"
        obstacle_far = float(np.max(forward[blockers]))
        turning_radius = math.hypot(
            max(self.config.front_m, self.config.rear_m),
            max(self.config.left_m, self.config.right_m),
        )
        if side > 0:
            offset = float(np.max(across[blockers]) + turning_radius + self.clearance_m)
        else:
            offset = float(np.min(across[blockers]) - turning_radius - self.clearance_m)
        if abs(offset) > 1.20:
            return None, "offset_limit"
        side_x = min(0.12, max(0.0, route_distance - 0.10))
        pass_x = max(0.90, obstacle_far + self.config.front_m + self.clearance_m)
        join_x = pass_x + 0.55
        route_relative = route[:2] - origin[:, None]
        route_forward = direction @ route_relative
        suffix_indices = np.flatnonzero(route_forward >= join_x)
        if suffix_indices.size:
            suffix_start = int(suffix_indices[0])
        elif route_forward[-1] >= pass_x - 0.05:
            suffix_start = route.shape[1] - 1
        else:
            return None, "route_ends_before_pass"
        side_xy = origin + side_x * direction + offset * lateral
        pass_xy = origin + pass_x * direction + offset * lateral
        join_xy = route[:2, suffix_start]
        side_yaw = math.atan2(side_xy[1] - origin[1], side_xy[0] - origin[0])
        route_yaw = math.atan2(direction[1], direction[0])
        waypoints = [
            (float(origin[0]), float(origin[1]), float(state[2, 0])),
            (float(side_xy[0]), float(side_xy[1]), side_yaw),
            (float(pass_xy[0]), float(pass_xy[1]), route_yaw),
            (float(join_xy[0]), float(join_xy[1]), float(route[2, suffix_start])),
        ]
        candidate = _sample_waypoints(waypoints, int(route[3, 0]))
        if suffix_start + 1 < route.shape[1]:
            candidate = np.hstack((candidate, route[:, suffix_start + 1:]))
        return LocalDetour(
            candidate, self.next_identifier, side, side_xy, pass_xy, join_xy,
            origin.copy(), direction.copy(), obstacle_far, now,
        ), "candidate"

    def update(
        self, state: np.ndarray, route: np.ndarray, raw_points: np.ndarray,
        planner_points: np.ndarray, route_decision: MotionDecision,
        path_version: int, now: float,
    ) -> LocalDetour | None:
        target = route[:3, -1].copy()
        target_changed = (
            self.target_pose is not None and (
                np.linalg.norm(target[:2] - self.target_pose[:2]) > 0.15
                or abs(math.atan2(
                    math.sin(float(target[2] - self.target_pose[2])),
                    math.cos(float(target[2] - self.target_pose[2])),
                )) > 0.35
            )
        )
        if target_changed:
            self.reset()
        elif self.path_version != path_version and self.active is None:
            self.reset()
        self.target_pose = target
        self.path_version = path_version
        self.last_status = {"state": "route_clear", "sides": {}}
        cloud = np.hstack((raw_points, planner_points))
        if self.active is not None:
            active = self.active
            validation = replace(
                self.config, preview_m=1.20, sample_step_m=0.04,
                front_m=self.config.front_m + 0.03,
                rear_m=self.config.rear_m + 0.03,
                left_m=self.config.left_m + 0.03,
                right_m=self.config.right_m + 0.03,
            )
            if not math.isinf(first_swept_collision_distance(
                state, active.path, cloud, validation
            )):
                self.active = None
        if self.active is not None:
            active = self.active
            progress = float(active.direction_xy @ (state[:2, 0] - active.origin_xy))
            lateral_error = abs(float(np.cross(
                active.direction_xy, state[:2, 0] - active.origin_xy
            )))
            if (progress > active.obstacle_far_m + self.config.front_m + 0.20
                    and lateral_error < 0.15 and not route_decision.route_blocked):
                self.active = None
        if self.active is not None or not route_decision.route_blocked:
            if self.active is not None:
                self.last_status = {"state": "active", "sides": {}}
            return self.active
        if route.shape[0] < 4 or route.shape[1] < 3 or cloud.shape[1] == 0:
            self.last_status = {"state": "insufficient_input", "sides": {}}
            return None
        xy = route[:2].T
        closest = int(np.argmin(np.sum((xy - state[:2, 0]) ** 2, axis=1)))
        ahead = min(route.shape[1] - 1, closest + 2)
        tangent = route[:2, ahead] - route[:2, closest]
        norm = float(np.linalg.norm(tangent))
        if norm < 1.0e-4:
            self.last_status = {"state": "route_tangent_missing", "sides": {}}
            return None
        direction = tangent / norm
        heading = np.asarray([math.cos(state[2, 0]), math.sin(state[2, 0])])
        if float(direction @ heading) < 0.75:
            self.last_status = {"state": "heading_misaligned", "sides": {}}
            return None
        validation = replace(
            self.config, preview_m=3.0, sample_step_m=0.04,
            front_m=self.config.front_m + 0.03,
            rear_m=self.config.rear_m + 0.03,
            left_m=self.config.left_m + 0.03,
            right_m=self.config.right_m + 0.03,
        )
        feasible = []
        side_status: dict[str, str] = {}
        for side in (1, -1):
            side_name = "left" if side > 0 else "right"
            candidate, reason = self._candidate(
                state, route, cloud, direction, side, route_decision.free_distance_m, now
            )
            if candidate is None:
                side_status[side_name] = reason
                continue
            side_bearing = math.atan2(
                candidate.side_xy[1] - state[1, 0],
                candidate.side_xy[0] - state[0, 0],
            )
            turn = math.atan2(
                math.sin(side_bearing - state[2, 0]),
                math.cos(side_bearing - state[2, 0]),
            )
            if not math.isinf(first_rotation_collision_distance(
                state, turn, cloud, validation
            )):
                side_status[side_name] = "start_rotation_blocked"
                continue
            pass_bearing = math.atan2(
                candidate.pass_xy[1] - candidate.side_xy[1],
                candidate.pass_xy[0] - candidate.side_xy[0],
            )
            join_bearing = math.atan2(
                candidate.join_xy[1] - candidate.pass_xy[1],
                candidate.join_xy[0] - candidate.pass_xy[0],
            )
            waypoint_turns = (
                (candidate.side_xy, side_bearing, pass_bearing),
                (candidate.pass_xy, pass_bearing, join_bearing),
            )
            if any(not math.isinf(first_rotation_collision_distance(
                np.asarray([[xy[0]], [xy[1]], [bearing]]),
                math.atan2(math.sin(target - bearing), math.cos(target - bearing)),
                cloud, validation,
            )) for xy, bearing, target in waypoint_turns):
                side_status[side_name] = "waypoint_rotation_blocked"
                continue
            legs = (
                (state[:2, 0], candidate.side_xy, side_bearing),
                (candidate.side_xy, candidate.pass_xy, pass_bearing),
                (candidate.pass_xy, candidate.join_xy, join_bearing),
            )
            if not all(math.isinf(first_swept_collision_distance(
                np.asarray([[start[0]], [start[1]], [bearing]]),
                np.asarray([[start[0], end[0]], [start[1], end[1]],
                            [bearing, bearing]]),
                cloud,
                replace(validation, preview_m=max(0.01, float(np.linalg.norm(end - start)))),
            )) for start, end, bearing in legs):
                side_status[side_name] = "leg_blocked"
                continue
            if math.isinf(first_swept_collision_distance(
                state, candidate.path, cloud, validation
            )):
                feasible.append(candidate)
                side_status[side_name] = "feasible"
            else:
                side_status[side_name] = "reference_path_blocked"
        if not feasible:
            self.last_status = {"state": "no_feasible_side", "sides": side_status}
            return None
        self.active = min(feasible, key=lambda value: np.linalg.norm(
            value.path[:2, 1] - value.origin_xy
        ))
        self.next_identifier += 1
        self.last_status = {"state": "selected", "sides": side_status}
        return self.active


class LocalDetourController:
    """Track each local waypoint once, without switching back to a passed one."""

    def __init__(self) -> None:
        self.identifier = 0
        self.phase_index = 0

    def reset(self) -> None:
        self.identifier = 0
        self.phase_index = 0

    def command(self, state: np.ndarray, detour: LocalDetour) -> tuple[float, float, str]:
        if self.identifier != detour.identifier:
            self.identifier = detour.identifier
            self.phase_index = 0
        pose = np.asarray(state, dtype=np.float64).reshape(-1)
        if self.phase_index == 0 and np.linalg.norm(pose[:2] - detour.side_xy) <= 0.12:
            self.phase_index = 1
        if self.phase_index == 1 and np.linalg.norm(pose[:2] - detour.pass_xy) <= 0.15:
            self.phase_index = 2
        targets = (detour.side_xy, detour.pass_xy, detour.join_xy)
        phases = ("TURN_SIDE", "PASS", "REJOIN")
        target = targets[self.phase_index]
        desired = math.atan2(target[1] - pose[1], target[0] - pose[0])
        error = math.atan2(math.sin(desired - pose[2]), math.cos(desired - pose[2]))
        angular = max(-0.30, min(0.30, 1.5 * error))
        if abs(error) > 0.20:
            angular = math.copysign(max(0.12, abs(angular)), angular)
            return 0.0, angular, phases[self.phase_index]
        linear = 0.16 * max(0.35, 1.0 - abs(error) / 0.50)
        return linear, angular, phases[self.phase_index]


class ReverseEscapePlanner:
    """One bounded reverse attempt after a blocked local route has settled."""

    def __init__(
        self, config: MotionPolicyConfig, delay_sec: float = 1.5,
        speed_mps: float = 0.10, max_distance_m: float = 0.40,
        max_duration_sec: float = 6.0,
    ) -> None:
        self.config = config
        self.delay_sec = delay_sec
        self.speed_mps = speed_mps
        self.max_distance_m = max_distance_m
        self.max_duration_sec = max_duration_sec
        self.context_version = -1
        self.wait_since: float | None = None
        self.started_at: float | None = None
        self.start_xy: np.ndarray | None = None
        self.exhausted = False

    def update(
        self, state: np.ndarray, measured_linear_x: float,
        raw_points: np.ndarray, planner_points: np.ndarray,
        route_decision: MotionDecision, detour_available: bool,
        active_gear: int, context_version: int, now: float,
    ) -> tuple[float | None, str]:
        if context_version != self.context_version:
            backing_interrupted = self.started_at is not None and active_gear < 0
            self.context_version = context_version
            self.wait_since = None
            self.started_at = None
            self.start_xy = None
            self.exhausted = backing_interrupted
        if self.started_at is not None:
            travelled = float(np.linalg.norm(state[:2, 0] - self.start_xy))
            rear_distance = min(
                first_command_collision_distance(
                    state, -self.speed_mps, 0.0, raw_points, self.config
                ),
                first_command_collision_distance(
                    state, -self.speed_mps, 0.0, planner_points, self.config
                ),
            )
            required = (
                self.speed_mps * self.config.reaction_sec
                + self.speed_mps ** 2 / (2.0 * self.config.braking_mps2)
                + self.config.stop_margin_m
            )
            if (travelled >= self.max_distance_m or
                    now - self.started_at >= self.max_duration_sec or
                    rear_distance <= required):
                self.started_at = None
                self.exhausted = True
            else:
                return -self.speed_mps, "BACKING"
        if self.exhausted:
            return (0.06, "RETURN_FORWARD_GEAR") if active_gear < 0 else (None, "EXHAUSTED")
        stopped_at_blocker = (
            active_gear == 1 and route_decision.route_blocked and not detour_available
            and abs(measured_linear_x) <= 0.015
        )
        if not stopped_at_blocker:
            self.wait_since = None
            return None, "NOT_BLOCKED"
        if self.wait_since is None:
            self.wait_since = now
            return None, "WAITING"
        if now - self.wait_since < self.delay_sec:
            return None, "WAITING"
        rear_distance = min(
            first_command_collision_distance(
                state, -self.speed_mps, 0.0, raw_points, self.config
            ),
            first_command_collision_distance(
                state, -self.speed_mps, 0.0, planner_points, self.config
            ),
        )
        if rear_distance <= 0.30:
            return None, "REAR_BLOCKED"
        self.started_at = now
        self.start_xy = state[:2, 0].copy()
        return -self.speed_mps, "BACKING"
