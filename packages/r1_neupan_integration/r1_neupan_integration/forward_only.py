"""Forward-only actuator adapter for NeuPAN commands.

NeuPAN owns both path tracking and obstacle avoidance.  This adapter is only
the final no-reverse boundary: it never substitutes a global-path heading for
the planner's command.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class ForwardOnlyConfig:
    lookahead_m: float = 0.6
    max_forward_speed_mps: float = 0.15

    def __post_init__(self) -> None:
        positive = (self.lookahead_m, self.max_forward_speed_mps)
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("forward-only parameters must be finite and positive")


@dataclass(frozen=True)
class ForwardOnlyDecision:
    state: str
    linear_x: float
    angular_z: float
    reason: str
    raw_linear_x: float
    raw_angular_z: float
    yaw_error_rad: float
    observed_front_clearance_m: float | None
    required_front_clearance_m: float
    blocked: bool = False


class ForwardOnlyController:
    """Deterministic adapter state machine with no reverse output path."""

    def __init__(self, config: ForwardOnlyConfig) -> None:
        self.config = config
        self._context: tuple[int, int] | None = None
        self._state = "TRACK"

    def reset(self, path_version: int, goal_version: int) -> None:
        self._context = (int(path_version), int(goal_version))
        self._state = "TRACK"

    def _ensure_context(self, path_version: int, goal_version: int) -> None:
        context = (int(path_version), int(goal_version))
        if self._context != context:
            self.reset(*context)

    def _path_yaw_error(self, state: np.ndarray, path: np.ndarray) -> float:
        if state.shape[0] < 3 or path.shape[0] < 3 or path.shape[1] == 0:
            raise ValueError("state and path must contain x, y, yaw")
        robot_xy = np.asarray(state[:2, 0], dtype=np.float64)
        robot_yaw = float(state[2, 0])
        path_xy = np.asarray(path[:2, :], dtype=np.float64).T
        nearest = int(np.argmin(np.sum((path_xy - robot_xy) ** 2, axis=1)))
        target = path_xy[-1]
        traversed = 0.0
        # Measure lookahead along the SMAC polyline, not from the robot to
        # the nearest point.  When the robot has already drifted from the
        # path, counting that cross-track gap makes the nearest point satisfy
        # the lookahead immediately and produces an unnecessarily sharp
        # re-entry heading.
        previous = path_xy[nearest]
        for point in path_xy[nearest + 1:]:
            traversed += float(np.linalg.norm(point - previous))
            previous = point
            target = point
            if traversed >= self.config.lookahead_m:
                break
        direction = target - robot_xy
        if float(np.linalg.norm(direction)) > 1.0e-4:
            target_yaw = math.atan2(float(direction[1]), float(direction[0]))
        else:
            target_yaw = float(path[2, min(nearest, path.shape[1] - 1)])
        return _wrap(target_yaw - robot_yaw)

    def decide(
        self,
        *,
        raw_linear_x: float,
        raw_angular_z: float,
        stop_requested: bool,
        arrived: bool,
        state: np.ndarray,
        path: np.ndarray,
        path_version: int,
        goal_version: int,
    ) -> ForwardOnlyDecision:
        self._ensure_context(path_version, goal_version)
        raw_linear_x = float(raw_linear_x)
        raw_angular_z = float(raw_angular_z)
        if not math.isfinite(raw_linear_x) or not math.isfinite(raw_angular_z):
            raise ValueError("raw NeuPAN command must be finite")
        yaw_error = self._path_yaw_error(state, path)

        def decision(linear: float, angular: float, reason: str, blocked: bool = False) -> ForwardOnlyDecision:
            return ForwardOnlyDecision(
                state=self._state,
                linear_x=max(0.0, float(linear)),
                angular_z=float(angular),
                reason=reason,
                raw_linear_x=raw_linear_x,
                raw_angular_z=raw_angular_z,
                yaw_error_rad=yaw_error,
                observed_front_clearance_m=None,
                required_front_clearance_m=0.0,
                blocked=blocked,
            )

        if arrived:
            self._state = "ARRIVED"
            return decision(0.0, 0.0, "arrived")

        # A planner stop is authoritative.  In particular, do not retain an
        # angular component here: NeuPAN used this cycle to request a stop.
        if stop_requested:
            self._state = "WAIT"
            return decision(0.0, 0.0, "planner_stop_requested")

        if raw_linear_x >= 0.0:
            self._state = "TRACK"
            return decision(raw_linear_x, raw_angular_z, "forward_solution")

        # Retain NeuPAN's turn when it requested reverse.  The next solve sees
        # the new pose and can produce a forward solution without an external
        # controller forcing it toward the SMAC heading.
        self._state = "REORIENT"
        return decision(0.0, raw_angular_z, "reverse_linear_clamped")
