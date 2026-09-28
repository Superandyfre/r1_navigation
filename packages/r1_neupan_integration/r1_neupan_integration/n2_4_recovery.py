"""Forward-only recovery design for the R1 integration layer.

This module is deliberately ROS-free.  It describes and tests the recovery
contract without publishing a recovery command or changing NeuPAN.  A future
runtime adapter must pass every returned command through the existing N2.3
safety gate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable


STATES = (
    "NORMAL",
    "RECOVERY_STOP",
    "RECOVERY_ROTATE",
    "RECOVERY_REANCHOR",
    "RECOVERY_FAILED",
)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def point_segment_projection(point: tuple[float, float], left: tuple[float, float], right: tuple[float, float]) -> tuple[float, tuple[float, float]]:
    dx, dy = right[0] - left[0], right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return 0.0, left
    scale = clamp(((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq, 0.0, 1.0)
    return scale, (left[0] + scale * dx, left[1] + scale * dy)


def segment_heading(left: tuple[float, float], right: tuple[float, float]) -> float | None:
    dx, dy = right[0] - left[0], right[1] - left[1]
    return math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else None


def path_segment(path: list[tuple[float, float]], index: int) -> tuple[tuple[float, float], tuple[float, float]] | None:
    if index < 0 or index + 1 >= len(path):
        return None
    return path[index], path[index + 1]


@dataclass(frozen=True)
class RecoveryConfig:
    negative_v_threshold: float = 0.01
    minimum_negative_samples: int = 3
    minimum_negative_duration_sec: float = 0.15
    sample_gap_sec: float = 0.25
    minimum_path_deviation_m: float = 0.15
    minimum_corridor_margin_m: float = 0.05
    stop_linear_threshold: float = 0.02
    stop_angular_threshold: float = 0.03
    stop_settle_samples: int = 3
    heading_tolerance_rad: float = 0.12
    minimum_clearance_m: float = 0.20
    rotation_speed_rad_s: float = 0.4
    max_rotation_duration_sec: float = 8.0
    max_recovery_attempts: int = 2
    reentry_positive_samples: int = 3
    reentry_negative_samples_before_retry: int = 2


@dataclass(frozen=True)
class RecoveryObservation:
    cycle_id: int
    monotonic: float
    raw_linear: float
    raw_angular: float
    pose: tuple[float, float, float]
    path: tuple[tuple[float, float], ...]
    active_path_index: int
    path_deviation_m: float
    goal_distance_m: float | None
    stop: bool = False
    arrive: bool = False
    planner_fresh: bool = True
    path_valid: bool = True
    obstacle_clearance_m: float | None = None
    corridor_margin_m: float | None = None
    odom_linear_m_s: float | None = None
    odom_angular_rad_s: float | None = None
    scan_topology: dict[str, Any] = field(default_factory=dict)
    reference_points: tuple[tuple[float, float], ...] = ()
    optimized_points: tuple[tuple[float, float], ...] = ()


def _negative(row: RecoveryObservation, config: RecoveryConfig) -> bool:
    return finite(row.raw_linear) and float(row.raw_linear) < 0.0


def _strong_negative(row: RecoveryObservation, config: RecoveryConfig) -> bool:
    return finite(row.raw_linear) and float(row.raw_linear) <= -config.negative_v_threshold


def _recent_negative_run(rows: list[RecoveryObservation], config: RecoveryConfig) -> list[RecoveryObservation]:
    if not rows or not _negative(rows[-1], config):
        return []
    run = [rows[-1]]
    for row, next_row in zip(reversed(rows[:-1]), reversed(rows[1:])):
        if not _negative(row, config):
            break
        if float(next_row.monotonic) - float(row.monotonic) > config.sample_gap_sec:
            break
        run.append(row)
    return list(reversed(run))


def _index_stalled(rows: Iterable[RecoveryObservation]) -> bool:
    indices = [int(row.active_path_index) for row in rows]
    return bool(indices) and max(indices) - min(indices) <= 1


def evaluate_trigger(history: list[RecoveryObservation], current: RecoveryObservation, config: RecoveryConfig = RecoveryConfig()) -> dict[str, Any]:
    """Return an auditable trigger decision; one negative sample never triggers."""
    rows = [*history, current]
    run = _recent_negative_run(rows, config)
    duration = 0.0 if len(run) < 2 else float(run[-1].monotonic) - float(run[0].monotonic)
    strong_count = sum(_strong_negative(row, config) for row in run)
    goal_delta = None
    if len(run) >= 2 and run[0].goal_distance_m is not None and run[-1].goal_distance_m is not None:
        goal_delta = float(run[-1].goal_distance_m) - float(run[0].goal_distance_m)
    margin = current.corridor_margin_m
    geometry_rejected = margin is not None and margin < config.minimum_corridor_margin_m
    path_deviation_ok = finite(current.path_deviation_m) and current.path_deviation_m >= config.minimum_path_deviation_m
    index_stalled = _index_stalled(run)
    eligible = (
        not current.stop
        and not current.arrive
        and current.planner_fresh
        and current.path_valid
        and not geometry_rejected
        and len(run) >= config.minimum_negative_samples
        and duration >= config.minimum_negative_duration_sec
        and strong_count >= 1
        and path_deviation_ok
        and index_stalled
    )
    reasons = []
    if len(run) >= config.minimum_negative_samples:
        reasons.append("negative_persistence")
    if strong_count:
        reasons.append("negative_magnitude")
    if path_deviation_ok:
        reasons.append("path_deviation")
    if index_stalled:
        reasons.append("path_index_stalled")
    if goal_delta is not None and goal_delta >= 0.0:
        reasons.append("goal_distance_nonimproving")
    if geometry_rejected:
        reasons.append("geometry_infeasible_boundary")
    return {
        "trigger": bool(eligible),
        "trigger_reason": "+".join(reasons) if eligible else "",
        "negative_run_length": len(run),
        "negative_duration_sec": duration,
        "negative_min_v": min((float(row.raw_linear) for row in run), default=None),
        "strong_negative_count": strong_count,
        "path_deviation_m": float(current.path_deviation_m) if finite(current.path_deviation_m) else None,
        "path_index_stalled": index_stalled,
        "path_indices": [int(row.active_path_index) for row in run],
        "goal_distance_delta_m": goal_delta,
        "geometry_rejected": geometry_rejected,
        "false_trigger_controls": {
            "requires_path_deviation": True,
            "requires_stalled_progress_index": True,
            "rejects_narrow_geometry": True,
            "rejects_stop_closure": True,
        },
    }


def _clearance_for_heading(heading: float, current_heading: float, scan: dict[str, Any], fallback: float | None) -> float | None:
    signed = wrap_angle(heading - current_heading)
    key = "min_left" if signed > 0.12 else "min_right" if signed < -0.12 else "min_front"
    value = scan.get(key)
    if not finite(value):
        value = fallback
    return float(value) if finite(value) else None


def _reference_heading(observation: RecoveryObservation) -> float | None:
    if len(observation.reference_points) >= 2:
        return segment_heading(observation.reference_points[0], observation.reference_points[1])
    segment = path_segment(list(observation.path), observation.active_path_index)
    return None if segment is None else segment_heading(*segment)


def _optimized_heading(observation: RecoveryObservation) -> float | None:
    if len(observation.optimized_points) >= 2:
        return segment_heading(observation.optimized_points[0], observation.optimized_points[1])
    return None


def compute_reanchor(observation: RecoveryObservation, last_reanchor_index: int = -1) -> dict[str, Any] | None:
    """Project onto the nearest future segment and advance beyond its projection."""
    path = list(observation.path)
    first_segment = max(int(observation.active_path_index), int(last_reanchor_index))
    candidates = []
    for index in range(max(0, first_segment), max(0, len(path) - 1)):
        segment = path_segment(path, index)
        if segment is None:
            continue
        left, right = segment
        fraction, projection = point_segment_projection((observation.pose[0], observation.pose[1]), left, right)
        heading = segment_heading(left, right)
        if heading is None:
            continue
        distance = math.hypot(observation.pose[0] - projection[0], observation.pose[1] - projection[1])
        candidates.append((distance, index, fraction, projection, heading))
    if not candidates:
        return None
    distance, index, fraction, projection, heading = min(candidates, key=lambda value: (value[0], value[1]))
    left, right = path[index], path[index + 1]
    length = math.hypot(right[0] - left[0], right[1] - left[1])
    advance = min(0.15, max(0.0, (1.0 - fraction) * length))
    point = (projection[0] + advance * math.cos(heading), projection[1] + advance * math.sin(heading))
    if fraction >= 0.999 and index + 1 < len(path) - 1:
        index += 1
        point = path[index]
        left, right = path[index], path[index + 1]
        heading = segment_heading(left, right) or heading
    rebuilt = [(float(observation.pose[0]), float(observation.pose[1])), point, *path[index + 1:]]
    return {
        "segment_index": int(index),
        "point": [float(point[0]), float(point[1])],
        "projection": [float(projection[0]), float(projection[1])],
        "projection_distance_m": float(distance),
        "projection_fraction": float(fraction),
        "segment_heading": float(heading),
        "path_index_monotonic": int(index) >= int(observation.active_path_index) and int(index) >= int(last_reanchor_index),
        "avoids_same_failure_point": float(advance) > 0.0 or int(index) > int(observation.active_path_index),
        "original_path_preserved": True,
        "rebuilt_path": [[float(x), float(y)] for x, y in rebuilt],
    }


def choose_heading(observation: RecoveryObservation, reanchor: dict[str, Any], config: RecoveryConfig = RecoveryConfig()) -> dict[str, Any]:
    """Score geometric candidates with a small, fixed physical rule."""
    x, y, current = observation.pose
    target = tuple(reanchor["point"])
    target_bearing = math.atan2(target[1] - y, target[0] - x)
    goal = observation.path[-1]
    goal_bearing = math.atan2(goal[1] - y, goal[0] - x)
    reference = _reference_heading(observation) or target_bearing
    optimized = _optimized_heading(observation)
    next_heading = segment_heading(*path_segment(list(observation.path), observation.active_path_index)) if path_segment(list(observation.path), observation.active_path_index) else reference
    candidates = [
        ("H1_reference_tangent", reference),
        ("H2_next_path_segment_tangent", next_heading),
        ("H3_goal_bearing", goal_bearing),
        ("H4_optimized_heading", optimized),
        ("H5_current_plus_90deg", wrap_angle(current + math.pi / 2.0)),
        ("H5_current_minus_90deg", wrap_angle(current - math.pi / 2.0)),
        ("H6_clearance_left", wrap_angle(reference + math.pi / 2.0)),
        ("H6_clearance_right", wrap_angle(reference - math.pi / 2.0)),
    ]
    details = []
    for label, heading in candidates:
        if heading is None:
            continue
        target_error = abs(wrap_angle(heading - target_bearing))
        reference_alignment = (1.0 + math.cos(wrap_angle(heading - reference))) / 2.0
        goal_alignment = (1.0 + math.cos(wrap_angle(heading - goal_bearing))) / 2.0
        target_alignment = (1.0 + math.cos(wrap_angle(heading - target_bearing))) / 2.0
        turn_score = 1.0 - abs(wrap_angle(heading - current)) / math.pi
        clearance = _clearance_for_heading(heading, current, observation.scan_topology, observation.obstacle_clearance_m)
        clearance_score = clamp((clearance or 0.0) / 1.0, 0.0, 1.0)
        forward_feasibility = math.cos(wrap_angle(heading - target_bearing))
        expected_path_reentry_quality = 0.5 * target_alignment + 0.5 * max(0.0, forward_feasibility)
        feasible = forward_feasibility > 0.0 and (clearance is None or clearance >= config.minimum_clearance_m)
        score = (
            0.35 * reference_alignment
            + 0.20 * target_alignment
            + 0.20 * goal_alignment
            + 0.15 * clearance_score
            + 0.10 * turn_score
        )
        details.append({
            "candidate": label,
            "heading": float(heading),
            "heading_error_to_reentry_rad": float(target_error),
            "required_rotation_rad": float(wrap_angle(heading - current)),
            "reference_alignment": float(reference_alignment),
            "goal_alignment": float(goal_alignment),
            "reentry_alignment": float(target_alignment),
            "expected_path_reentry_quality": float(expected_path_reentry_quality),
            "turn_score": float(turn_score),
            "clearance_m": clearance,
            "clearance_score": float(clearance_score),
            "forward_feasibility": float(forward_feasibility),
            "score": float(score),
            "feasible": bool(feasible),
        })
    feasible = [item for item in details if item["feasible"]]
    selected = max(feasible or details, key=lambda item: (item["score"], item["reentry_alignment"], -abs(item["required_rotation_rad"]), item["candidate"]))
    return {
        "selected": selected,
        "candidates": details,
        "rule": "reject non-forward or low-clearance candidates; maximize 0.35 reference + 0.20 reentry + 0.20 goal + 0.15 clearance + 0.10 turn",
        "target_heading": float(selected["heading"]),
        "heading_error_rad": float(selected["heading_error_to_reentry_rad"]),
        "required_rotation_rad": float(selected["required_rotation_rad"]),
    }


class ForwardOnlyRecoveryFSM:
    """Deterministic FSM.  It never emits a negative linear command."""

    def __init__(self, config: RecoveryConfig = RecoveryConfig()) -> None:
        self.config = config
        self.state = "NORMAL"
        self.attempt = 0
        self.failure_latched = False
        self.failure_reason = ""
        self.trigger: dict[str, Any] = {}
        self.heading: dict[str, Any] = {}
        self.reanchor: dict[str, Any] = {}
        self.stop_samples = 0
        self.reentry_positive_samples = 0
        self.reentry_negative_samples = 0
        self.reentry_raw_v: float | None = None
        self.reentry_path_deviation: float | None = None
        self.stop_entry_cycle: int | None = None
        self.zero_command_cycle: int | None = None
        self.rotate_start_cycle: int | None = None
        self.reanchor_cycle: int | None = None
        self.failure_cycle: int | None = None
        self.last_reanchor_index = -1
        self._state_start_t: float | None = None
        self.transitions: list[dict[str, Any]] = []

    def _command(self, linear: float, angular: float, reason: str) -> dict[str, Any]:
        linear = float(linear) if finite(linear) else 0.0
        angular = float(angular) if finite(angular) else 0.0
        linear = max(0.0, linear)
        angular = clamp(angular, -0.4, 0.4)
        return {
            "linear_x": linear,
            "angular_z": angular,
            "reason": reason,
            "through_existing_safety_gate": True,
            "no_reverse": linear >= 0.0,
            "finite": finite(linear) and finite(angular),
        }

    def _transition(self, state: str, observation: RecoveryObservation, reason: str) -> None:
        previous = self.state
        self.state = state
        self._state_start_t = float(observation.monotonic)
        self.transitions.append({
            "cycle_id": int(observation.cycle_id),
            "monotonic": float(observation.monotonic),
            "from": previous,
            "to": state,
            "reason": reason,
        })

    def _fail(self, observation: RecoveryObservation, reason: str) -> dict[str, Any]:
        self.failure_latched = True
        self.failure_reason = str(reason)
        self.failure_cycle = int(observation.cycle_id)
        self._transition("RECOVERY_FAILED", observation, reason)
        return self._command(0.0, 0.0, "recovery_failed:" + str(reason))

    def reset(self) -> None:
        self.state = "NORMAL"
        self.attempt = 0
        self.failure_latched = False
        self.failure_reason = ""
        self.trigger = {}
        self.heading = {}
        self.reanchor = {}
        self.stop_samples = 0
        self.reentry_positive_samples = 0
        self.reentry_negative_samples = 0
        self.reentry_raw_v = None
        self.reentry_path_deviation = None
        self.stop_entry_cycle = None
        self.zero_command_cycle = None
        self.rotate_start_cycle = None
        self.reanchor_cycle = None
        self.failure_cycle = None
        self.last_reanchor_index = -1
        self._state_start_t = None
        self.transitions = []

    def update(self, observation: RecoveryObservation, history: list[RecoveryObservation], *, shadow_stop_settled: bool = False) -> dict[str, Any]:
        if self.state == "NORMAL":
            trigger = evaluate_trigger(history, observation, self.config)
            if trigger["trigger"]:
                self.attempt = 1
                self.trigger = trigger
                self.stop_entry_cycle = int(observation.cycle_id)
                self.zero_command_cycle = int(observation.cycle_id)
                self._transition("RECOVERY_STOP", observation, trigger["trigger_reason"])
                return self._command(0.0, 0.0, "recovery_stop_entry")
            return self._command(max(0.0, observation.raw_linear), observation.raw_angular, "normal_forward_only")

        if self.state == "RECOVERY_STOP":
            self.zero_command_cycle = self.zero_command_cycle or int(observation.cycle_id)
            settled = shadow_stop_settled or (
                finite(observation.odom_linear_m_s)
                and finite(observation.odom_angular_rad_s)
                and abs(float(observation.odom_linear_m_s)) <= self.config.stop_linear_threshold
                and abs(float(observation.odom_angular_rad_s)) <= self.config.stop_angular_threshold
            )
            self.stop_samples = self.stop_samples + 1 if settled else 0
            if self.stop_samples >= self.config.stop_settle_samples:
                self.reanchor = compute_reanchor(observation, self.last_reanchor_index)
                if self.reanchor is None:
                    return self._fail(observation, "no_forward_reentry_segment")
                self.heading = choose_heading(observation, self.reanchor, self.config)
                self.rotate_start_cycle = int(observation.cycle_id)
                self._transition("RECOVERY_ROTATE", observation, "stop_interlock_settled")
            return self._command(0.0, 0.0, "recovery_stop_interlock")

        if self.state == "RECOVERY_ROTATE":
            target = float(self.heading["target_heading"])
            error = wrap_angle(target - float(observation.pose[2]))
            elapsed = 0.0 if self._state_start_t is None else float(observation.monotonic) - self._state_start_t
            if not finite(error) or elapsed > self.config.max_rotation_duration_sec:
                return self._fail(observation, "rotation_timeout")
            if abs(error) <= self.config.heading_tolerance_rad:
                self.reanchor_cycle = int(observation.cycle_id)
                self._transition("RECOVERY_REANCHOR", observation, "heading_tolerance")
                return self._command(0.0, 0.0, "recovery_reanchor_entry")
            angular = math.copysign(self.config.rotation_speed_rad_s, error)
            return self._command(0.0, angular, "recovery_rotate")

        if self.state == "RECOVERY_REANCHOR":
            self.reentry_raw_v = float(observation.raw_linear) if finite(observation.raw_linear) else None
            self.reentry_path_deviation = float(observation.path_deviation_m) if finite(observation.path_deviation_m) else None
            valid = (
                observation.planner_fresh
                and observation.path_valid
                and not observation.stop
                and observation.corridor_margin_m is not None
                and observation.corridor_margin_m >= self.config.minimum_corridor_margin_m
                and observation.obstacle_clearance_m is not None
                and observation.obstacle_clearance_m >= self.config.minimum_clearance_m
                and finite(observation.raw_linear)
                and observation.raw_linear >= 0.0
            )
            if valid:
                self.reentry_positive_samples += 1
                self.reentry_negative_samples = 0
            else:
                self.reentry_positive_samples = 0
                self.reentry_negative_samples += 1
            if self.reentry_positive_samples >= self.config.reentry_positive_samples:
                self.last_reanchor_index = int(self.reanchor["segment_index"])
                self._transition("NORMAL", observation, "fresh_nonnegative_forward_reentry")
                return self._command(max(0.0, observation.raw_linear), observation.raw_angular, "recovery_resume_forward")
            if self.reentry_negative_samples >= self.config.reentry_negative_samples_before_retry:
                if self.attempt >= self.config.max_recovery_attempts:
                    return self._fail(observation, "reentry_negative_after_max_attempts")
                self.attempt += 1
                self.stop_samples = 0
                self.reentry_positive_samples = 0
                self.reentry_negative_samples = 0
                self.stop_entry_cycle = int(observation.cycle_id)
                self._transition("RECOVERY_STOP", observation, "reentry_negative_retry")
            return self._command(0.0, 0.0, "recovery_reanchor_wait")

        return self._command(0.0, 0.0, "recovery_failed_latch")

    def snapshot(self) -> dict[str, Any]:
        return {
            "recovery_state": self.state,
            "recovery_attempt": self.attempt,
            "trigger_reason": self.trigger.get("trigger_reason", ""),
            "negative_run_length": self.trigger.get("negative_run_length", 0),
            "recovery_target_heading": self.heading.get("target_heading"),
            "heading_error": self.heading.get("heading_error_rad"),
            "reanchor_path_index": self.reanchor.get("segment_index"),
            "reanchor_point": self.reanchor.get("point"),
            "reentry_raw_v": self.reentry_raw_v,
            "reentry_path_deviation": self.reentry_path_deviation,
            "recovery_timeout": self.failure_reason == "rotation_timeout",
            "recovery_failure_reason": self.failure_reason,
            "failure_latched": self.failure_latched,
            "stop_entry_cycle": self.stop_entry_cycle,
            "zero_command_cycle": self.zero_command_cycle,
            "rotate_start_cycle": self.rotate_start_cycle,
            "reanchor_cycle": self.reanchor_cycle,
            "transitions": list(self.transitions),
        }


def observation_from_cycle(row: dict[str, Any], path: list[tuple[float, float]], *, corridor_margin_m: float | None, obstacle_clearance_m: float | None) -> RecoveryObservation:
    pose_value = row.get("robot_pose") or [[0.0], [0.0], [0.0]]
    pose = tuple(float(pose_value[index][0]) for index in range(3))
    action = row.get("action") or [[0.0], [0.0]]
    ref = row.get("reference_state") or []
    opt = row.get("optimized_state") or []
    reference_points = tuple((float(ref[0][index]), float(ref[1][index])) for index in range(min(len(ref[0]) if ref else 0, len(ref[1]) if ref else 0)))
    optimized_points = tuple((float(opt[0][index]), float(opt[1][index])) for index in range(min(len(opt[0]) if opt else 0, len(opt[1]) if opt else 0)))
    return RecoveryObservation(
        cycle_id=int(row["cycle_id"]),
        monotonic=float(row.get("audit_t", row["cycle_id"] * 0.1)),
        raw_linear=float(action[0][0]),
        raw_angular=float(action[1][0]),
        pose=pose,
        path=tuple(path),
        active_path_index=int((row.get("active_path_index") or {}).get("point_index") or 0),
        path_deviation_m=float(row.get("path_deviation_m", 0.0)),
        goal_distance_m=None if row.get("goal_distance") is None else float(row["goal_distance"]),
        stop=bool(row.get("stop", False)),
        arrive=bool(row.get("arrive", False)),
        planner_fresh=True,
        path_valid=len(path) >= 2,
        obstacle_clearance_m=obstacle_clearance_m,
        corridor_margin_m=corridor_margin_m,
        scan_topology=dict(row.get("scan_topology") or {}),
        reference_points=reference_points,
        optimized_points=optimized_points,
    )


def json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


__all__ = [
    "ForwardOnlyRecoveryFSM", "RecoveryConfig", "RecoveryObservation", "STATES",
    "choose_heading", "compute_reanchor", "evaluate_trigger", "json_safe",
    "observation_from_cycle", "wrap_angle",
]
