"""N2-PRE.3 closed-loop motion, stop and fault evidence collector."""

from __future__ import annotations

import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Int8, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from r1_neupan_integration.n2_pre_scenarios import n2_scenario_spec
from r1_neupan_integration.n2_4c_runtime_attribution import RuntimeAttributionCollector


TIMING_FIELDS = (
    "schema", "cycle_id", "scan_counter", "scan_stamp_sec",
    "scan_receive_monotonic", "path_version", "goal_version", "tf_stamp_sec",
    "cycle_start_monotonic", "worker_core_start_monotonic",
    "worker_core_end_monotonic", "output_publish_monotonic",
    "main_input_extraction_ms", "main_request_prep_ms",
    "main_request_serialization_ms", "main_request_send_ms", "main_wait_ms",
    "main_response_receive_ms", "main_reconstruction_ms", "main_ipc_roundtrip_ms",
    "main_publish_ms", "main_full_cycle_ms", "worker_decode_input_ms",
    "worker_dune_ms", "worker_nrmp_ms", "worker_cvxpy_ms", "worker_pan_ms",
    "worker_core_total_ms", "worker_core_cpu_ms", "worker_result_pack_ms",
    "worker_result_serialization_ms", "worker_response_send_ms",
    "obstacle_points_count", "worker_pan_iterations",
    "worker_pan_early_converged", "worker_dune_effective_points_count",
    "worker_nrmp_effective_points_count", "dropped_obsolete_scans",
    "duplicate_scan", "queue_depth", "request_id", "worker_sequence",
    "worker_pid", "worker_generation", "worker_failure", "finite", "nan_inf",
    "stale_response",
)


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _finite_stats(values: list[float]) -> dict[str, float | int | None]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count": 0, "total": len(values), "nonfinite": len(values)}
    ordered = sorted(finite)
    return {
        "count": len(finite),
        "total": len(values),
        "nonfinite": len(values) - len(finite),
        "min": min(finite),
        "max": max(finite),
        "mean": statistics.fmean(finite),
        "p95": ordered[int(0.95 * (len(ordered) - 1))],
        "p99": ordered[int(0.99 * (len(ordered) - 1))],
    }


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _local_to_world(start: list[float], point: tuple[float, float]) -> list[float]:
    c, s = math.cos(start[3]), math.sin(start[3])
    return [start[1] + c * point[0] - s * point[1],
            start[2] + s * point[0] + c * point[1]]


def _point_segment_distance(point: list[float], left: list[float], right: list[float]) -> float:
    dx, dy = right[0] - left[0], right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - left[0], point[1] - left[1])
    scale = ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq
    scale = max(0.0, min(1.0, scale))
    return math.hypot(point[0] - (left[0] + scale * dx),
                      point[1] - (left[1] + scale * dy))


def _polyline_distance(point: list[float], path: list[list[float]]) -> float:
    if len(path) < 2:
        return float("nan")
    return min(_point_segment_distance(point, path[index], path[index + 1])
               for index in range(len(path) - 1))


class N2Pre3Audit(Node):
    """Own the release/latch checkpoint and capture every safety-relevant stream."""

    def __init__(self) -> None:
        super().__init__("r1_n2_pre3_audit")
        self.declare_parameter("scenario", "m1_forward")
        self.declare_parameter("output_file", "n2_pre_results/n2p3.json")
        self.declare_parameter("duration_sec", 20.0)
        self.declare_parameter("health_hold_sec", 1.0)
        self.declare_parameter("min_motion_sec", 2.0)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("post_latch_sec", 1.0)
        self.declare_parameter("velocity_epsilon", 0.01)
        self.declare_parameter("auto_release", True)
        self.declare_parameter("auto_latch", True)
        self.declare_parameter("stop_on_arrival", True)
        self.declare_parameter("capture_detail", True)
        self.declare_parameter("runtime_attribution_file", "")
        self.declare_parameter("runtime_run_label", "unknown")
        self._started = time.monotonic()
        self._health_since = None
        self._release_request = None
        self._release_response = None
        self._release_done = None
        self._latch_request = None
        self._latch_response = None
        self._finish_after = None
        self._arrive_first = None
        self._arrival_times = []
        self._stop_first = None
        self._last_nonzero = None
        self._zero_after_arrive = None
        self._robot_zero_since = None
        self._last_gate = None
        self._bridge_events = []
        self._raw = []
        self._gated = []
        self._odom = []
        self._scan = []
        self._scan_detail = []
        self._tf_seen = []
        self._timing = []
        self._semantics = []
        self._contacts = []
        self._capture_detail = bool(self.get_parameter("capture_detail").value)
        self._scan_count = 0
        self._scan_minimum = None
        self._gear = []
        self._gate = []
        self._records = []
        self._envelope_source = []
        self._envelope_relay = []
        self._finished = False
        self._scenario_spec = n2_scenario_spec(str(self.get_parameter("scenario").value))
        self._service = self.create_client(SetBool, "/r1_n2_pre_safety_gate/emergency_stop")
        self.create_subscription(Twist, "/neupan_cmd_vel", self._raw_cb, 50)
        self.create_subscription(Twist, "/cmd_vel", self._gated_cb, 50)
        self.create_subscription(String, "/r1_safety_gate_status", self._gate_cb, 50)
        self.create_subscription(String, "/neupan_bridge_status", self._bridge_cb, 50)
        self.create_subscription(String, "/neupan_semantics", self._semantic_cb, 50)
        self.create_subscription(String, "/neupan_planner_result", self._source_envelope_cb, 50)
        self.create_subscription(String, "/neupan_cmd_envelope", self._relay_envelope_cb, 50)
        self.create_subscription(Int8, "/r1_motion_gear", self._gear_cb, 20)
        sensor_qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/scan_neupan", self._scan_cb, sensor_qos)
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, sensor_qos)
        self.create_subscription(Float64MultiArray, "/neupan_timing", self._timing_cb, 50)
        self.create_subscription(Contacts, "/world/empty/contacts", self._contacts_cb, 20)
        runtime_output = str(self.get_parameter("runtime_attribution_file").value)
        self._runtime_collector = (
            RuntimeAttributionCollector(
                self,
                runtime_output,
                str(self.get_parameter("runtime_run_label").value),
            )
            if runtime_output else None
        )
        # Use a steady clock so the collector cannot stall when a test is run
        # before /clock is available or after Gazebo is stopped.
        self.timer = self.create_timer(
            0.02, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _t(self) -> float:
        return time.monotonic() - self._started

    def _raw_cb(self, msg: Twist) -> None:
        now = self._t()
        self._raw.append([now, float(msg.linear.x), float(msg.angular.z), time.monotonic()])

    def _gated_cb(self, msg: Twist) -> None:
        now = self._t()
        linear, angular = float(msg.linear.x), float(msg.angular.z)
        self._gated.append([now, linear, angular, time.monotonic()])
        nonzero = abs(linear) > 1e-5 or abs(angular) > 1e-5
        if nonzero:
            self._last_nonzero = now
            self._zero_after_arrive = None
        elif self._arrive_first is not None and now >= self._arrive_first:
            if self._zero_after_arrive is None:
                self._zero_after_arrive = now

    def _gate_cb(self, msg: String) -> None:
        now = self._t()
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"malformed": True}
        self._last_gate = value
        self._gate.append([now, value, time.monotonic()])

    def _bridge_cb(self, msg: String) -> None:
        now = self._t()
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"malformed": True}
        self._bridge_events.append([now, value])

    def _semantic_cb(self, msg: String) -> None:
        now = self._t()
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"malformed": True}
        info = value.get("info", {}) if isinstance(value, dict) else {}
        if (info.get("arrive") is True and self._arrive_first is None
                and self._release_done is not None and now >= self._release_done):
            self._arrive_first = now
            self._arrival_times.append(now)
        if (info.get("stop") is True and self._stop_first is None
                and self._release_done is not None and now >= self._release_done):
            self._stop_first = now
        if not self._capture_detail:
            return
        self._semantics.append([now, value, time.monotonic()])

    def _source_envelope_cb(self, msg: String) -> None:
        self._envelope_cb(msg, self._envelope_source)

    def _relay_envelope_cb(self, msg: String) -> None:
        self._envelope_cb(msg, self._envelope_relay)

    def _envelope_cb(self, msg: String, target: list) -> None:
        now = self._t()
        receive = time.monotonic()
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            payload = {"malformed": True, "raw": str(msg.data)}
        target.append({
            "audit_t": now,
            "receive_monotonic": receive,
            "payload": payload,
        })

    def _gear_cb(self, msg: Int8) -> None:
        self._gear.append([self._t(), int(msg.data)])

    def _scan_cb(self, msg: LaserScan) -> None:
        monotonic = time.monotonic()
        finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
        self._scan_count += 1
        if finite:
            minimum = min(finite)
            self._scan_minimum = minimum if self._scan_minimum is None else min(self._scan_minimum, minimum)
        if not self._capture_detail:
            return
        self._scan.append([self._t(), min(finite) if finite else None, len(finite)])
        left = right = front = 0
        min_left = min_right = min_front = None
        if msg.ranges:
            step = (float(msg.angle_max) - float(msg.angle_min)) / max(1, len(msg.ranges) - 1)
            for index, value in enumerate(msg.ranges):
                value = float(value)
                if not math.isfinite(value) or value <= float(msg.range_min) or value >= float(msg.range_max):
                    continue
                angle = float(msg.angle_min) + index * step
                if abs(angle) <= math.pi / 12.0:
                    front += 1
                    min_front = value if min_front is None else min(min_front, value)
                elif angle > 0.0:
                    left += 1
                    min_left = value if min_left is None else min(min_left, value)
                else:
                    right += 1
                    min_right = value if min_right is None else min(min_right, value)
        self._scan_detail.append({
            "t": self._t(), "monotonic": monotonic, "finite_count": len(finite),
            "front_count": front, "left_count": left, "right_count": right,
            "min_front": min_front, "min_left": min_left, "min_right": min_right,
        })

    def _odom_cb(self, msg: Odometry) -> None:
        now = self._t()
        pose = msg.pose.pose
        twist = msg.twist.twist
        linear = math.hypot(float(twist.linear.x), float(twist.linear.y))
        angular = abs(float(twist.angular.z))
        if linear <= float(self.get_parameter("velocity_epsilon").value) and angular <= float(self.get_parameter("velocity_epsilon").value):
            if self._robot_zero_since is None:
                self._robot_zero_since = now
        else:
            self._robot_zero_since = None
        self._odom.append([
            now, float(pose.position.x), float(pose.position.y), _yaw(pose.orientation),
            float(twist.linear.x), float(twist.linear.y), float(twist.angular.z),
        ])

    def _tf_cb(self, msg: TFMessage) -> None:
        if any(item.child_frame_id == "base_link" for item in msg.transforms):
            self._tf_seen.append(self._t())

    def _timing_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < len(TIMING_FIELDS):
            return
        self._timing.append([self._t(), dict(zip(TIMING_FIELDS, [float(v) for v in msg.data]))])

    def _contacts_cb(self, msg: Contacts) -> None:
        now = self._t()
        contacts = []
        for item in msg.contacts:
            names = (item.collision1.name, item.collision2.name)
            if any("r1_with_arm" in name or "wheel_" in name or "caster_" in name for name in names):
                contacts.append({
                    "collision1": names[0], "collision2": names[1],
                    "depths": [float(v) for v in item.depths],
                })
        if contacts:
            self._contacts.append([now, contacts])

    def _healthy(self) -> bool:
        gate = self._last_gate
        if not isinstance(gate, dict):
            return False
        if gate.get("emergency_stop") is not True or gate.get("failure_latched"):
            return False
        ages = gate.get("ages_sec", {})
        limits = {"scan": 0.30, "odom": 0.30, "tf": 0.30, "timing": 0.50}
        return all(math.isfinite(float(ages.get(key, math.inf))) and float(ages[key]) <= limit for key, limit in limits.items())

    def _call_stop(self, value: bool, kind: str) -> None:
        if not self._service.service_is_ready():
            return
        request = SetBool.Request()
        request.data = bool(value)
        future = self._service.call_async(request)
        stamp = self._t()
        if kind == "release":
            self._release_request = stamp
        else:
            self._latch_request = stamp

        def done(result_future):
            try:
                result = result_future.result()
                value_out = {"success": bool(result.success), "message": str(result.message)}
            except Exception as exc:
                value_out = {"success": False, "message": repr(exc)}
            value_out["t"] = self._t()
            if kind == "release":
                self._release_response = value_out
                if value_out.get("success"):
                    self._release_done = float(value_out["t"])
            else:
                self._latch_response = value_out

        future.add_done_callback(done)

    def _tick(self) -> None:
        now = self._t()
        if self._capture_detail:
            self._records.append({"t": now, "healthy": self._healthy(), "release_response": self._release_response, "latch_response": self._latch_response})
        if self._release_request is None and bool(self.get_parameter("auto_release").value):
            if self._healthy():
                self._health_since = now if self._health_since is None else self._health_since
                if now - self._health_since >= float(self.get_parameter("health_hold_sec").value):
                    self._call_stop(False, "release")
            else:
                self._health_since = None
        if self._release_response and not self._release_response.get("success"):
            self.get_logger().error("e-stop release failed: %s", self._release_response)
        released = self._release_done is not None and now >= self._release_done
        arrived_or_window = released and (
            self._arrive_first is not None
            or now - self._release_done >= float(self.get_parameter("min_motion_sec").value)
        )
        stable_zero = bool(self.get_parameter("stop_on_arrival").value) and (
            self._zero_after_arrive is not None
            and now - self._zero_after_arrive >= float(self.get_parameter("settle_sec").value)
        )
        timed_out = now >= float(self.get_parameter("duration_sec").value)
        should_latch = bool(self.get_parameter("auto_latch").value) and arrived_or_window and (stable_zero or timed_out)
        if self._latch_request is None and should_latch:
            self._call_stop(True, "latch")
        if timed_out and self._latch_request is None and self._finish_after is None:
            if bool(self.get_parameter("auto_latch").value) and self._service.service_is_ready():
                self._call_stop(True, "latch")
            else:
                self._write()
                self._finished = True
        if self._latch_response and self._finish_after is None:
            self._finish_after = now + float(self.get_parameter("post_latch_sec").value)
        if self._finish_after is not None and now >= self._finish_after:
            self._write()
            self.timer.cancel()
            self._finished = True

    @staticmethod
    def _envelope_stats(entries: list[dict]) -> dict:
        receive_times = [float(entry["receive_monotonic"]) for entry in entries]
        def source_stamp(entry: dict) -> float | None:
            try:
                value = float(entry.get("payload", {}).get("planner_result_monotonic", math.nan))
            except (TypeError, ValueError, AttributeError):
                return None
            return value if math.isfinite(value) else None
        source_times = [
            value
            for entry in entries
            for value in [source_stamp(entry)]
            if value is not None
        ]
        receive_intervals = [right - left for left, right in zip(receive_times, receive_times[1:])]
        source_intervals = [right - left for left, right in zip(source_times, source_times[1:])]
        identities = []
        relay_source_ages = []
        for entry in entries:
            payload = entry.get("payload", {})
            if not isinstance(payload, dict):
                continue
            identity = tuple(payload.get(field) for field in (
                "worker_generation", "request_id", "cycle_id", "scan_counter",
                "path_version", "goal_version",
            ))
            identities.append(identity)
            stamp = source_stamp(entry)
            if stamp is not None:
                relay_source_ages.append(float(entry["receive_monotonic"]) - stamp)
        duplicate_count = sum(left == right for left, right in zip(identities, identities[1:]))
        return {
            "count": len(entries),
            "receive_interval_sec": _finite_stats(receive_intervals),
            "source_interval_sec": _finite_stats(source_intervals),
            "source_age_sec_at_receive": _finite_stats(relay_source_ages),
            "duplicate_source_identity_count": duplicate_count,
            "malformed_count": sum(
                not isinstance(entry.get("payload"), dict)
                or entry.get("payload", {}).get("malformed", False)
                for entry in entries
            ),
            "first": entries[0] if entries else None,
            "last": entries[-1] if entries else None,
        }

    def _write(self) -> None:
        if self._runtime_collector is not None:
            self._runtime_collector.finalize()
        output = Path(str(self.get_parameter("output_file").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        semantic_arrivals = list(self._arrival_times)
        nonzero_raw = [math.hypot(row[1], row[2]) for row in self._raw]
        nonzero_gated = [math.hypot(row[1], row[2]) for row in self._gated]
        output_linear = [row[1] for row in self._gated]
        output_angular = [row[2] for row in self._gated]
        gate_status = [row[1] for row in self._gate if isinstance(row[1], dict)]
        negative_raw = [row[1] for row in self._raw if row[1] < 0.0]
        negative_gated = [row[1] for row in self._gated if row[1] < 0.0]
        worker = [row[1].get("worker_core_total_ms", math.nan) for row in self._timing]
        full = [
            (row[1].get("output_publish_monotonic", math.nan) - row[1].get("cycle_start_monotonic", math.nan)) * 1000.0
            for row in self._timing
        ]
        first_zero = self._zero_after_arrive
        spec = self._scenario_spec
        initial = self._odom[0] if self._odom else None
        path_world = None
        goal_world = None
        if spec is not None and initial is not None:
            path_world = [_local_to_world(initial, point) for point in spec["path"]]
            goal_xy = _local_to_world(initial, spec["path"][-1])
            goal_world = goal_xy + [initial[3] + float(spec["goal_yaw"])]
        post_release = [row for row in self._odom
                        if self._release_done is not None and row[0] >= self._release_done]
        final_pose = post_release[-1] if post_release else (self._odom[-1] if self._odom else None)
        trajectory_distances = []
        path_deviations = []
        if post_release:
            for previous, current in zip(post_release, post_release[1:]):
                trajectory_distances.append(math.hypot(current[1] - previous[1], current[2] - previous[2]))
            if path_world is not None:
                path_deviations = [_polyline_distance([row[1], row[2]], path_world)
                                   for row in post_release]
        position_error = None
        yaw_error = None
        if final_pose is not None and goal_world is not None:
            position_error = math.hypot(final_pose[1] - goal_world[0], final_pose[2] - goal_world[1])
            yaw_error = _wrap_angle(final_pose[3] - goal_world[2])
        active_angular = [row[2] for row in self._gated
                          if self._release_done is not None and row[0] >= self._release_done
                          and abs(row[2]) >= 0.03]
        signs = [1 if value > 0.0 else -1 for value in active_angular]
        oscillation_count = sum(left != right for left, right in zip(signs, signs[1:]))
        recent = []
        if post_release:
            recent = [row for row in post_release if row[0] >= post_release[-1][0] - 5.0]
        recent_displacement = None
        recent_goal_progress = None
        if len(recent) >= 2 and goal_world is not None:
            recent_displacement = math.hypot(recent[-1][1] - recent[0][1], recent[-1][2] - recent[0][2])
            recent_goal_progress = (math.hypot(recent[0][1] - goal_world[0], recent[0][2] - goal_world[1])
                                    - math.hypot(recent[-1][1] - goal_world[0], recent[-1][2] - goal_world[1]))
        recent_commands = [row for row in self._gated
                           if recent and recent[0][0] <= row[0] <= recent[-1][0]
                           and (abs(row[1]) > 0.01 or abs(row[2]) > 0.03)]
        stuck = bool(final_pose is not None and goal_world is not None
                     and position_error is not None and position_error > 0.6
                     and recent_displacement is not None and recent_displacement < 0.10
                     and (recent_goal_progress is None or recent_goal_progress < 0.05)
                     and (len(recent_commands) > 0 or self._arrive_first is None))
        cycle_evidence = []
        for stamp, semantic, _semantic_receive_monotonic in self._semantics:
            if not isinstance(semantic, dict) or "cycle_id" not in semantic:
                continue
            publish_monotonic = float(semantic.get("output_publish_monotonic", 0.0))
            gate_candidates = [item for item in self._gate
                               if isinstance(item[1], dict)
                               and math.isfinite(float(item[1].get("monotonic", math.nan)))]
            gate_value = None
            if gate_candidates and publish_monotonic > 0.0:
                gate_value = min(
                    gate_candidates,
                    key=lambda item: abs(float(item[1]["monotonic"]) - publish_monotonic),
                )[1]
            scan_value = None
            if self._scan_detail and publish_monotonic > 0.0:
                scan_value = min(
                    self._scan_detail,
                    key=lambda item: abs(float(item["monotonic"]) - publish_monotonic),
                )
            cycle_evidence.append({
                "audit_t": stamp,
                "cycle_id": int(semantic["cycle_id"]),
                "robot_pose": semantic.get("state"),
                "goal_distance": semantic.get("goal_distance"),
                "active_path_index": {
                    "point_index": semantic.get("point_index"),
                    "curve_index": semantic.get("curve_index"),
                },
                "reference_state": semantic.get("ref_state"),
                "optimized_state": semantic.get("opt_state"),
                "action": semantic.get("action"),
                "obstacle_point_count": semantic.get("obstacle_points_count"),
                "min_planner_distance": semantic.get("min_planner_distance"),
                "arrive": semantic.get("info", {}).get("arrive"),
                "stop": semantic.get("info", {}).get("stop"),
                "pan": semantic.get("pan"),
                "gated_cmd": None if gate_value is None else {
                    "linear_x": gate_value.get("output_linear_x"),
                    "angular_z": gate_value.get("output_angular_z"),
                    "clamp_count": gate_value.get("counts", {}).get("clamp"),
                    "negative_linear_clamp_count": gate_value.get("counts", {}).get("negative_linear_clamp_count"),
                    "reasons": gate_value.get("reasons", []),
                    "gate_monotonic": gate_value.get("monotonic"),
                    "publish_step_monotonic": gate_value.get("publish_step_monotonic"),
                    "callback_receive_monotonic": gate_value.get("callback_receive_monotonic"),
                    "ages_sec": gate_value.get("ages_sec"),
                    "watchdog_limits_sec": gate_value.get("watchdog_limits_sec"),
                    "watchdog_triggered": gate_value.get("watchdog_triggered", []),
                },
                "transport_trace": semantic.get("transport_trace"),
                "scan_topology": scan_value,
            })
        summary = {
            "schema": "r1_n2_pre3_motion_audit_v1",
            "scenario": str(self.get_parameter("scenario").value),
            "duration_sec": self._t(),
            "scenario_spec": spec,
            "limits": {"max_linear": 0.15, "max_angular": 0.40, "max_linear_acceleration": 0.30, "max_angular_acceleration": 0.80},
            "safety": {
                "release_request_t": self._release_request,
                "release_response": self._release_response,
                "release_done_t": self._release_done,
                "latch_request_t": self._latch_request,
                "latch_response": self._latch_response,
                "first_arrive_t": self._arrive_first,
                "first_stop_t": self._stop_first,
                "last_nonzero_gated_t": self._last_nonzero,
                "first_zero_after_arrive_t": first_zero,
                "arrive_to_zero_latency_sec": None if self._arrive_first is None or first_zero is None else first_zero - self._arrive_first,
                "robot_zero_since_t": self._robot_zero_since,
            },
            "raw_cmd": {"linear_x": _finite_stats([row[1] for row in self._raw]), "angular_z": _finite_stats([row[2] for row in self._raw]), "speed_norm": _finite_stats(nonzero_raw)},
            "gated_cmd": {"linear_x": _finite_stats(output_linear), "angular_z": _finite_stats(output_angular), "speed_norm": _finite_stats(nonzero_gated), "samples": len(self._gated)},
            "no_reverse": {
                "policy_active": all(item.get("no_reverse_policy_active") is True for item in gate_status) if gate_status else False,
                "negative_linear_input_count": len(negative_raw),
                "negative_linear_gated_count": len(negative_gated),
                "minimum_upstream_linear_x": min((row[1] for row in self._raw), default=None),
                "minimum_gated_linear_x": min((row[1] for row in self._gated), default=None),
                "gate_negative_linear_input_count": max((int(item.get("counts", {}).get("negative_linear_input_count", 0)) for item in gate_status), default=0),
                "gate_negative_linear_clamp_count": max((int(item.get("counts", {}).get("negative_linear_clamp_count", 0)) for item in gate_status), default=0),
            },
            "arrivals": {"count": len(semantic_arrivals), "first_t": semantic_arrivals[0] if semantic_arrivals else None},
            "odom": {"samples": len(self._odom), "start": self._odom[0] if self._odom else None, "end": self._odom[-1] if self._odom else None},
            "trajectory": self._odom,
            "commands": {"raw": self._raw, "gated": self._gated},
            "command_trace": {
                "raw": [
                    {"audit_t": row[0], "linear_x": row[1],
                     "angular_z": row[2], "monotonic": row[3]}
                    for row in self._raw
                ],
                "gated": [
                    {"audit_t": row[0], "linear_x": row[1],
                     "angular_z": row[2], "monotonic": row[3]}
                    for row in self._gated
                ],
            },
            "freshness": {
                "mode": "freshness_envelope" if self._envelope_relay else "legacy_twist",
                "planner_result": self._envelope_stats(self._envelope_source),
                "relay": self._envelope_stats(self._envelope_relay),
                "source_identity_changed_count": sum(
                    left.get("payload", {}).get("request_id")
                    != right.get("payload", {}).get("request_id")
                    for left, right in zip(self._envelope_relay, self._envelope_relay[1:])
                    if isinstance(left.get("payload"), dict) and isinstance(right.get("payload"), dict)
                ),
            },
            "command_envelope_trace": {
                "planner_result": self._envelope_source,
                "relay": self._envelope_relay,
            },
            "gate_trace": [
                {"audit_t": row[0], "audit_receive_monotonic": row[2],
                 "value": row[1]}
                for row in self._gate
            ],
            "goal": {"path_local": spec["path"] if spec else None,
                     "path_world": path_world,
                     "goal_world": goal_world,
                     "goal_yaw_local": spec["goal_yaw"] if spec else None},
            "navigation_metrics": {
                "final_position_error_m": position_error,
                "final_yaw_error_rad": yaw_error,
                "distance_travelled_m": sum(trajectory_distances),
                "path_deviation_m": _finite_stats(path_deviations),
                "oscillation_count": oscillation_count,
                "stuck": stuck,
                "recent_displacement_m": recent_displacement,
                "recent_goal_progress_m": recent_goal_progress,
                "timed_out_without_arrival": self._arrive_first is None,
            },
            "scan": {"samples": self._scan_count if not self._capture_detail else len(self._scan),
                     "minimum_range": self._scan_minimum if not self._capture_detail else min((row[1] for row in self._scan if row[1] is not None), default=None),
                     "sample_log": self._scan},
            "cycle_evidence": cycle_evidence,
            "contacts": {"message_count": len(self._contacts), "contact_count": sum(len(row[1]) for row in self._contacts), "events": self._contacts[:100]},
            "gear": self._gear,
            "gate": {"samples": len(self._gate), "clamp_count": sum(1 for row in self._gate if row[1].get("clamped")), "zeroed_count": sum(1 for row in self._gate if row[1].get("zeroed")), "reasons": sorted({reason for row in self._gate for reason in row[1].get("reasons", [])}), "last": self._gate[-1][1] if self._gate else None},
            "bridge_events": self._bridge_events,
            "timing": {"worker_core_total_ms": _finite_stats(worker), "main_full_cycle_ms": _finite_stats(full), "queue_depth_max": max((row[1].get("queue_depth", 0.0) for row in self._timing), default=None), "stale": sum(row[1].get("stale_response", 0.0) != 0.0 for row in self._timing), "failures": sum(row[1].get("worker_failure", 0.0) != 0.0 for row in self._timing), "nan_inf": sum(row[1].get("nan_inf", 0.0) != 0.0 for row in self._timing)},
            "semantics": self._semantics,
            "records": self._records,
            "capture_detail": self._capture_detail,
        }
        output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = N2Pre3Audit()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node._finished:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        if not Path(str(node.get_parameter("output_file").value)).exists():
            node._write()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
