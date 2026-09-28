"""R1-specific command safety gate for the N2-PRE admission experiments.

This module is deliberately outside the NeuPAN wrapper.  It owns the only
ROS command path to Gazebo during admission and fails closed when command,
sensor, transform, timing, or bridge-health evidence is missing.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from r1_neupan_integration.n2_3_contract import FreshnessTracker


N116_TIMING_FIELDS = (
    "schema", "cycle_id", "scan_counter", "scan_stamp_sec",
    "scan_receive_monotonic", "path_version", "goal_version", "tf_stamp_sec",
    "cycle_start_monotonic", "worker_core_start_monotonic",
    "worker_core_end_monotonic", "output_publish_monotonic",
)
NO_REVERSE_POLICY_ACTIVE = True
FAILURE_CODES = {
    "bridge_closed", "bridge_exception", "worker_crash", "worker_timeout",
    "worker_response_failure", "malformed_response", "request_id_mismatch",
    "generation_mismatch", "path_version_mismatch", "goal_version_mismatch",
    "stale_response", "nan_inf", "timeout", "pipe_closed",
}


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _json_finite(value: Any) -> Any:
    """Convert non-finite diagnostic values to JSON null.

    Diagnostic topics cross the C++ controller boundary, where `Infinity` is
    not valid JSON.  Missing samples remain explicit through null while the
    watchdog reasons continue to report the fail-closed condition.
    """
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_finite(item) for item in value]
    return value


@dataclass(frozen=True)
class GateDecision:
    linear: float
    angular: float
    reasons: tuple[str, ...]
    clamped: bool
    zeroed: bool
    ages: dict[str, float]
    negative_input: bool = False
    negative_clamped: bool = False


class SafetyGateCore:
    """Pure, deterministic state machine used by the ROS gate and tests."""

    def __init__(
        self,
        *,
        max_linear: float = 0.15,
        max_angular: float = 0.4,
        max_linear_acceleration: float = 0.30,
        max_angular_acceleration: float = 0.8,
        command_timeout: float = 0.30,
        scan_timeout: float = 0.30,
        odom_timeout: float = 0.30,
        tf_timeout: float = 0.30,
        timing_timeout: float = 0.50,
        require_timing: bool = True,
        command_mode: str = "legacy_twist",
        relay_timeout: float | None = None,
        planner_freshness_timeout: float = 0.30,
    ) -> None:
        limits = (max_linear, max_angular, max_linear_acceleration,
                  max_angular_acceleration)
        if any(not _finite(value) or value <= 0.0 for value in limits):
            raise ValueError("safety limits must be finite and positive")
        relay_timeout = command_timeout if relay_timeout is None else relay_timeout
        timeouts = (command_timeout, relay_timeout, planner_freshness_timeout,
                    scan_timeout, odom_timeout, tf_timeout, timing_timeout)
        if any(not _finite(value) or value <= 0.0 for value in timeouts):
            raise ValueError("watchdog timeouts must be finite and positive")
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.max_linear_acceleration = float(max_linear_acceleration)
        self.max_angular_acceleration = float(max_angular_acceleration)
        self.command_timeout = float(command_timeout)
        self.relay_timeout = float(relay_timeout)
        self.planner_freshness_timeout = float(planner_freshness_timeout)
        if command_mode not in {"legacy_twist", "freshness_envelope"}:
            raise ValueError("command_mode must be legacy_twist or freshness_envelope")
        self.command_mode = str(command_mode)
        self.scan_timeout = float(scan_timeout)
        self.odom_timeout = float(odom_timeout)
        self.tf_timeout = float(tf_timeout)
        self.timing_timeout = float(timing_timeout)
        self.require_timing = bool(require_timing)
        self.emergency_stop = True
        self.failure_latched = False
        self.failure_reason = "emergency_stop"
        self.current_linear = 0.0
        self.current_angular = 0.0
        self._last_now = None
        self.last_step = None

    def engage(self, reason: str = "emergency_stop") -> None:
        self.emergency_stop = True
        self.failure_reason = str(reason)
        self.current_linear = 0.0
        self.current_angular = 0.0

    def release(self) -> None:
        self.emergency_stop = False

    def latch_failure(self, reason: str) -> None:
        self.failure_latched = True
        self.failure_reason = str(reason)
        self.current_linear = 0.0
        self.current_angular = 0.0

    def clear_failure(self) -> None:
        self.failure_latched = False
        self.failure_reason = ""

    @staticmethod
    def _age(now: float, seen: float) -> float:
        if seen < 0.0:
            return float("inf")
        return max(0.0, float(now) - float(seen))

    def step(
        self,
        now: float,
        linear: float,
        angular: float,
        *,
        command_seen: float,
        scan_seen: float,
        odom_seen: float,
        tf_seen: float,
        timing_seen: float,
        relay_seen: float | None = None,
        planner_result_seen: float = -1.0,
        metadata_fault: str | None = None,
        path_version_mismatch: bool = False,
        bridge_failure: bool = False,
        bridge_failure_reason: str = "bridge_failure",
    ) -> GateDecision:
        ages = {
            "command": self._age(now, command_seen),
            "relay": self._age(now, command_seen if relay_seen is None else relay_seen),
            "planner_result": self._age(now, planner_result_seen),
            "scan": self._age(now, scan_seen),
            "odom": self._age(now, odom_seen),
            "tf": self._age(now, tf_seen),
            "timing": self._age(now, timing_seen),
        }
        reasons: list[str] = []
        if self.emergency_stop:
            reasons.append("emergency_stop")
        if self.failure_latched:
            reasons.append(self.failure_reason or "failure_latched")
        elif bridge_failure:
            reasons.append(bridge_failure_reason or "bridge_failure")
        if self.command_mode == "freshness_envelope":
            if ages["relay"] > self.relay_timeout:
                reasons.append("relay_watchdog")
            if ages["planner_result"] > self.planner_freshness_timeout:
                reasons.append("planner_freshness_watchdog")
            if metadata_fault:
                reasons.append(str(metadata_fault))
        elif ages["command"] > self.command_timeout:
            reasons.append("command_watchdog")
        if ages["scan"] > self.scan_timeout:
            reasons.append("scan_watchdog")
        if ages["odom"] > self.odom_timeout:
            reasons.append("odom_watchdog")
        if ages["tf"] > self.tf_timeout:
            reasons.append("tf_watchdog")
        if self.require_timing and ages["timing"] > self.timing_timeout:
            reasons.append("timing_watchdog")
        if path_version_mismatch:
            reasons.append("path_version_mismatch")
        finite_command = _finite(linear) and _finite(angular)
        negative_input = finite_command and float(linear) < 0.0
        if not finite_command:
            reasons.append("nonfinite_command")

        # Once the operator has released the e-stop, a freshness or command
        # integrity fault is latched.  Startup remains represented by the
        # ordinary e-stop state, so stale initial samples cannot make release
        # permanently impossible before the health checkpoint runs.
        latch_reasons = {
            "command_watchdog", "scan_watchdog", "odom_watchdog",
            "tf_watchdog", "timing_watchdog", "nonfinite_command",
            "path_version_mismatch", "relay_watchdog", "planner_freshness_watchdog",
            "metadata_fault", "generation_mismatch", "goal_version_mismatch",
            "source_identity_mismatch", "stale_result", "envelope_schema_mismatch",
            "duplicate_command_mismatch", "source_timestamp_regressed",
            "nonfinite_source", "context_schema_mismatch", "context_version_malformed",
        }
        if not self.emergency_stop and any(
            reason in latch_reasons or reason.endswith("_mismatch") or reason.endswith("_watchdog")
            for reason in reasons
        ):
            fault = next(reason for reason in reasons if (
                reason in latch_reasons or reason.endswith("_mismatch") or reason.endswith("_watchdog")
            ))
            self.latch_failure(fault)

        if reasons:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self._last_now = float(now)
            decision = GateDecision(
                0.0, 0.0, tuple(dict.fromkeys(reasons)), negative_input, True,
                ages, negative_input, negative_input
            )
            self.last_step = decision
            return decision

        bounded_linear = max(-self.max_linear, min(self.max_linear, float(linear)))
        # This is the actuator-bound policy.  It is intentionally not a
        # parameter: planner, gear, and test configuration cannot disable it.
        target_linear = max(0.0, bounded_linear)
        target_angular = max(-self.max_angular, min(self.max_angular, float(angular)))
        clamped = (target_linear != float(linear) or target_angular != float(angular))
        # A planner zero is an explicit stop/arrive result.  Do not retain a
        # non-zero command while the acceleration limiter ramps down.
        if target_linear == 0.0 and target_angular == 0.0:
            self.current_linear = 0.0
            self.current_angular = 0.0
        else:
            dt = 0.0 if self._last_now is None else max(0.0, float(now) - self._last_now)
            # The previous timestamp is kept outside the public age envelope.
            if dt <= 0.0 or dt > 1.0:
                dt = min(self.command_timeout, 0.02)
            linear_delta = self.max_linear_acceleration * dt
            angular_delta = self.max_angular_acceleration * dt
            self.current_linear += max(-linear_delta, min(linear_delta, target_linear - self.current_linear))
            self.current_angular += max(-angular_delta, min(angular_delta, target_angular - self.current_angular))
        self._last_now = float(now)
        decision = GateDecision(
            self.current_linear,
            self.current_angular,
            tuple(dict.fromkeys(reasons)),
            clamped,
            False,
            ages,
            negative_input,
            negative_input and target_linear == 0.0,
        )
        self.last_step = decision
        return decision


class R1SafetyGate(Node):
    """Fail-closed relay from ``/neupan_cmd_vel`` to Gazebo ``/cmd_vel``."""

    def __init__(self) -> None:
        super().__init__("r1_n2_pre_safety_gate")
        self._lock = threading.Lock()
        self.declare_parameter("input_topic", "/neupan_cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan_neupan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("timing_topic", "/neupan_timing")
        self.declare_parameter("bridge_status_topic", "/neupan_bridge_status")
        self.declare_parameter("planner_context_topic", "/neupan_planner_context")
        self.declare_parameter("planner_source_topic", "/neupan_planner_result")
        self.declare_parameter("diagnostics_topic", "/r1_safety_gate_status")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_linear", 0.15)
        self.declare_parameter("max_angular", 0.4)
        self.declare_parameter("max_linear_acceleration", 0.30)
        self.declare_parameter("max_angular_acceleration", 0.8)
        self.declare_parameter("command_timeout", 0.30)
        self.declare_parameter("command_mode", "legacy_twist")
        self.declare_parameter("relay_timeout", 0.30)
        self.declare_parameter("planner_freshness_timeout", 0.30)
        self.declare_parameter("scan_timeout", 0.30)
        self.declare_parameter("odom_timeout", 0.30)
        self.declare_parameter("tf_timeout", 0.30)
        self.declare_parameter("timing_timeout", 0.50)
        self.declare_parameter("require_timing", True)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("emergency_stop", True)

        self.core = SafetyGateCore(
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            max_linear_acceleration=float(self.get_parameter("max_linear_acceleration").value),
            max_angular_acceleration=float(self.get_parameter("max_angular_acceleration").value),
            command_timeout=float(self.get_parameter("command_timeout").value),
            scan_timeout=float(self.get_parameter("scan_timeout").value),
            odom_timeout=float(self.get_parameter("odom_timeout").value),
            tf_timeout=float(self.get_parameter("tf_timeout").value),
            timing_timeout=float(self.get_parameter("timing_timeout").value),
            require_timing=bool(self.get_parameter("require_timing").value),
            command_mode=str(self.get_parameter("command_mode").value),
            relay_timeout=float(self.get_parameter("relay_timeout").value),
            planner_freshness_timeout=float(self.get_parameter("planner_freshness_timeout").value),
        )
        if not bool(self.get_parameter("emergency_stop").value):
            self.core.release()
        self.add_on_set_parameters_callback(self._parameter_callback)
        self._last_command = (0.0, 0.0)
        self._command_seen = -1.0
        self._command_receive_monotonic = -1.0
        self._relay_seen = -1.0
        self._relay_receive_monotonic = -1.0
        self._planner_result_seen = -1.0
        self._source_metadata = {}
        self._metadata_fault = None
        self._freshness_tracker = FreshnessTracker()
        self._source_freshness_tracker = FreshnessTracker()
        self._scan_seen = -1.0
        self._scan_receive_monotonic = -1.0
        self._odom_seen = -1.0
        self._odom_receive_monotonic = -1.0
        self._tf_seen = -1.0
        self._tf_receive_monotonic = -1.0
        self._timing_seen = -1.0
        self._timing_receive_monotonic = -1.0
        self._path_version_mismatch = False
        self._bridge_failure = False
        self._bridge_failure_reason = ""
        self._samples = 0
        self._clamp_count = 0
        self._zero_count = 0
        self._negative_linear_input_count = 0
        self._negative_linear_clamp_count = 0
        self._minimum_upstream_linear_x = math.inf
        self._minimum_gated_linear_x = math.inf
        self._last_reason = ("emergency_stop",)

        out_topic = str(self.get_parameter("output_topic").value)
        self.cmd_pub = self.create_publisher(Twist, out_topic, 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("diagnostics_topic").value), 50
        )
        command_topic = str(self.get_parameter("input_topic").value)
        command_mode = str(self.get_parameter("command_mode").value)
        if command_mode == "freshness_envelope":
            self.create_subscription(String, command_topic, self._envelope_callback, 20)
        else:
            self.create_subscription(Twist, command_topic, self._command_callback, 20)
        self.create_subscription(
            String, str(self.get_parameter("planner_context_topic").value),
            self._context_callback, 20,
        )
        self.create_subscription(
            String, str(self.get_parameter("planner_source_topic").value),
            self._planner_source_callback, 50,
        )
        sensor_qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value),
            self._scan_callback, sensor_qos
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._odom_callback, 20
        )
        self.create_subscription(
            TFMessage, str(self.get_parameter("tf_topic").value),
            self._tf_callback, 50
        )
        self.create_subscription(
            Float64MultiArray, str(self.get_parameter("timing_topic").value),
            self._timing_callback, 50
        )
        self.create_subscription(
            String, str(self.get_parameter("bridge_status_topic").value),
            self._bridge_status_callback, 50
        )
        self.reset_srv = self.create_service(SetBool, "~/emergency_stop", self._emergency_service)
        hz = float(self.get_parameter("publish_hz").value)
        if hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        self.timer = self.create_timer(1.0 / hz, self._publish_step)
        self.get_logger().info(
            "N2-PRE safety gate ready: emergency_stop=%s, no_reverse=%s, "
            "limits=(%.3f m/s, %.3f rad/s)"
            % (self.core.emergency_stop, NO_REVERSE_POLICY_ACTIVE,
               self.core.max_linear, self.core.max_angular)
        )

    @staticmethod
    def _parameter_callback(parameters) -> SetParametersResult:
        """Keep the operator e-stop transition on its audited service API."""
        if any(parameter.name == "emergency_stop" for parameter in parameters):
            return SetParametersResult(
                successful=False,
                reason=(
                    "emergency_stop is launch-time only; use "
                    "~/emergency_stop SetBool service"
                ),
            )
        return SetParametersResult(successful=True)

    def _command_callback(self, message: Twist) -> None:
        now = time.monotonic()
        with self._lock:
            self._last_command = (float(message.linear.x), float(message.angular.z))
            self._command_seen = now
            self._command_receive_monotonic = now

    def _envelope_callback(self, message: String) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            accepted, reason = self._freshness_tracker.accept(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            accepted, reason = False, "envelope_decode_error"
            self.get_logger().error("N2.3 envelope decode failed: %s", exc)
        with self._lock:
            self._relay_seen = now
            self._relay_receive_monotonic = now
            self._command_receive_monotonic = now
            if accepted:
                command = payload["command"]
                self._last_command = (
                    float(command["linear_x"]), float(command["angular_z"])
                )
                self._command_seen = now
                self._source_metadata = {
                        field: payload[field] for field in (
                            "worker_generation", "request_id", "cycle_id", "scan_counter",
                            "planner_result_monotonic", "path_version", "goal_version",
                            "relay_sequence", "relay_publish_monotonic",
                            "relay_source_receive_monotonic",
                        )
                }
                if reason == "new_source_result":
                    self._metadata_fault = None
            else:
                self._metadata_fault = str(reason)

    def _planner_source_callback(self, message: String) -> None:
        """Track planner source freshness independently of the relay stream."""
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            accepted, reason = self._source_freshness_tracker.accept(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            accepted, reason = False, "planner_source_decode_error"
            self.get_logger().error("N2.3 planner source decode failed: %s", exc)
        with self._lock:
            if accepted:
                self._planner_result_seen = float(payload["planner_result_monotonic"])
                if reason == "new_source_result":
                    self._source_metadata.update({
                        field: payload[field] for field in (
                            "worker_generation", "request_id", "cycle_id", "scan_counter",
                            "planner_result_monotonic", "path_version", "goal_version",
                        )
                    })
                    if self._metadata_fault in {None, "duplicate_source_result"}:
                        self._metadata_fault = None
            else:
                self._metadata_fault = str(reason)

    def _context_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            reason = self._freshness_tracker.set_context(payload)
            self._source_freshness_tracker.set_context(payload)
        except (TypeError, ValueError, json.JSONDecodeError):
            reason = "context_decode_error"
        with self._lock:
            self._metadata_fault = None if reason == "context_updated" else reason
            if reason == "context_updated":
                current_path = self._freshness_tracker.path_version
                current_goal = self._freshness_tracker.goal_version
                expected_path = self._freshness_tracker.expected_path_version
                expected_goal = self._freshness_tracker.expected_goal_version
                if current_path is not None and current_path != expected_path:
                    self._metadata_fault = "path_version_mismatch"
                elif current_goal is not None and current_goal != expected_goal:
                    self._metadata_fault = "goal_version_mismatch"

    def _scan_callback(self, _message: LaserScan) -> None:
        now = time.monotonic()
        with self._lock:
            self._scan_seen = now
            self._scan_receive_monotonic = now

    def _odom_callback(self, _message: Odometry) -> None:
        now = time.monotonic()
        with self._lock:
            self._odom_seen = now
            self._odom_receive_monotonic = now

    def _tf_callback(self, message: TFMessage) -> None:
        base_frame = str(self.get_parameter("base_frame").value)
        if any(transform.child_frame_id == base_frame for transform in message.transforms):
            now = time.monotonic()
            with self._lock:
                self._tf_seen = now
                self._tf_receive_monotonic = now

    def _timing_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < len(N116_TIMING_FIELDS):
            return
        now = time.monotonic()
        with self._lock:
            self._timing_seen = now
            self._timing_receive_monotonic = now

    def _bridge_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self.core.latch_failure("malformed_bridge_status")
            with self._lock:
                self._bridge_failure = True
                self._bridge_failure_reason = "malformed_bridge_status"
            return
        code = str(payload.get("code", ""))
        status = str(payload.get("status", ""))
        if status == "fail-safe" or code in FAILURE_CODES:
            reason = code or "bridge_fail_safe"
            self.core.latch_failure(reason)
            with self._lock:
                self._bridge_failure = True
                self._bridge_failure_reason = reason

    def _emergency_service(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        now = time.monotonic()
        if request.data:
            self.core.engage("operator_emergency_stop")
            response.success = True
            response.message = "emergency stop latched"
            return response
        with self._lock:
            ages = {
                "scan": self.core._age(now, self._scan_seen),
                "odom": self.core._age(now, self._odom_seen),
                "tf": self.core._age(now, self._tf_seen),
            }
            if self.core.command_mode == "freshness_envelope":
                ages["relay"] = self.core._age(now, self._relay_seen)
                ages["planner_result"] = self.core._age(now, self._planner_result_seen)
        timeout_by_signal = {
            "scan": self.core.scan_timeout,
            "odom": self.core.odom_timeout,
            "tf": self.core.tf_timeout,
            "relay": self.core.relay_timeout,
            "planner_result": self.core.planner_freshness_timeout,
        }
        blocked = [name for name, age in ages.items()
                   if age > timeout_by_signal[name]]
        with self._lock:
            if self.core.require_timing and self.core._age(now, self._timing_seen) > self.core.timing_timeout:
                blocked.append("timing")
            if self._metadata_fault:
                blocked.append(str(self._metadata_fault))
            if self._path_version_mismatch:
                blocked.append("path_version_mismatch")
            if self._bridge_failure:
                blocked.append(self._bridge_failure_reason or "bridge_failure")
            if not all(_finite(value) for value in self._last_command):
                blocked.append("nonfinite_command")
        if blocked:
            response.success = False
            response.message = "release denied: " + ",".join(dict.fromkeys(blocked))
            return response
        # A failure latch never clears on its own. A fresh explicit operator
        # release is the sole reset path, after the complete live health check
        # above has passed.
        self.core.clear_failure()
        self.core.release()
        response.success = True
        response.message = "emergency stop released"
        return response

    def _publish_step(self) -> None:
        now = time.monotonic()
        with self._lock:
            linear, angular = self._last_command
            values = {
                "command_seen": self._command_seen,
                "scan_seen": self._scan_seen,
                "odom_seen": self._odom_seen,
                "tf_seen": self._tf_seen,
                "timing_seen": self._timing_seen,
                "relay_seen": self._relay_seen,
                "planner_result_seen": self._planner_result_seen,
                "metadata_fault": self._metadata_fault,
                "path_version_mismatch": self._path_version_mismatch,
                "bridge_failure": self._bridge_failure,
                "bridge_failure_reason": self._bridge_failure_reason,
            }
            callback_receive = {
                "command_receive_monotonic": self._command_receive_monotonic,
                "scan_receive_monotonic": self._scan_receive_monotonic,
                "odom_receive_monotonic": self._odom_receive_monotonic,
                "tf_receive_monotonic": self._tf_receive_monotonic,
                "timing_receive_monotonic": self._timing_receive_monotonic,
                "relay_receive_monotonic": self._relay_receive_monotonic,
            }
        decision = self.core.step(now, linear, angular, **values)
        self._samples += 1
        if decision.clamped:
            self._clamp_count += 1
        if decision.zeroed:
            self._zero_count += 1
        if decision.negative_input:
            self._negative_linear_input_count += 1
        if decision.negative_clamped:
            self._negative_linear_clamp_count += 1
        if math.isfinite(linear):
            self._minimum_upstream_linear_x = min(
                self._minimum_upstream_linear_x, linear
            )
        if math.isfinite(decision.linear):
            self._minimum_gated_linear_x = min(
                self._minimum_gated_linear_x, decision.linear
            )
        self._last_reason = decision.reasons
        command = Twist()
        command.linear.x = float(decision.linear)
        command.angular.z = float(decision.angular)
        self.cmd_pub.publish(command)
        output_publish_monotonic = time.monotonic()
        diagnostics = {
            "schema": "r1_n2_pre_safety_gate_v2",
            "monotonic": now,
            "publish_step_monotonic": now,
            "output_publish_monotonic": output_publish_monotonic,
            "callback_receive_monotonic": {
                key: callback_receive[key] for key in (
                    "command_receive_monotonic", "scan_receive_monotonic",
                    "odom_receive_monotonic", "tf_receive_monotonic",
                    "timing_receive_monotonic", "relay_receive_monotonic",
                )
            },
            "no_reverse_policy_active": NO_REVERSE_POLICY_ACTIVE,
            "emergency_stop": self.core.emergency_stop,
            "failure_latched": self.core.failure_latched,
            "input_linear_x": linear,
            "input_angular_z": angular,
            "output_linear_x": decision.linear,
            "output_angular_z": decision.angular,
            "reasons": list(decision.reasons),
            "clamped": decision.clamped,
            "zeroed": decision.zeroed,
            "ages_sec": {key: value for key, value in decision.ages.items() if key != "_now"},
            "watchdog_limits_sec": {
                "command": self.core.command_timeout,
                "relay": self.core.relay_timeout,
                "planner_result": self.core.planner_freshness_timeout,
                "scan": self.core.scan_timeout,
                "odom": self.core.odom_timeout,
                "tf": self.core.tf_timeout,
                "timing": self.core.timing_timeout,
            },
            "watchdog_triggered": [
                reason for reason in decision.reasons if reason.endswith("_watchdog")
            ],
            "limits": {
                "max_linear": self.core.max_linear,
                "max_angular": self.core.max_angular,
                "max_linear_acceleration": self.core.max_linear_acceleration,
                "max_angular_acceleration": self.core.max_angular_acceleration,
            },
            "counts": {
                "samples": self._samples,
                "clamp": self._clamp_count,
                "zero": self._zero_count,
                "negative_linear_input_count": self._negative_linear_input_count,
                "negative_linear_clamp_count": self._negative_linear_clamp_count,
            },
            "freshness": {
                "mode": self.core.command_mode,
                "source_metadata": dict(self._source_metadata),
                "metadata_fault": self._metadata_fault,
                "relay_sequence": (
                    self._source_metadata.get("relay_sequence")
                    if self._source_metadata else None
                ),
            },
            "minimum_upstream_linear_x": (
                self._minimum_upstream_linear_x
                if math.isfinite(self._minimum_upstream_linear_x) else None
            ),
            "minimum_gated_linear_x": (
                self._minimum_gated_linear_x
                if math.isfinite(self._minimum_gated_linear_x) else None
            ),
        }
        message = String()
        message.data = json.dumps(_json_finite(diagnostics), sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = R1SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # rclpy can invalidate a subscription handle during launch-wide
        # shutdown. Treat only that known teardown race as normal exit.
        if "Unable to convert call argument" not in str(error):
            raise
    finally:
        node.core.engage("shutdown")
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            # The launch shutdown can invalidate the ROS context before this
            # final best-effort zero reaches the publisher.
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
