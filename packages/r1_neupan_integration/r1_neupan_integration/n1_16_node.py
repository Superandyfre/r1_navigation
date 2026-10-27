"""ROS I/O node for N1.16 production-grade isolated planner validation."""

from __future__ import annotations

import json
import os
import signal
import threading
import time
from dataclasses import replace
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from r1_neupan_integration.n1_15_node import N115Node
from r1_neupan_integration.n1_16_bridge import (
    BridgeProtocolError,
    N116_SCHEMA,
    ProductionWholeCoreClient,
)
from r1_neupan_integration.n1_15_whole_core import same_terminal_pose
from r1_neupan_integration.command_stabilizer import (
    BidirectionalCommandStabilizer,
    CommandStabilizerConfig,
    GearTransitionConfig,
    GearTransitionGate,
)
from r1_neupan_integration.n2_3_contract import make_context, make_planner_envelope
from r1_neupan_integration.forward_only import ForwardOnlyConfig, ForwardOnlyController
from r1_neupan_integration.latest_input_scheduler import LatestInputScheduler
from r1_neupan_integration.local_detour import (
    LocalDetourController, LocalDetourPlanner, ReverseEscapePlanner,
)
from r1_neupan_integration.path_validation import (
    TemporalPathDeviationGuard,
    max_path_deviation_m as calculate_max_path_deviation_m,
    near_term_path_deviation_m as calculate_near_term_path_deviation_m,
)
from r1_neupan_integration.obstacle_motion_policy import (
    MotionPolicyConfig,
    first_command_collision_distance,
    first_swept_collision_distance,
    plan_obstacle_speed,
)
from r1_neupan_integration.stop_reverse_recovery import (
    StopReverseRecovery, stopped_reverse_velocity,
)


N116_TIMING_FIELDS = [
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
    "cloud_receive_monotonic", "scan_callback_receive_monotonic",
    "scan_update_monotonic", "request_created_monotonic",
    "request_dispatched_monotonic", "worker_request_received_monotonic",
    "worker_solve_start_monotonic", "worker_solve_end_monotonic",
    "worker_response_sent_monotonic", "main_response_received_monotonic",
    "main_validation_complete_monotonic", "planner_result_publish_monotonic",
    "relay_source_receive_monotonic", "relay_publish_monotonic",
    "gate_relay_receive_monotonic", "gate_output_publish_monotonic",
]


class N116Node(N115Node):
    """N1.15 ROS I/O surface with a stricter production bridge contract."""

    NODE_NAME = "n1_16_production_bridge"
    CLIENT_CLASS = ProductionWholeCoreClient

    def declare_client_parameters(self) -> None:
        self.declare_parameter("n116_fault_mode", "")
        self.declare_parameter("n116_fault_request_id", -1)
        self.declare_parameter("n116_fault_sleep_sec", 1.0)
        self.declare_parameter("bridge_status_topic", "/neupan_bridge_status")
        self.declare_parameter("semantic_topic", "/neupan_semantics")
        self.declare_parameter("planner_result_topic", "/neupan_planner_result")
        self.declare_parameter("planner_context_topic", "/neupan_planner_context")
        self.declare_parameter("forward_only_status_topic", "/neupan_forward_only_status")
        self.declare_parameter("forward_only_enabled", True)
        self.declare_parameter("forward_only_lookahead_m", 0.6)
        self.declare_parameter("forward_only_max_forward_speed_mps", 0.15)
        self.declare_parameter("command_stabilizer_enabled", True)
        self.declare_parameter("command_stabilizer_angular_deadband_radps", 0.05)
        self.declare_parameter("command_stabilizer_linear_deadband_mps", 0.015)
        self.declare_parameter("command_stabilizer_angular_flip_threshold_radps", 0.12)
        self.declare_parameter("command_stabilizer_linear_flip_threshold_mps", 0.04)
        self.declare_parameter("command_stabilizer_flip_confirm_cycles", 2)
        self.declare_parameter("command_stabilizer_low_speed_linear_mps", 0.03)
        self.declare_parameter("command_stabilizer_decisive_turn_radps", 0.10)
        self.declare_parameter("auto_gear_transition_enabled", True)
        self.declare_parameter("auto_gear_mismatch_deadband_mps", 0.025)
        self.declare_parameter("auto_gear_confirm_cycles", 5)
        self.declare_parameter("auto_gear_opposite_direction_commit_cycles", 12)
        self.declare_parameter("auto_gear_stopped_speed_mps", 0.015)
        self.declare_parameter("auto_gear_stop_timeout_sec", 2.0)
        self.declare_parameter("auto_gear_odom_timeout_sec", 0.5)
        self.declare_parameter("max_path_deviation_m", 0.15)
        self.declare_parameter("path_deviation_near_term_states", 2)
        self.declare_parameter("path_deviation_warn_m", 0.15)
        self.declare_parameter("path_deviation_stop_m", 0.20)
        self.declare_parameter("path_deviation_stop_cycles", 3)
        self.declare_parameter("obstacle_motion_enabled", False)
        self.declare_parameter("local_detour_enabled", False)
        self.declare_parameter("reverse_escape_enabled", False)
        self.declare_parameter("neupan_bypass_enabled", False)
        self.declare_parameter("obstacle_braking_mps2", 0.20)
        self.declare_parameter("obstacle_reaction_sec", 0.30)
        self.declare_parameter("obstacle_stop_margin_m", 0.20)
        self.declare_parameter("obstacle_preview_m", 0.90)
        self.declare_parameter("local_detour_preview_m", 1.50)
        self.declare_parameter("obstacle_odom_timeout_sec", 0.50)
        # Bounded worker recovery. A solve/IPC failure may restart the spawn
        # worker from the latest valid task instead of latching the node dead;
        # the budget is per goal/task and never unbounded.
        self.declare_parameter("worker_recovery_enabled", True)
        self.declare_parameter("worker_recovery_budget", 2)
        self.declare_parameter("worker_recovery_wait_sec", 5.0)
        # These stay disabled outside the dedicated HW0 shadow launch. The
        # force flag is runtime-settable only when that launch explicitly
        # allows it, and a triggered fault requires a process restart.
        self.declare_parameter("shadow_tf_fault_injection_allowed", False)
        self.declare_parameter("shadow_force_robot_tf_failure", False)

    def make_client(self, output_dir: str, config_path: str, checkpoint_path: str,
                    pan: dict[str, Any], receding: int | None,
                    torch_intraop: int, torch_interop: int, warmup_count: int,
                    timeout_sec: float, exact_capture_dir: str = "",
                    exact_capture_max_records: int = 512,
                    capture_metadata: dict[str, Any] | None = None) -> ProductionWholeCoreClient:
        return ProductionWholeCoreClient(
            output_dir, config_path, checkpoint_path, pan, receding,
            torch_intraop, torch_interop, warmup_count, timeout_sec,
            exact_capture_dir=exact_capture_dir,
            exact_capture_max_records=exact_capture_max_records,
            capture_metadata=capture_metadata,
            fault_mode=str(self.get_parameter("n116_fault_mode").value),
            fault_request_id=int(self.get_parameter("n116_fault_request_id").value),
            fault_sleep_sec=float(self.get_parameter("n116_fault_sleep_sec").value),
        )

    def __init__(self) -> None:
        super().__init__()
        # N1.15 creates a timer during base initialisation. In production the
        # scan callback and that timer compete in the same Python executor;
        # cancel it before spinning and solve from fresh input notifications.
        self.timer.cancel()
        self._shutdown_requested = False
        self._bridge_failed = False
        # Latest task inputs, cached so a failed worker can be rebuilt from the
        # current task instead of a replayed command. References only: each
        # cycle rebuilds these arrays, so nothing is mutated in place.
        self._last_inputs: dict[str, Any] | None = None
        self._scheduler_fault = ""
        self._worker_recovery_enabled = bool(
            self.get_parameter("worker_recovery_enabled").value
        )
        self._worker_recovery_budget = max(
            0, int(self.get_parameter("worker_recovery_budget").value)
        )
        self._worker_recovery_wait_sec = max(
            0.0, float(self.get_parameter("worker_recovery_wait_sec").value)
        )
        self._worker_recovery_used = 0
        self._worker_recovery_goal_version = None
        self._worker_recovery_last_code = ""
        # Optional context ledger: inert unless N116_CTX_LEDGER names a file.
        self._ctx_ledger_path = os.environ.get("N116_CTX_LEDGER", "").strip()
        self._ctx_ledger_restore()
        self._shadow_tf_fault_injection_allowed = bool(
            self.get_parameter("shadow_tf_fault_injection_allowed").value
        )
        self._shadow_force_robot_tf_failure = bool(
            self.get_parameter("shadow_force_robot_tf_failure").value
        )
        if self._shadow_force_robot_tf_failure and not self._shadow_tf_fault_injection_allowed:
            raise ValueError(
                "shadow_force_robot_tf_failure requires "
                "shadow_tf_fault_injection_allowed=true"
            )
        self._freeze_planner_result = False
        self._active_request = False
        self._active_lock = threading.Lock()
        self._last_wait_trace = ("", 0.0)
        self._scheduler_callback_started_monotonic = -1.0
        # Consecutive planner cycles whose predicted trajectory sits inside the
        # DUNE obstacle field. Downstream progress checking uses it to fail a
        # provably infeasible segment instead of dithering until the generic
        # movement timeout.
        self.infeasible_cycles = 0
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("bridge_status_topic").value), 50
        )
        self.semantic_pub = self.create_publisher(
            String, str(self.get_parameter("semantic_topic").value), 50
        )
        self.planner_result_pub = self.create_publisher(
            String, str(self.get_parameter("planner_result_topic").value), 50
        )
        self.planner_context_pub = self.create_publisher(
            String, str(self.get_parameter("planner_context_topic").value), 20
        )
        self.forward_only_enabled = bool(self.get_parameter("forward_only_enabled").value)
        self.forward_only = ForwardOnlyController(ForwardOnlyConfig(
            lookahead_m=float(self.get_parameter("forward_only_lookahead_m").value),
            max_forward_speed_mps=float(
                self.get_parameter("forward_only_max_forward_speed_mps").value),
        ))
        self.forward_only_status_pub = self.create_publisher(
            String, str(self.get_parameter("forward_only_status_topic").value), 50
        )
        self.command_stabilizer_enabled = bool(
            self.get_parameter("command_stabilizer_enabled").value
        )
        self.command_stabilizer = BidirectionalCommandStabilizer(CommandStabilizerConfig(
            angular_deadband_radps=float(
                self.get_parameter("command_stabilizer_angular_deadband_radps").value),
            linear_deadband_mps=float(
                self.get_parameter("command_stabilizer_linear_deadband_mps").value),
            angular_flip_threshold_radps=float(self.get_parameter(
                "command_stabilizer_angular_flip_threshold_radps").value),
            linear_flip_threshold_mps=float(self.get_parameter(
                "command_stabilizer_linear_flip_threshold_mps").value),
            flip_confirm_cycles=int(self.get_parameter(
                "command_stabilizer_flip_confirm_cycles").value),
            low_speed_linear_mps=float(self.get_parameter(
                "command_stabilizer_low_speed_linear_mps").value),
            decisive_turn_radps=float(self.get_parameter(
                "command_stabilizer_decisive_turn_radps").value),
        ))
        self.auto_gear_transition_enabled = (
            bool(self.get_parameter("auto_gear_transition_enabled").value) and
            self.allow_reverse
        )
        self.auto_gear_odom_timeout_sec = float(
            self.get_parameter("auto_gear_odom_timeout_sec").value
        )
        if self.auto_gear_odom_timeout_sec <= 0.0:
            raise ValueError("auto_gear_odom_timeout_sec must be positive")
        self.max_path_deviation_m = float(self.get_parameter("max_path_deviation_m").value)
        self.path_deviation_near_term_states = int(self.get_parameter(
            "path_deviation_near_term_states").value)
        self.path_deviation_guard = TemporalPathDeviationGuard(
            warn_m=float(self.get_parameter("path_deviation_warn_m").value),
            stop_m=float(self.get_parameter("path_deviation_stop_m").value),
            stop_cycles=int(self.get_parameter("path_deviation_stop_cycles").value),
        )
        self._path_deviation_goal_version: int | None = None
        self.gear_transition_gate = GearTransitionGate(GearTransitionConfig(
            mismatch_deadband_mps=float(self.get_parameter(
                "auto_gear_mismatch_deadband_mps").value),
            confirm_cycles=int(self.get_parameter("auto_gear_confirm_cycles").value),
            stopped_speed_mps=float(self.get_parameter(
                "auto_gear_stopped_speed_mps").value),
            stop_timeout_sec=float(self.get_parameter(
                "auto_gear_stop_timeout_sec").value),
            opposite_direction_commit_cycles=int(self.get_parameter(
                "auto_gear_opposite_direction_commit_cycles").value),
        ))
        self.reverse_recovery = StopReverseRecovery()
        self.obstacle_motion_enabled = bool(
            self.get_parameter("obstacle_motion_enabled").value
        )
        self.local_detour_enabled = bool(
            self.get_parameter("local_detour_enabled").value
        )
        self.reverse_escape_enabled = bool(
            self.get_parameter("reverse_escape_enabled").value
        )
        self.neupan_bypass_enabled = bool(
            self.get_parameter("neupan_bypass_enabled").value
        )
        self._neupan_bypass_engaged = False
        self._neupan_bypass_goal_version = None
        self._neupan_bypass_goal_pose = None
        self.obstacle_odom_timeout_sec = float(
            self.get_parameter("obstacle_odom_timeout_sec").value
        )
        if self.obstacle_odom_timeout_sec <= 0.0:
            raise ValueError("obstacle_odom_timeout_sec must be positive")
        with open(self.client.config_path, "r", encoding="utf-8") as stream:
            planner_config = yaml.safe_load(stream) or {}
        self.obstacle_motion_config = MotionPolicyConfig(
            max_speed_mps=float(planner_config["ref_speed"]),
            braking_mps2=float(self.get_parameter("obstacle_braking_mps2").value),
            reaction_sec=float(self.get_parameter("obstacle_reaction_sec").value),
            stop_margin_m=float(self.get_parameter("obstacle_stop_margin_m").value),
            preview_m=float(self.get_parameter("obstacle_preview_m").value),
            front_m=self.planner_base_extents["base_front_m"],
            rear_m=self.planner_base_extents["base_rear_m"],
            left_m=self.planner_base_extents["base_left_m"],
            right_m=self.planner_base_extents["base_right_m"],
            steps=int(self.client.receding or planner_config["receding"]),
            step_time_sec=float(planner_config["step_time"]),
        )
        self.local_detour_config = replace(
            self.obstacle_motion_config,
            preview_m=max(
                self.obstacle_motion_config.preview_m,
                float(self.get_parameter("local_detour_preview_m").value),
            ),
        )
        self.local_detour_planner = LocalDetourPlanner(self.local_detour_config)
        self.local_detour_controller = LocalDetourController()
        self.reverse_escape_planner = ReverseEscapePlanner(self.obstacle_motion_config)
        self._forward_blocked_reported_context: tuple[int, int] | None = None
        self.add_on_set_parameters_callback(self._shadow_fault_parameter_callback)
        self.get_logger().info(
            "N1.16 production bridge ready; forward-only=%s; planner exists only in one clean spawn worker"
            % self.forward_only_enabled
        )
        signal.signal(signal.SIGUSR1, self._freeze_planner_result_signal)
        self._planner_scheduler = LatestInputScheduler(
            self._run_from_latest_input, self._planner_scheduler_error
        )
        self._planner_scheduler.start()
        self.get_logger().info(
            "N1.16 planner scheduling is scan-driven with a one-slot latest-input queue"
        )

    def _planner_input_updated(self) -> None:
        """Wake one planner cycle; concurrent updates are deliberately coalesced."""
        scheduler = getattr(self, "_planner_scheduler", None)
        if scheduler is not None and not self._shutdown_requested:
            scheduler.notify()

    def _run_from_latest_input(self, started_monotonic: float) -> None:
        self._scheduler_callback_started_monotonic = float(started_monotonic)
        self.run()

    def _planner_scheduler_error(self, exc: BaseException) -> None:
        """Terminal-safe scheduler failure path.

        Order matters: the fail-safe stop is published *before* any logging or
        status publish, so an exception in either cannot skip the stop. The old
        code passed extra positional arguments to ``RcutilsLogger.error``, which
        raises ``TypeError`` and therefore never reached ``_publish_failsafe_stop``
        — the node died silently on the first exception.
        """
        self._scheduler_fault = "%s: %s" % (type(exc).__name__, exc)
        try:
            self._publish_failsafe_stop()
        except Exception:
            pass
        try:
            self.get_logger().error(
                "N1.16 latest-input scheduler failed: %s" % (self._scheduler_fault,)
            )
        except Exception:
            pass
        try:
            self._status("scheduler_exception", self._scheduler_fault)
        except Exception:
            pass
        if not self._recover_worker_from_cached_inputs("scheduler_exception"):
            self._bridge_failed = True

    def _recover_worker_from_cached_inputs(self, code: str) -> bool:
        """Rebuild the spawn worker from the latest valid task. Bounded.

        Returns True when a fresh worker handshake succeeded, so the caller must
        *not* latch ``_bridge_failed``. Returns False when recovery is disabled,
        unavailable, or the per-task budget is exhausted — the caller then
        latches as before. No command is reused: the caller has already
        published a zero Twist and no envelope is emitted for this cycle.
        """
        if not self._worker_recovery_enabled:
            self._worker_recovery_last_code = code + ":disabled"
            return False
        inputs = self._last_inputs
        if inputs is None:
            self._worker_recovery_last_code = code + ":no_latest_task"
            return False
        goal_version = inputs.get("goal_version")
        if goal_version != self._worker_recovery_goal_version:
            # A new task starts a new recovery budget.
            self._worker_recovery_goal_version = goal_version
            self._worker_recovery_used = 0
        if self._worker_recovery_used >= self._worker_recovery_budget:
            self._worker_recovery_last_code = code + ":budget_exhausted"
            return False
        self._worker_recovery_used += 1
        self._worker_recovery_last_code = code
        started = time.monotonic()
        try:
            self.client.recover_worker(
                inputs["path"],
                inputs["state"],
                inputs["points"],
                inputs["path_version"],
                navigation_loop_mode=inputs["navigation_loop_mode"],
                active_gear=inputs["active_gear"],
                wait_sec=self._worker_recovery_wait_sec,
            )
        except Exception as exc:
            self._worker_recovery_last_code = (
                code + ":recovery_failed:" + type(exc).__name__ + ":" + str(exc)
            )
            try:
                self.get_logger().error(
                    "N1.16 worker recovery failed: %s" % (self._worker_recovery_last_code,)
                )
            except Exception:
                pass
            try:
                self._publish_failsafe_stop()
            except Exception:
                pass
            return False
        elapsed = time.monotonic() - started
        try:
            self.get_logger().warn(
                "N1.16 worker recovered from %s in %.3fs (attempt %d/%d)"
                % (code, elapsed, self._worker_recovery_used, self._worker_recovery_budget)
            )
        except Exception:
            pass
        try:
            self._publish_failsafe_stop()
        except Exception:
            pass
        # Schedule a fresh solve from the latest scan; never replay a command.
        self._planner_input_updated()
        return True

    def _trace_wait_state(self, stage: str, **fields: Any) -> None:
        """Expose the first unsatisfied live input without flooding the log."""
        now = time.monotonic()
        previous_stage, previous_time = self._last_wait_trace
        if stage == previous_stage and now - previous_time < 2.0:
            return
        details = ", ".join(f"{key}={value}" for key, value in fields.items())
        self.get_logger().info(f"N1.16 waiting for {stage}: {details}")
        self._last_wait_trace = (stage, now)

    def _shadow_fault_parameter_callback(self, parameters) -> SetParametersResult:
        """Permit the one-shot TF fault only in the explicit shadow launch."""
        for parameter in parameters:
            if parameter.name == "shadow_tf_fault_injection_allowed":
                return SetParametersResult(
                    successful=False,
                    reason="shadow_tf_fault_injection_allowed is launch-time immutable",
                )
            if parameter.name != "shadow_force_robot_tf_failure":
                continue
            if not self._shadow_tf_fault_injection_allowed:
                return SetParametersResult(
                    successful=False,
                    reason="TF fault injection is disabled outside neupan_shadow",
                )
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False,
                    reason="shadow_force_robot_tf_failure must be boolean",
                )
            if self._shadow_force_robot_tf_failure and not parameter.value:
                return SetParametersResult(
                    successful=False,
                    reason="TF fault is latched; restart the shadow launch to reset",
                )
            self._shadow_force_robot_tf_failure = bool(parameter.value)
        return SetParametersResult(successful=True)

    def _freeze_planner_result_signal(self, _signum, _frame) -> None:
        """N2.3R1 LF1 hook: stop only source-envelope publication."""
        self._freeze_planner_result = True

    def _publish_context(self) -> None:
        with self.state_lock:
            context = make_context(self.nav2_path_version, self.goal_version)
        message = String()
        message.data = json.dumps(context, sort_keys=True, allow_nan=False)
        self.planner_context_pub.publish(message)

    # ------------------------------------------------------------------
    # Optional context ledger (additive; inert unless N116_CTX_LEDGER is set).
    #
    # nav2_path_version/goal_version are local counters that must match the
    # controller's path_version_/goal_version_ exactly. They start at (0, -1)
    # in N1.15, so every restart of this process silently moves the origin and
    # the controller then discards every envelope as
    # "discarded_envelope_context_mismatch" while this side never learns why.
    #
    # Persisting the counters makes a restart lossless: the controller only
    # advances them when it publishes a path, and it publishes no path while
    # no goal is active, so restarting this process with the goal idle leaves
    # the controller's counters untouched.
    #
    # Both helpers swallow every error: a broken ledger must never affect
    # planning or navigation, only emit a warning.
    # ------------------------------------------------------------------
    def _ctx_ledger_restore(self) -> None:
        path = self._ctx_ledger_path
        if not path:
            return
        current = (self.nav2_path_version, self.goal_version)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            path_version = int(payload["path_version"])
            goal_version = int(payload["goal_version"])
        except FileNotFoundError:
            self.get_logger().warn(
                f"N1.16 context ledger absent at {path}; "
                f"counters stay at {current}"
            )
            return
        except Exception as exc:
            self.get_logger().warn(
                f"N1.16 context ledger unreadable at {path} ({exc}); "
                f"counters stay at {current}"
            )
            return
        self.nav2_path_version = path_version
        self.goal_version = goal_version
        self.get_logger().warn(
            f"N1.16 context ledger restored {current} -> "
            f"({path_version}, {goal_version}) from {path}"
        )

    def _ctx_ledger_persist(self) -> None:
        path = self._ctx_ledger_path
        if not path:
            return
        try:
            payload = {
                "path_version": int(self.nav2_path_version),
                "goal_version": int(self.goal_version),
                "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "pid": os.getpid(),
            }
            tmp = f"{path}.tmp{os.getpid()}"
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, sort_keys=True)
            os.replace(tmp, path)
        except Exception as exc:
            self.get_logger().warn(f"N1.16 context ledger persist failed: {exc}")

    def path_callback(self, message) -> None:
        super().path_callback(message)
        self._publish_context()
        self._ctx_ledger_persist()

    def goal_callback(self, message) -> None:
        super().goal_callback(message)
        self._publish_context()
        self._ctx_ledger_persist()

    def _status(self, code: str, message: str, **fields: Any) -> None:
        payload = {"schema": N116_SCHEMA, "status": "fail-safe", "code": code,
                   "message": str(message), "monotonic": time.monotonic(), **fields}
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        self.get_logger().error(f"N1.16 bridge {code}: {message}")

    def _publish_failsafe_stop(self) -> None:
        # A zero command is the only command emitted after a bridge failure;
        # no planner output or stale plan is published.
        self.cmd_pub.publish(Twist())

    def _publish_forward_only_status(self, decision, path_version: int,
                                     goal_version: int, cycle_id: int) -> None:
        message = String()
        message.data = json.dumps({
            "schema": "r1_neupan_forward_only_status_v1",
            "state": decision.state,
            "reason": decision.reason,
            "blocked": bool(decision.blocked),
            "path_version": int(path_version),
            "goal_version": int(goal_version),
            "cycle_id": int(cycle_id),
            "raw_command": {
                "linear_x": float(decision.raw_linear_x),
                "angular_z": float(decision.raw_angular_z),
            },
            "actuator_command": {
                "linear_x": float(decision.linear_x),
                "angular_z": float(decision.angular_z),
            },
            "path_yaw_error_rad": float(decision.yaw_error_rad),
            "observed_front_clearance_m": decision.observed_front_clearance_m,
            "required_front_clearance_m": float(decision.required_front_clearance_m),
            "monotonic": time.monotonic(),
        }, allow_nan=False, sort_keys=True)
        self.forward_only_status_pub.publish(message)

    def _fresh_for_publish(self, record: dict[str, Any], expected_scan: int,
                           expected_path: int, expected_goal: int) -> tuple[bool, str]:
        with self.state_lock:
            current_path = int(self.path_version)
            current_goal = int(self.goal_version)
        checks = (
            (int(record.get("scan_counter", -1)) >= int(expected_scan), "scan_counter_lt_expected"),
            (current_path == int(expected_path), "path_version_changed"),
            (current_goal == int(expected_goal), "goal_version_changed"),
        )
        for passed, reason in checks:
            if not passed:
                return False, reason
        return True, "fresh"

    def run(self) -> None:
        if self._shutdown_requested or self._bridge_failed:
            return
        transition = self._consume_gear_transition()
        if transition is not None:
            self.cmd_pub.publish(Twist())
            message = String()
            message.data = json.dumps({
                "schema": "r1_n2_pre_semantics_v1",
                "semantic_event": "gear_transition",
                "navigation_mode": self.navigation_mode,
                "loop": self.navigation_loop,
                "active_gear": int(self.active_gear),
                "requested_gear": int(self.requested_gear),
                "gear_transition": transition,
                "action": [[0.0], [0.0]],
                "info": {"arrive": False, "stop": False},
            }, allow_nan=False, sort_keys=True)
            self.semantic_pub.publish(message)
            # The active gear changed after this enforced zero. Schedule a
            # fresh solve from the same latest scan, not a replayed command.
            self._planner_input_updated()
            return
        with self._active_lock:
            if self._active_request:
                return
            self._active_request = True
        try:
            cycle_start = time.monotonic()
            if self._shadow_force_robot_tf_failure:
                self._bridge_failed = True
                self._status(
                    "shadow_tf_fault_injected",
                    "shadow-only robot TF lookup failure injected; restart required",
                )
                self._publish_failsafe_stop()
                return
            if not self._robot_transform():
                self._trace_wait_state("map_to_base_tf")
                return
            with self.state_lock:
                if self.path_matrix is None:
                    self._trace_wait_state("global_path", path_version=self.path_version)
                    return
                if self.robot_state is None:
                    self._trace_wait_state("robot_state")
                    return
                if self.dynamic_footprint_enabled and not self.dynamic_footprint_ready:
                    self._trace_wait_state("dynamic_footprint")
                    self._publish_failsafe_stop()
                    return
                scan_counter = int(self.scan_counter)
                if self.latest_scan_only and scan_counter <= self.last_processed_scan:
                    self._trace_wait_state(
                        "new_scan",
                        scan_counter=scan_counter,
                        last_processed_scan=self.last_processed_scan,
                    )
                    return
                if self.obstacle_motion_enabled and self.processed_scan_counter != scan_counter:
                    self._trace_wait_state("processed_scan", scan_counter=scan_counter)
                    return
                if self.obstacle_motion_enabled and self.raw_environment_points is None:
                    self._trace_wait_state("scan_transform", scan_counter=scan_counter)
                    self._publish_failsafe_stop()
                    return
                state = self.robot_state.copy()
                points = (np.empty((2, 0), dtype=np.float64)
                          if self.obstacle_points is None else self.obstacle_points.copy())
                raw_points = (
                    np.empty((2, 0), dtype=np.float64)
                    if self.raw_environment_points is None
                    else self.raw_environment_points.copy()
                )
                path = self.path_matrix.copy()
                path_version = int(self.path_version)
                nav2_path_version = int(self.nav2_path_version)
                goal = None if self.goal is None else self.goal.copy()
                goal_version = int(self.goal_version)
                scan_stamp = float(self.latest_scan_stamp_sec)
                scan_receive = float(self.latest_scan_receive_monotonic)
                scan_metrics = dict(self.latest_scan_metrics)
                scan_filter = dict(self.latest_scan_filter)
                measured_velocity = np.asarray([
                    [self.observed_linear_x], [self.observed_angular_z],
                ], dtype=np.float64)
                odom_age_sec = time.monotonic() - self.observed_odom_received_monotonic
                tf_stamp = float(getattr(self, "tf_stamp_sec", -1.0))
                active_gear = int(self.active_gear)
                requested_gear = int(self.requested_gear)
                gear_transition_pending = self.pending_gear is not None
                previous_scan = int(self.last_processed_scan)
                self.last_processed_scan = scan_counter
            # Cache the exact task a rebuilt worker must resume. These arrays
            # are rebuilt every cycle and never mutated in place, so keeping
            # references is safe and costs nothing per cycle.
            self._last_inputs = {
                "path": path,
                "state": state,
                "points": points,
                "path_version": path_version,
                "goal_version": goal_version,
                "navigation_loop_mode": self.navigation_mode,
                "active_gear": active_gear,
                "at_monotonic": time.monotonic(),
            }
            motion_decision = None
            route_decision = None
            local_detour = None
            bypass_requested = False
            if self._neupan_bypass_goal_version != goal_version:
                if not same_terminal_pose(self._neupan_bypass_goal_pose, goal):
                    self._neupan_bypass_engaged = False
                self._neupan_bypass_goal_version = goal_version
                self._neupan_bypass_goal_pose = None if goal is None else goal.copy()
            if self.obstacle_motion_enabled:
                if odom_age_sec > self.obstacle_odom_timeout_sec:
                    self._trace_wait_state("fresh_odom", age_sec=odom_age_sec)
                    self._publish_failsafe_stop()
                    return
                route_decision = plan_obstacle_speed(
                    state, path, raw_points, points, self.obstacle_motion_config
                )
                bypass_requested = (
                    self.neupan_bypass_enabled
                    and ((active_gear == 1 and route_decision.route_blocked)
                         or self._neupan_bypass_engaged)
                )
                if self.local_detour_enabled and active_gear == 1:
                    detour_route_decision = plan_obstacle_speed(
                        state, path, raw_points, points, self.local_detour_config
                    )
                    local_detour = self.local_detour_planner.update(
                        state, path, raw_points, points, detour_route_decision,
                        path_version, time.monotonic(),
                    )
                else:
                    self.local_detour_planner.reset()
                    self.local_detour_controller.reset()
                motion_decision = (
                    plan_obstacle_speed(
                        state, local_detour.path, raw_points, points,
                        self.obstacle_motion_config,
                    ) if local_detour is not None else route_decision
                )
            obsolete = 0 if previous_scan == 0 else max(0, scan_counter - previous_scan - 1)
            self.obsolete_dropped += obsolete
            self.unique_processed += 1
            self.cycle_id += 1
            cycle_id = int(self.cycle_id)
            self._trace_wait_state(
                "worker_result",
                cycle_id=cycle_id,
                scan_counter=scan_counter,
                path_version=path_version,
                goal_version=goal_version,
            )
            try:
                self.client.startup(
                    path, state, points, path_version,
                    navigation_loop_mode=self.navigation_mode,
                    active_gear=active_gear,
                )
                cycle_start = time.monotonic()
                input_finished = time.perf_counter()
                outputs, record = self.client.solve(
                    state, points, path, path_version, scan_counter, scan_stamp,
                    goal=goal, goal_version=goal_version, cycle_id=cycle_id,
                    navigation_loop_mode=self.navigation_mode,
                    active_gear=active_gear,
                    scan_callback_receive_monotonic=float(
                        getattr(self, "latest_scan_receive_monotonic", -1.0)
                    ),
                    scan_update_monotonic=float(
                        getattr(self, "latest_scan_update_monotonic", -1.0)
                    ),
                    reference_speed_profile=(
                        motion_decision.speed_profile if motion_decision is not None else None
                    ),
                    measured_velocity=(
                        measured_velocity if motion_decision is not None else None
                    ),
                    local_reference_path=(
                        local_detour.path if local_detour is not None else None
                    ),
                    local_reference_id=(
                        local_detour.identifier if local_detour is not None else 0
                    ),
                    bypass_mode=bypass_requested,
                )
            except BridgeProtocolError as exc:
                # Retract this cycle's motion output before anything else. A
                # recoverable transport/solve failure rebuilds the worker from
                # the cached task instead of latching the node dead; only an
                # unavailable or exhausted recovery latches as before.
                self._publish_failsafe_stop()
                self._status(exc.code, str(exc), cycle_id=cycle_id, scan_counter=scan_counter)
                if not self._recover_worker_from_cached_inputs(str(exc.code)):
                    self._status(
                        "worker_recovery_exhausted",
                        self._worker_recovery_last_code,
                        cycle_id=cycle_id,
                        scan_counter=scan_counter,
                    )
                    self._bridge_failed = True
                return
            except Exception as exc:
                self._bridge_failed = True
                self.client._event("bridge_exception", str(exc), cycle_id=cycle_id,
                                   scan_counter=scan_counter)
                self._status("bridge_exception", str(exc), cycle_id=cycle_id,
                             scan_counter=scan_counter)
                self._publish_failsafe_stop()
                return

            fresh, reason = self._fresh_for_publish(record, scan_counter, path_version, goal_version)
            main_validation_complete_monotonic = time.monotonic()
            if not fresh:
                self.client.mark_stale_response(reason, record)
                self._status("stale_response", reason, request_id=record["request_id"],
                             cycle_id=cycle_id, scan_counter=scan_counter)
                self._publish_failsafe_stop()
                return

            # Classify the trajectory before any geometry runs. On a normal
            # arrive the core packs an empty opt_state; the >=3-state collision
            # helpers raise on that, and the exception used to escape run() and
            # kill the node through the scheduler error path.
            opt_state = outputs["opt_state"]
            plan_usable = bool(
                isinstance(opt_state, np.ndarray)
                and opt_state.ndim == 2
                and opt_state.shape[0] >= 3
                and opt_state.shape[1] >= 1
                and bool(np.isfinite(opt_state).all())
            )
            arrived = bool(record.get("info", {}).get("arrive"))
            if not plan_usable and not arrived:
                # Empty/invalid plan without an arrival is a solve-side fault,
                # not a stop: hold the fail-safe output and stay alive so the
                # next valid scan can produce a plan.
                self._status(
                    "invalid_trajectory",
                    "core returned an unusable opt_state: ndim=%s shape=%s"
                    % (getattr(opt_state, "ndim", None),
                       getattr(opt_state, "shape", None)),
                    request_id=record["request_id"],
                    cycle_id=cycle_id,
                    scan_counter=scan_counter,
                )
                self._publish_failsafe_stop()
                return

            timing = dict(record["timings"])
            timing["main_input_extraction_ms"] = (input_finished - cycle_start) * 1000.0
            timing["scheduler_wakeup_to_cycle_start_ms"] = max(
                0.0,
                (cycle_start - self._scheduler_callback_started_monotonic) * 1000.0,
            )
            # The worker PID is carried by the response envelope. Copy it
            # into the published timing record for resource attribution.
            timing["worker_pid"] = float(record.get("worker_pid", -1))
            timing["dropped_obsolete_scans"] = float(obsolete)
            timing["duplicate_scan"] = 0.0
            timing["queue_depth"] = 1.0
            timing["worker_generation"] = float(record["worker_generation"])
            timing["worker_failure"] = 0.0
            timing["finite"] = 1.0
            timing["nan_inf"] = 0.0
            timing["stale_response"] = 0.0
            timing["obstacle_points_count"] = float(points.shape[1])
            timing.update(scan_metrics)
            publish_started = time.perf_counter()
            stamp = self.get_clock().now().to_msg()
            self.plan_pub.publish(self._path_message(outputs["opt_state"], self.map_frame, stamp))
            self.ref_pub.publish(self._path_message(outputs["ref_state"], self.map_frame, stamp))
            action = outputs["action"]
            command = Twist()
            optimized_free_distance = float("inf")
            if motion_decision is not None and plan_usable:
                optimized_free_distance = min(
                    first_swept_collision_distance(
                        state, opt_state, raw_points,
                        self.obstacle_motion_config,
                    ),
                    first_swept_collision_distance(
                        state, opt_state, points,
                        self.obstacle_motion_config,
                    ),
                )
            verified_neupan_bypass = (
                plan_usable
                and bypass_requested
                and bool(record.get("info", {}).get("bypass_valid", False))
                and np.isinf(optimized_free_distance)
            )
            bypass_phase = record.get("info", {}).get("bypass_phase")
            if bypass_phase == "done":
                self._neupan_bypass_engaged = False
            elif verified_neupan_bypass:
                self._neupan_bypass_engaged = True
            max_path_deviation_m = calculate_max_path_deviation_m(outputs["opt_state"], path)
            near_term_deviation_m = calculate_near_term_path_deviation_m(
                outputs["opt_state"], path, self.path_deviation_near_term_states)
            if self._path_deviation_goal_version != goal_version:
                self.path_deviation_guard.reset()
                self._path_deviation_goal_version = goal_version
            path_deviation = self.path_deviation_guard.evaluate(near_term_deviation_m)
            path_deviation_blocked = path_deviation.blocked
            if verified_neupan_bypass:
                path_deviation_blocked = False
                self.path_deviation_guard.reset()
            verified_local_detour = False
            if local_detour is not None and not motion_decision.route_blocked:
                verified_local_detour = True
                path_deviation_blocked = False
                self.path_deviation_guard.reset()
            elif (self.local_detour_enabled and plan_usable
                  and motion_decision is not None
                  and motion_decision.route_blocked):
                # plan_usable already proved ndim == 2, shape[0] >= 3 and
                # shape[1] >= 1, so predicted[:2, -1] below cannot index past
                # the end of a normal-arrive empty trajectory.
                predicted = outputs["opt_state"]
                route_vector = path[:2, -1] - state[:2, 0]
                route_norm = float(np.linalg.norm(route_vector))
                predicted_progress = (
                    float(np.dot(predicted[:2, -1] - predicted[:2, 0], route_vector)
                          / route_norm)
                    if route_norm > 1.0e-6 else 0.0
                )
                verified_local_detour = (
                    optimized_free_distance >= motion_decision.free_distance_m
                    and optimized_free_distance > self.obstacle_motion_config.stop_margin_m
                    and near_term_deviation_m > self.path_deviation_guard.warn_m
                    and near_term_deviation_m <= 0.35
                    and predicted_progress > 0.02
                )
                if verified_local_detour:
                    path_deviation_blocked = False
                    self.path_deviation_guard.reset()
            raw_linear_x = 0.0
            raw_angular_z = 0.0
            decision = None
            stabilization = None
            gear_gate_decision = None
            stop_requested = bool(record.get("info", {}).get("stop"))
            arrived = bool(record.get("info", {}).get("arrive"))
            recovery_reverse_requested = False
            escape_linear = None
            escape_state = "DISABLED"
            if (self.obstacle_motion_enabled and self.reverse_escape_enabled
                    and route_decision is not None):
                escape_linear, escape_state = self.reverse_escape_planner.update(
                    state, float(measured_velocity[0, 0]), raw_points, points,
                    route_decision, local_detour is not None, active_gear,
                    nav2_path_version, time.monotonic(),
                )
            detour_phase = None
            guided_detour_command = False
            recovery_velocity = (
                stopped_reverse_velocity(outputs.get("optimized_velocity", np.empty((0,))))
                if stop_requested and self.allow_reverse and not self.forward_only_enabled
                and not self.obstacle_motion_enabled
                else None
            )
            if recovery_velocity is not None and not arrived and goal is not None:
                recovery_reverse_requested = self.reverse_recovery.permit(
                    goal_pose=(float(goal[0, 0]), float(goal[1, 0]), float(goal[2, 0])),
                    pose=(float(state[0, 0]), float(state[1, 0])),
                    active_gear=active_gear,
                    now=time.monotonic(),
                )
            if escape_linear is not None and escape_linear < 0.0:
                recovery_reverse_requested = True
            if not arrived and (recovery_reverse_requested or
                                escape_linear is not None or
                                (local_detour is not None and verified_local_detour) or
                                (not stop_requested and not path_deviation_blocked)):
                if escape_linear is not None:
                    candidate = np.asarray([[escape_linear], [0.0]])
                elif recovery_reverse_requested:
                    candidate = np.asarray(recovery_velocity).reshape(2, 1)
                elif (route_decision is not None and route_decision.route_blocked
                      and local_detour is None
                      and route_decision.speed_profile[0] <= 0.0
                      and not verified_neupan_bypass):
                    candidate = np.zeros((2, 1))
                else:
                    candidate = action
                if local_detour is not None and verified_local_detour and not recovery_reverse_requested:
                    guided_linear, guided_angular, detour_phase = self.local_detour_controller.command(
                        state, local_detour
                    )
                    candidate = np.asarray([[guided_linear], [guided_angular]])
                    guided_detour_command = True
                if candidate.size >= 2:
                    raw_linear_x = float(candidate.reshape(-1)[0])
                    raw_angular_z = float(candidate.reshape(-1)[1])
                    if verified_neupan_bypass and raw_linear_x > 0.0:
                        optimized_speed = plan_obstacle_speed(
                            state, outputs["opt_state"], raw_points, points,
                            self.obstacle_motion_config,
                        ).speed_profile[0]
                        raw_linear_x = min(raw_linear_x, float(optimized_speed))
                    if self.forward_only_enabled:
                        decision = self.forward_only.decide(
                            raw_linear_x=raw_linear_x,
                            raw_angular_z=raw_angular_z,
                            stop_requested=stop_requested,
                            arrived=arrived,
                            state=state,
                            path=path,
                            path_version=path_version,
                            goal_version=goal_version,
                        )
                        self._publish_forward_only_status(
                            decision, path_version, goal_version, cycle_id
                        )
                        command.linear.x = decision.linear_x
                        command.angular.z = decision.angular_z
                    else:
                        # Reverse is allowed only in the explicit HW1 profile.
                        # The collision guard validates the signed stopping
                        # trajectory before forwarding it to the base.
                        if self.auto_gear_transition_enabled:
                            with self.state_lock:
                                observed_linear_x = float(self.observed_linear_x)
                                observed_odom_receive = float(
                                    self.observed_odom_received_monotonic)
                            now = time.monotonic()
                            odom_fresh = (
                                observed_odom_receive >= 0.0 and
                                now - observed_odom_receive <=
                                self.auto_gear_odom_timeout_sec
                            )
                            gear_gate_decision = self.gear_transition_gate.decide(
                                linear_x=raw_linear_x,
                                angular_z=raw_angular_z,
                                active_gear=active_gear,
                                observed_linear_x=observed_linear_x,
                                odom_fresh=odom_fresh,
                                now=now,
                            )
                            if gear_gate_decision.request_gear is not None:
                                with self.state_lock:
                                    if (self.active_gear == active_gear and
                                            self.pending_gear is None):
                                        self.pending_gear = (
                                            gear_gate_decision.request_gear)
                                        self.requested_gear = (
                                            gear_gate_decision.request_gear)
                                    else:
                                        self.gear_transition_gate.reset()
                            if gear_gate_decision.state != "DRIVE":
                                # Never feed an opposite-direction action to
                                # the closed-loop velocity smoother.
                                self.command_stabilizer.reset()
                            elif self.command_stabilizer_enabled:
                                stabilization = self.command_stabilizer.stabilize(
                                    gear_gate_decision.linear_x,
                                    gear_gate_decision.angular_z
                                )
                                command.linear.x = stabilization.linear_x
                                command.angular.z = stabilization.angular_z
                            else:
                                command.linear.x = gear_gate_decision.linear_x
                                command.angular.z = gear_gate_decision.angular_z
                        elif self.command_stabilizer_enabled:
                            stabilization = self.command_stabilizer.stabilize(
                                raw_linear_x, raw_angular_z
                            )
                            command.linear.x = stabilization.linear_x
                            command.angular.z = stabilization.angular_z
                        else:
                            command.linear.x = raw_linear_x
                            command.angular.z = raw_angular_z
            if arrived or (stop_requested and not guided_detour_command and
                           not recovery_reverse_requested and escape_linear is None):
                self.command_stabilizer.reset()
                self.gear_transition_gate.reset()
                self.path_deviation_guard.reset()
            unsafe_prediction = False
            command_free_distance = float("inf")
            stopping_distance = 0.0
            if motion_decision is not None and (
                abs(command.linear.x) > 1.0e-6 or abs(command.angular.z) > 1.0e-6
            ):
                speed = max(abs(command.linear.x), abs(float(measured_velocity[0, 0])))
                stopping_distance = (
                    speed * self.obstacle_motion_config.reaction_sec
                    + speed * speed / (2.0 * self.obstacle_motion_config.braking_mps2)
                    + self.obstacle_motion_config.stop_margin_m
                )
                command_preview = replace(
                    self.obstacle_motion_config,
                    preview_m=min(
                        self.obstacle_motion_config.preview_m,
                        max(0.30, stopping_distance + 0.10),
                    ),
                    # The MPC horizon grew from 6 to 20 steps for bypass.
                    # Keep the original 1.5 s constant-command rotation sweep.
                    steps=min(6, self.obstacle_motion_config.steps),
                )
                command_free_distance = min(
                    first_command_collision_distance(
                        state, command.linear.x, command.angular.z,
                        raw_points, command_preview,
                    ),
                    first_command_collision_distance(
                        state, command.linear.x, command.angular.z,
                        points, command_preview,
                    ),
                )
                if not guided_detour_command:
                    command_free_distance = min(
                        command_free_distance, optimized_free_distance
                    )
                unsafe_prediction = command_free_distance < stopping_distance
                if unsafe_prediction:
                    command = Twist()
            recovery_reverse_active = recovery_reverse_requested and command.linear.x < 0.0
            obstacle_state = "CLEAR"
            if local_detour is not None and verified_local_detour and not unsafe_prediction:
                obstacle_state = "DETOUR" if abs(command.linear.x) > 0.005 or abs(command.angular.z) > 0.05 else "WAIT"
            elif motion_decision is not None and motion_decision.route_blocked:
                if recovery_reverse_active:
                    obstacle_state = "RECOVER"
                elif verified_neupan_bypass and not unsafe_prediction:
                    obstacle_state = "DETOUR"
                elif verified_local_detour and not unsafe_prediction and command.linear.x > 0.0:
                    obstacle_state = "DETOUR"
                elif abs(command.linear.x) < 0.005 and abs(measured_velocity[0, 0]) < 0.015:
                    obstacle_state = "WAIT"
                else:
                    obstacle_state = "SLOW"
            cmd_publish_monotonic = time.monotonic()
            self.cmd_pub.publish(command)
            output_publish = time.monotonic()
            planner_result_monotonic = time.monotonic()
            planner_result = make_planner_envelope(
                linear_x=command.linear.x,
                angular_z=command.angular.z,
                worker_generation=int(record["worker_generation"]),
                request_id=int(record["request_id"]),
                cycle_id=cycle_id,
                scan_counter=scan_counter,
                planner_result_monotonic=planner_result_monotonic,
                path_version=nav2_path_version,
                goal_version=goal_version,
            )
            planner_result_message = String()
            planner_result_message.data = json.dumps(
                planner_result, sort_keys=True, allow_nan=False
            )
            # N2.3R1 LF1 test hook: freeze only the planner-result source
            # stream.  The bridge, timing heartbeat, and relay input remain
            # alive so the safety gate must distinguish source staleness from
            # relay loss.  The default path is unchanged.
            if not self._freeze_planner_result:
                self.planner_result_pub.publish(planner_result_message)
                planner_result_publish_monotonic = time.monotonic()
            else:
                planner_result_publish_monotonic = -1.0
            record["planner_result"] = planner_result
            timing["main_validation_complete_monotonic"] = main_validation_complete_monotonic
            timing["planner_result_publish_monotonic"] = planner_result_publish_monotonic
            timing["main_publish_ms"] = (time.perf_counter() - publish_started) * 1000.0
            timing["main_full_cycle_ms"] = (output_publish - cycle_start) * 1000.0
            timing_record = {
                "schema": N116_SCHEMA, "cycle_id": float(cycle_id),
                "scan_counter": float(scan_counter), "scan_stamp_sec": scan_stamp,
                "scan_receive_monotonic": scan_receive,
                "path_version": float(nav2_path_version),
                "goal_version": float(goal_version), "tf_stamp_sec": tf_stamp,
                "cycle_start_monotonic": float(cycle_start),
                "worker_core_start_monotonic": float(timing.get("worker_core_start_monotonic", cycle_start)),
                "worker_core_end_monotonic": float(timing.get("worker_core_end_monotonic", output_publish)),
                "output_publish_monotonic": float(output_publish),
                "planner_result_monotonic": float(planner_result_monotonic),
                "main_validation_complete_monotonic": float(main_validation_complete_monotonic),
                "planner_result_publish_monotonic": float(planner_result_publish_monotonic),
            }
            for field in N116_TIMING_FIELDS:
                if field not in timing_record:
                    timing_record[field] = float(
                        timing.get(field, -1.0 if field.endswith("_monotonic") else 0.0)
                    )
            timing_message = Float64MultiArray()
            timing_message.data = [float(timing_record[field]) for field in N116_TIMING_FIELDS]
            self.timing_pub.publish(timing_message)
            record["timing_record"] = timing_record
            semantic = dict(record.get("semantic", {}))
            dune_min_value = semantic.get("dune_min_distance_m")
            if stop_requested and isinstance(dune_min_value, (int, float)) and dune_min_value < 0.0:
                self.infeasible_cycles += 1
            else:
                self.infeasible_cycles = 0
            if isinstance(semantic.get("initial_path"), np.ndarray):
                semantic["initial_path"] = semantic["initial_path"].tolist()
            semantic.update({
                "schema": "r1_n2_pre_semantics_v1",
                "cycle_id": int(cycle_id),
                "scan_counter": int(scan_counter),
                "path_version": int(nav2_path_version),
                "planner_path_version": int(path_version),
                "goal_version": int(goal_version),
                "state": state.tolist(),
                "input_path": path.tolist(),
                "goal": None if goal is None else goal.tolist(),
                "action": action.tolist(),
                "ref_state": outputs["ref_state"].tolist(),
                "ref_speed": outputs.get("ref_speed", np.empty((0,))).tolist(),
                "opt_state": outputs.get("opt_state", np.empty((0,))).tolist(),
                "optimized_velocity": outputs.get("optimized_velocity", np.empty((0,))).tolist(),
                "distance": outputs.get("distance", np.empty((0,))).tolist(),
                "min_planner_distance": (
                    float(np.min(outputs["distance"]))
                    if outputs.get("distance") is not None and outputs["distance"].size else None
                ),
                "scan_filter": scan_filter,
                "infeasible_cycles": int(self.infeasible_cycles),
                "recovery_reverse_requested": recovery_reverse_requested,
                "recovery_reverse_active": recovery_reverse_active,
                "obstacle_motion": {
                    "enabled": self.obstacle_motion_enabled,
                    "local_detour_enabled": self.local_detour_enabled,
                    "reverse_escape_enabled": self.reverse_escape_enabled,
                    "neupan_bypass_enabled": self.neupan_bypass_enabled,
                    "neupan_bypass_requested": bypass_requested,
                    "neupan_bypass_verified": verified_neupan_bypass,
                    "neupan_bypass_phase": bypass_phase,
                    "neupan_bypass_side": int(record.get("info", {}).get("bypass_side", 0)),
                    "neupan_bypass_clearance_m": record.get("info", {}).get("bypass_clearance_m"),
                    "neupan_bypass_solver_failures": int(record.get("info", {}).get("bypass_solver_failures", 0)),
                    "state": obstacle_state,
                    "reason": (
                        motion_decision.reason if motion_decision is not None else "disabled"
                    ),
                    "route_blocked": bool(
                        motion_decision.route_blocked if motion_decision is not None else False
                    ),
                    "original_route_blocked": bool(
                        route_decision.route_blocked if route_decision is not None else False
                    ),
                    "local_detour_id": (
                        local_detour.identifier if local_detour is not None else None
                    ),
                    "local_detour_side": (
                        local_detour.side if local_detour is not None else None
                    ),
                    "local_detour_phase": detour_phase,
                    "local_detour_search": self.local_detour_planner.last_status,
                    "guided_detour_command": guided_detour_command,
                    "worker_stop_requested": stop_requested,
                    "reverse_escape_state": escape_state,
                    "free_distance_m": (
                        motion_decision.free_distance_m
                        if motion_decision is not None and np.isfinite(motion_decision.free_distance_m)
                        else None
                    ),
                    "optimized_free_distance_m": (
                        optimized_free_distance if np.isfinite(optimized_free_distance)
                        else None
                    ),
                    "command_free_distance_m": (
                        command_free_distance if np.isfinite(command_free_distance)
                        else None
                    ),
                    "command_stopping_distance_m": stopping_distance,
                    "reference_speed_profile": (
                        motion_decision.speed_profile.tolist()
                        if motion_decision is not None else None
                    ),
                    "verified_local_detour": verified_local_detour,
                    "unsafe_prediction": unsafe_prediction,
                    "odom_age_sec": float(odom_age_sec),
                },
                "scan_timing_ms": {
                    key: float(value) for key, value in scan_metrics.items()
                },
                "dune": {
                    "min_distance_m": semantic.get("dune_min_distance_m"),
                    "obstacle_points_count": semantic.get("dune_obstacle_points_count"),
                },
                "forward_only": {
                    "enabled": self.forward_only_enabled,
                    "state": decision.state if decision is not None else "IDLE",
                    "reason": (
                        decision.reason if decision is not None else
                        ("legacy_clamp" if not self.forward_only_enabled else "no_action")
                    ),
                    "raw_linear_x": float(raw_linear_x),
                    "raw_angular_z": float(raw_angular_z),
                    "actuator_linear_x": float(command.linear.x),
                    "actuator_angular_z": float(command.angular.z),
                    "path_yaw_error_rad": (
                        float(decision.yaw_error_rad) if decision is not None else None
                    ),
                    "observed_front_clearance_m": (
                        decision.observed_front_clearance_m if decision is not None else None
                    ),
                    "required_front_clearance_m": (
                        float(decision.required_front_clearance_m)
                        if decision is not None else None
                    ),
                },
                "command_stabilizer": {
                    "enabled": self.command_stabilizer_enabled,
                    "state": stabilization.state if stabilization is not None else "BYPASSED",
                    "reason": stabilization.reason if stabilization is not None else (
                        "forward_only" if decision is not None else "no_action"
                    ),
                    "raw_linear_x": float(raw_linear_x),
                    "raw_angular_z": float(raw_angular_z),
                    "linear_x": float(command.linear.x),
                    "angular_z": float(command.angular.z),
                },
                "path_deviation_guard": {
                    "blocked": bool(path_deviation_blocked),
                    "max_horizon_deviation_m": float(max_path_deviation_m),
                    "near_term_deviation_m": float(near_term_deviation_m),
                    "warning": bool(path_deviation.warning),
                    "deviation_violation_cycles": int(path_deviation.violation_cycles),
                    "reason": path_deviation.reason,
                    "warn_m": float(self.path_deviation_guard.warn_m),
                    "stop_m": float(self.path_deviation_guard.stop_m),
                    "stop_cycles": int(self.path_deviation_guard.stop_cycles),
                    "near_term_states": int(self.path_deviation_near_term_states),
                    "legacy_max_horizon_limit_m": float(self.max_path_deviation_m),
                },
                "gear_transition_gate": {
                    "enabled": self.auto_gear_transition_enabled,
                    "state": (
                        gear_gate_decision.state if gear_gate_decision is not None
                        else "BYPASSED"
                    ),
                    "reason": (
                        gear_gate_decision.reason if gear_gate_decision is not None
                        else "forward_only" if decision is not None else "no_action"
                    ),
                    "request_gear": (
                        gear_gate_decision.request_gear
                        if gear_gate_decision is not None else None
                    ),
                    "observed_linear_x": float(self.observed_linear_x),
                    "odom_fresh": bool(
                        self.observed_odom_received_monotonic >= 0.0 and
                        time.monotonic() - self.observed_odom_received_monotonic <=
                        self.auto_gear_odom_timeout_sec
                    ),
                    "raw_linear_x": float(raw_linear_x),
                    "raw_angular_z": float(raw_angular_z),
                    "linear_x": float(command.linear.x),
                    "angular_z": float(command.angular.z),
                },
                "obstacle_points_count": int(points.shape[1]),
                "pan": {
                    "iterations": int(timing.get("worker_pan_iterations", 0.0)),
                    "early_converged": bool(timing.get("worker_pan_early_converged", 0.0)),
                    "dune_effective_points_count": int(timing.get("worker_dune_effective_points_count", 0.0)),
                    "nrmp_effective_points_count": int(timing.get("worker_nrmp_effective_points_count", 0.0)),
                },
                "goal_distance": float(np.linalg.norm(
                    state[:2, 0] - (goal[:2, 0] if goal is not None else path[:2, -1])
                )),
                "output_publish_monotonic": float(output_publish),
                "transport_trace": {
                    key: float(timing[key]) for key in (
                        "cycle_start_monotonic",
                        "worker_core_start_monotonic",
                        "worker_core_end_monotonic",
                        "transport_request_send_started_monotonic",
                        "transport_request_send_finished_monotonic",
                        "transport_wait_started_monotonic",
                        "transport_response_available_monotonic",
                        "transport_response_received_monotonic",
                        "transport_response_decoded_monotonic",
                    ) if key in timing
                } | {
                    "cmd_publish_monotonic": float(cmd_publish_monotonic),
                    "output_publish_monotonic": float(output_publish),
                    "main_full_cycle_ms": float(timing.get("main_full_cycle_ms", 0.0)),
                    "worker_core_total_ms": float(timing.get("worker_core_total_ms", 0.0)),
                    "scheduler_wakeup_to_cycle_start_ms": float(
                        timing.get("scheduler_wakeup_to_cycle_start_ms", 0.0)),
                },
                "scheduler": self._planner_scheduler.snapshot(),
                "info": dict(record.get("info", {})),
                "navigation_mode": self.navigation_mode,
                "loop": self.navigation_loop,
                "active_gear": active_gear,
                "requested_gear": requested_gear,
                "gear_transition_pending": gear_transition_pending,
                "gear_values": [int(value) for value in path[3, :].tolist()],
            })
            # Keep the same complete numeric evidence in the durable bridge
            # record and on the semantic topic consumed by the N2 audit.
            record["cycle_evidence"] = dict(semantic)
            semantic_message = String()
            semantic_message.data = json.dumps(semantic, allow_nan=False, sort_keys=True)
            self.semantic_pub.publish(semantic_message)
        finally:
            with self._active_lock:
                self._active_request = False

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self._shutdown_requested = True
        scheduler = getattr(self, "_planner_scheduler", None)
        if scheduler is not None:
            stopped = scheduler.stop(float(self.get_parameter("n115_timeout_sec").value) + 1.0)
            if not stopped:
                self.get_logger().error("N1.16 planner scheduler did not stop before finalization")
        try:
            self.timer.cancel()
        except Exception:
            pass
        self.client.shutdown["active_request_observed"] = bool(self._active_request)
        deadline = time.monotonic() + max(2.5, float(self.get_parameter("n115_timeout_sec").value) + 1.0)
        while self._active_request and time.monotonic() < deadline:
            time.sleep(0.01)
        self.client.set_scan_accounting(
            scans_received=self.scans_received,
            unique_processed=self.unique_processed,
            obsolete_dropped=self.obsolete_dropped,
            duplicate=self.duplicate_scans,
        )
        self.client.finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    # N2.4C6 Gate A: optional benchmark-only executor probe.  Disabled unless
    # R1_N116_EXEC_PROBE=1; observational only, no planner behaviour change.
    probe = None
    if os.environ.get("R1_N116_EXEC_PROBE"):
        from r1_neupan_integration.n1_16_exec_probe import install

        probe = install(N116Node)
    node = N116Node()
    if probe is not None:
        probe.install_node_hooks(node)
    # Scan, live-arm geometry, and path/status callbacks must remain
    # independently serviceable while the isolated planner is solving.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
