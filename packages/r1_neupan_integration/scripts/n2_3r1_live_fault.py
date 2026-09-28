#!/usr/bin/env python3
"""N2.3R1 live fail-safe regression runner.

This runner only orchestrates external fault injection and records ROS topic
evidence. It does not alter safety thresholds, command semantics, or planner
parameters.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64MultiArray, String


TIMING_FIELDS = [
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
]


def finite(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def process_cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
    except (FileNotFoundError, PermissionError, UnicodeDecodeError):
        return ""


def process_parent(pid: int) -> int | None:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        tail = stat.rsplit(")", 1)[1].split()
        return int(tail[1])
    except (FileNotFoundError, PermissionError, ValueError, IndexError):
        return None


def all_pids() -> list[int]:
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        result.append(pid)
    return sorted(result)


def find_pids(fragment: str) -> list[int]:
    return [pid for pid in all_pids() if fragment in process_cmdline(pid)]


class LiveEvidence(Node):
    def __init__(self, started: float, output: Path):
        super().__init__("r1_n2_3r1_live_evidence")
        self.started = started
        self.output = output
        self.gated: list[dict] = []
        self.raw: list[dict] = []
        self.planner: list[dict] = []
        self.relay: list[dict] = []
        self.gates: list[dict] = []
        self.bridge: list[dict] = []
        self.odom: list[dict] = []
        self.timing: list[dict] = []
        self.contacts: list[dict] = []
        self.last_worker_pid: int | None = None
        self.last_worker_generation: int | None = None
        self.last_request_id: int | None = None
        self.last_planner_result_monotonic: float | None = None
        self._last_nonzero_t: float | None = None
        self._last_nonzero_odom_t: float | None = None

        self.create_subscription(Twist, "/cmd_vel", self._gated_cb, 100)
        self.create_subscription(Twist, "/neupan_cmd_vel", self._raw_cb, 100)
        self.create_subscription(String, "/neupan_planner_result", self._planner_cb, 100)
        self.create_subscription(String, "/neupan_cmd_envelope", self._relay_cb, 100)
        self.create_subscription(String, "/r1_safety_gate_status", self._gate_cb, 100)
        self.create_subscription(String, "/neupan_bridge_status", self._bridge_cb, 100)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 100)
        self.create_subscription(Float64MultiArray, "/neupan_timing", self._timing_cb, 100)
        self.create_subscription(Contacts, "/world/empty/contacts", self._contacts_cb, 20)

    def t(self) -> float:
        return time.monotonic() - self.started

    def _json(self, message: String) -> dict:
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {"malformed": True}
        except (TypeError, ValueError, json.JSONDecodeError):
            return {"malformed": True, "raw": str(message.data)}

    def _gated_cb(self, message: Twist) -> None:
        self.gated.append({"t": self.t(), "monotonic": time.monotonic(),
                           "linear_x": float(message.linear.x),
                           "angular_z": float(message.angular.z)})

    def _raw_cb(self, message: Twist) -> None:
        self.raw.append({"t": self.t(), "monotonic": time.monotonic(),
                         "linear_x": float(message.linear.x),
                         "angular_z": float(message.angular.z)})

    def _planner_cb(self, message: String) -> None:
        payload = self._json(message)
        payload["audit_t"] = self.t()
        payload["receive_monotonic"] = time.monotonic()
        self.planner.append(payload)
        self.last_request_id = payload.get("request_id")
        self.last_planner_result_monotonic = finite(payload.get("planner_result_monotonic"))
        generation = payload.get("worker_generation")
        self.last_worker_generation = int(generation) if isinstance(generation, int) else self.last_worker_generation

    def _relay_cb(self, message: String) -> None:
        payload = self._json(message)
        payload["audit_t"] = self.t()
        payload["receive_monotonic"] = time.monotonic()
        self.relay.append(payload)

    def _gate_cb(self, message: String) -> None:
        value = self._json(message)
        value["t"] = self.t()
        value["receive_monotonic"] = time.monotonic()
        self.gates.append(value)

    def _bridge_cb(self, message: String) -> None:
        value = self._json(message)
        value["t"] = self.t()
        value["receive_monotonic"] = time.monotonic()
        self.bridge.append(value)

    def _odom_cb(self, message: Odometry) -> None:
        value = {
            "t": self.t(), "receive_monotonic": time.monotonic(),
            "linear_x": float(message.twist.twist.linear.x),
            "linear_y": float(message.twist.twist.linear.y),
            "angular_z": float(message.twist.twist.angular.z),
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
        }
        self.odom.append(value)

    def _timing_cb(self, message: Float64MultiArray) -> None:
        if len(message.data) < len(TIMING_FIELDS):
            return
        value = dict(zip(TIMING_FIELDS, [float(item) for item in message.data]))
        value["t"] = self.t()
        value["receive_monotonic"] = time.monotonic()
        self.timing.append(value)
        worker_pid = finite(value.get("worker_pid"))
        if worker_pid is not None and worker_pid > 0:
            self.last_worker_pid = int(worker_pid)

    def _contacts_cb(self, message: Contacts) -> None:
        if not message.contacts:
            return
        self.contacts.append({
            "t": self.t(), "contacts": [
                {"collision1": item.collision1.name, "collision2": item.collision2.name,
                 "depths": [float(depth) for depth in item.depths]}
                for item in message.contacts
            ],
        })

    def latest_gate(self) -> dict:
        return self.gates[-1] if self.gates else {}

    def released_and_moving(self) -> bool:
        gate = self.latest_gate()
        if gate.get("emergency_stop") is not False or gate.get("failure_latched"):
            return False
        for row in reversed(self.gated[-40:]):
            if row["t"] < self.t() - 0.8:
                break
            if row["linear_x"] > 0.02:
                self._last_nonzero_t = row["t"]
                return True
        return False

    def worker_pid(self) -> int | None:
        if self.last_worker_pid and process_cmdline(self.last_worker_pid):
            return self.last_worker_pid
        parents = find_pids("n1_16_node")
        children = [pid for pid in all_pids() if process_parent(pid) in parents]
        for pid in children:
            command = process_cmdline(pid)
            if command and "resource_tracker" not in command:
                return pid
        return None

    def freeze_planner_result(self) -> bool:
        candidates = find_pids("n1_16_node")
        target = max(candidates) if candidates else None
        if target is None or not process_cmdline(target):
            return False
        self.freeze_target_pid = target
        self.freeze_target_cmdline = process_cmdline(target)
        os.kill(target, signal.SIGUSR1)
        return True

    def write(self, metadata: dict) -> None:
        payload = {
            "schema": "r1_n2_3r1_live_fault_v1",
            **metadata,
            "planner_result": self.planner,
            "relay_envelope": self.relay,
            "gate": self.gates,
            "bridge": self.bridge,
            "raw_cmd": self.raw,
            "gated_cmd": self.gated,
            "odom": self.odom,
            "timing": self.timing,
            "contacts": self.contacts,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def first_t(rows: list[dict], predicate) -> float | None:
    for row in rows:
        if predicate(row):
            return finite(row.get("t"))
    return None


def source_age(row: dict, key: str) -> float | None:
    ages = row.get("ages_sec", {})
    return finite(ages.get(key))


def summary(evidence: LiveEvidence, fault: str, fault_t: float | None,
            injection: dict, launch_rc: int | None, smoke: bool = False) -> dict:
    gates = [row for row in evidence.gates if fault_t is None or row.get("t", -1) >= fault_t]
    gated_after = [row for row in evidence.gated if fault_t is None or row.get("t", -1) >= fault_t]
    motion_t = first_t(evidence.gated, lambda row: row.get("linear_x", 0.0) > 0.02)
    zero_rows = gated_after
    if smoke and motion_t is not None:
        zero_rows = [row for row in evidence.gated if row.get("t", -1) > motion_t]
    zero_t = first_t(zero_rows, lambda row: abs(row.get("linear_x", math.inf)) <= 1e-5 and abs(row.get("angular_z", math.inf)) <= 1e-5)
    if zero_t is None:
        zero_t = first_t(gates, lambda row: row.get("zeroed") is True or (row.get("output_linear_x") == 0.0 and row.get("output_angular_z") == 0.0))
    planner_cross = first_t(gates, lambda row: (source_age(row, "planner_result") or -1.0) > 0.30)
    relay_cross = first_t(gates, lambda row: (source_age(row, "relay") or -1.0) > 0.30)
    if fault == "lf1":
        detection_t = first_t(gates, lambda row: "planner_freshness_watchdog" in row.get("reasons", []))
        expected_reason = "planner_freshness_watchdog"
    elif fault == "lf2":
        detection_t = first_t(gates, lambda row: "relay_watchdog" in row.get("reasons", []))
        expected_reason = "relay_watchdog"
    elif fault == "lf3":
        detection_t = first_t(evidence.bridge, lambda row: row.get("status") == "fail-safe")
        expected_reason = "worker_crash"
    else:
        detection_t = None
        expected_reason = None
    if fault == "lf3" and detection_t is None:
        detection_t = first_t(gates, lambda row: any(reason in row.get("reasons", []) for reason in ("worker_crash", "bridge_failure")))
    settle_t = None
    zero_streak_start = None
    for row in evidence.odom:
        if fault_t is not None and row["t"] < fault_t:
            continue
        if smoke and motion_t is not None and row["t"] <= motion_t:
            continue
        if math.hypot(row["linear_x"], row["linear_y"]) <= 0.01 and abs(row["angular_z"]) <= 0.01:
            zero_streak_start = row["t"] if zero_streak_start is None else zero_streak_start
            if row["t"] - zero_streak_start >= 0.50:
                settle_t = zero_streak_start
                break
        else:
            zero_streak_start = None
    reason_timeline = [
        {"t": row.get("t"), "reasons": row.get("reasons", []),
         "ages_sec": row.get("ages_sec", {}), "output_linear_x": row.get("output_linear_x"),
         "output_angular_z": row.get("output_angular_z"),
         "emergency_stop": row.get("emergency_stop"),
         "failure_latched": row.get("failure_latched")}
        for row in gates if row.get("reasons")
    ]
    fault_abs = None if fault_t is None else evidence.started + fault_t
    planner_after = [
        row for row in evidence.planner
        if fault_abs is not None and (finite(row.get("planner_result_monotonic")) or math.inf) >= fault_abs
    ]
    relay_after = [
        row for row in evidence.relay
        if fault_abs is not None and row.get("receive_monotonic", -1) >= fault_abs
    ]
    max_gated_linear = max((row["linear_x"] for row in evidence.gated), default=None)
    min_gated_linear = min((row["linear_x"] for row in evidence.gated), default=None)
    nonfinite = sum(not math.isfinite(row["linear_x"]) or not math.isfinite(row["angular_z"]) for row in evidence.gated)
    queue_values = [row.get("queue_depth") for row in evidence.timing if finite(row.get("queue_depth")) is not None]
    worker_values = [row.get("worker_core_total_ms") for row in evidence.timing if finite(row.get("worker_core_total_ms")) is not None]
    main_values = [row.get("main_full_cycle_ms") for row in evidence.timing if finite(row.get("main_full_cycle_ms")) is not None]
    active_watchdogs = sorted({
        reason for row in gates
        if row.get("emergency_stop") is False
        for reason in row.get("watchdog_triggered", [])
    })
    pass_result = (
        (smoke and motion_t is not None and len(evidence.planner) >= 8
         and len(evidence.relay) >= 40 and not active_watchdogs and zero_t is not None
         and settle_t is not None)
        or (not smoke and detection_t is not None and zero_t is not None and settle_t is not None
            and ((fault == "lf1" and expected_reason in active_watchdogs and "relay_watchdog" not in active_watchdogs)
                 or (fault == "lf2" and expected_reason in active_watchdogs and "planner_freshness_watchdog" not in active_watchdogs)
                 or (fault == "lf3" and any(event.get("status") == "fail-safe" for event in evidence.bridge))))
    )
    return {
        "schema": "r1_n2_3r1_live_fault_summary_v1",
        "fault": fault,
        "smoke": smoke,
        "pass": bool(pass_result),
        "launch_returncode": launch_rc,
        "fault_injection": {**injection, "t": fault_t},
        "timeline": {
            "fault_t": fault_t,
            "planner_freshness_threshold_crossing_t": planner_cross,
            "relay_threshold_crossing_t": relay_cross,
            "fault_detection_t": detection_t,
            "zero_cmd_t": zero_t,
            "robot_settled_t": settle_t,
            "fault_to_zero_latency_sec": None if fault_t is None or zero_t is None else zero_t - fault_t,
            "fault_to_robot_stop_latency_sec": None if fault_t is None or settle_t is None else settle_t - fault_t,
            "threshold_to_zero_latency_sec": (
                None if zero_t is None else max(0.0, (
                    zero_t - planner_cross if fault == "lf1" and planner_cross is not None else
                    zero_t - relay_cross if fault == "lf2" and relay_cross is not None else
                    zero_t - detection_t if fault == "lf3" and detection_t is not None else 0.0
                ))
            ),
        },
        "last_planner_result": {
            "request_id": evidence.planner[-1].get("request_id") if evidence.planner else None,
            "planner_result_monotonic": evidence.last_planner_result_monotonic,
            "source_age_at_last_gate": source_age(evidence.gates[-1], "planner_result") if evidence.gates else None,
        },
        "rates_and_ages": {
            "planner_count": len(evidence.planner), "planner_after_fault_count": len(planner_after),
            "relay_count": len(evidence.relay), "relay_after_fault_count": len(relay_after),
            "planner_last_before_fault_request_id": (
                max((row.get("request_id", -1) for row in evidence.planner
                     if fault_abs is not None and (finite(row.get("planner_result_monotonic")) or -math.inf) < fault_abs), default=None)
            ),
            "planner_age_trace_after_fault": [{"t": row.get("t"), "age": source_age(row, "planner_result")} for row in gates],
            "relay_age_trace_after_fault": [{"t": row.get("t"), "age": source_age(row, "relay")} for row in gates],
        },
        "watchdog_reason_timeline": reason_timeline,
        "status": {
            "bridge_events": evidence.bridge,
            "active_watchdogs": active_watchdogs,
            "e_stop_final": evidence.gates[-1].get("emergency_stop") if evidence.gates else None,
            "failure_latched_final": evidence.gates[-1].get("failure_latched") if evidence.gates else None,
        },
        "invariants": {
            "max_gated_linear_x": max_gated_linear,
            "min_gated_linear_x": min_gated_linear,
            "no_reverse": min_gated_linear is None or min_gated_linear >= 0.0,
            "contact_count": len(evidence.contacts),
            "contact": len(evidence.contacts) == 0,
            "stale_publish": sum(row.get("stale_response", 0.0) != 0.0 for row in evidence.timing),
            "nan_inf": nonfinite + sum(row.get("nan_inf", 0.0) != 0.0 for row in evidence.timing),
            "queue_depth_max": max(queue_values, default=None),
            "queue_ok": max(queue_values, default=0.0) <= 1.0,
            "worker_core_latency_ms_max": max(worker_values, default=None),
            "worker_main_latency_ms_max": max(main_values, default=None),
        },
        "evidence_file": str(evidence.output),
    }


def run(args: argparse.Namespace) -> int:
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    launch_log = Path(args.launch_log).resolve()
    launch_log.parent.mkdir(parents=True, exist_ok=True)
    world = Path(args.world_file).resolve()
    bridge_dir = output.with_suffix("").with_name(output.stem + "_bridge")
    launch_cmd = [
        "ros2", "launch", "r1_neupan_integration", "n2_3_freshness.launch.py",
        "scenario:=m1_forward", f"duration_sec:={args.duration}",
        f"world_file:={world}", f"output_file:={output}",
        f"n116_output_dir:={bridge_dir}", "capture_detail:=true",
    ]
    started = time.monotonic()
    with launch_log.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(launch_cmd, stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        rclpy.init(args=None)
        evidence = LiveEvidence(started, output.with_suffix(".live.json"))
        executor = SingleThreadedExecutor()
        executor.add_node(evidence)
        fault_t = None
        injection = {"method": "none", "target_pid": None, "target_cmdline": None}
        try:
            deadline = time.monotonic() + args.duration + 12.0
            while time.monotonic() < deadline and launch.poll() is None:
                executor.spin_once(timeout_sec=0.05)
                if args.fault != "smoke" and fault_t is None and evidence.released_and_moving():
                    if args.fault == "lf1":
                        if not evidence.freeze_planner_result():
                            raise RuntimeError("failed to resolve LF1 bridge/source process")
                        target = getattr(evidence, "freeze_target_pid", None)
                        method = "SIGUSR1: freeze planner-result publication only"
                        target_cmdline = getattr(evidence, "freeze_target_cmdline", None)
                    elif args.fault == "lf2":
                        candidates = find_pids("n2_3_command_relay")
                        target = max(candidates) if candidates else None
                        method = "SIGSTOP command relay process"
                    else:
                        target = evidence.worker_pid()
                        method = "SIGKILL whole-core worker pid from timing stream"
                    if args.fault != "lf1" and (target is None or not process_cmdline(target)):
                        raise RuntimeError(f"unable to resolve live fault target for {args.fault}")
                    target_cmdline = locals().get("target_cmdline") if args.fault == "lf1" else process_cmdline(target)
                    if target is not None and args.fault != "lf1":
                        os.kill(target, signal.SIGKILL if args.fault == "lf3" else signal.SIGSTOP)
                    fault_t = evidence.t()
                    injection = {"method": method, "target_pid": target,
                                 "target_cmdline": target_cmdline}
                if fault_t is not None:
                    detection = summary(evidence, args.fault, fault_t, injection, None)
                    timeline = detection["timeline"]
                    if timeline["zero_cmd_t"] is not None and timeline["robot_settled_t"] is not None:
                        break
            if fault_t is not None and args.fault != "lf3":
                target = injection.get("target_pid")
                if target and process_cmdline(int(target)):
                    os.kill(int(target), signal.SIGCONT)
            if launch.poll() is None:
                os.killpg(os.getpgid(launch.pid), signal.SIGINT)
            try:
                launch.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(launch.pid), signal.SIGTERM)
                launch.wait(timeout=10.0)
            for _ in range(20):
                executor.spin_once(timeout_sec=0.05)
        finally:
            if fault_t is not None and args.fault != "lf3":
                target = injection.get("target_pid")
                if target and process_cmdline(int(target)):
                    os.kill(int(target), signal.SIGCONT)
            if launch.poll() is None:
                os.killpg(os.getpgid(launch.pid), signal.SIGTERM)
                launch.wait(timeout=10.0)
            evidence.write({"fault": args.fault, "duration_requested_sec": args.duration,
                            "started_monotonic": started})
            executor.shutdown()
            evidence.destroy_node()
            rclpy.shutdown()
    result = summary(evidence, args.fault, fault_t, injection, launch.returncode, args.fault == "smoke")
    result_path = output.with_suffix(".summary.json")
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["pass"] else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fault", choices=("lf1", "lf2", "lf3", "smoke"), required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--launch-log", required=True)
    parser.add_argument("--world-file", default="src/r1_neupan_integration/worlds/free_space.sdf")
    parser.add_argument("--duration", type=float, default=18.0)
    return run(parser.parse_args())


if __name__ == "__main__":
    sys.exit(main())
