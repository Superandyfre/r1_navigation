"""N2.4C-R monotonic pipeline and lightweight resource attribution audit.

This node observes the frozen N1.16/N2.3/N2-PRE data path.  It does not
publish commands, alter watchdogs, or modify planner input/state.  T0 is the
earliest observable PointCloud2 receipt in this process; Gazebo production
time is intentionally not fabricated because it is not carried in a
monotonic domain.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import statistics
import time
from pathlib import Path
from typing import Any

import psutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist


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
    "cloud_receive_monotonic", "scan_callback_receive_monotonic",
    "scan_update_monotonic", "request_created_monotonic",
    "request_dispatched_monotonic", "worker_request_received_monotonic",
    "worker_solve_start_monotonic", "worker_solve_end_monotonic",
    "worker_response_sent_monotonic", "main_response_received_monotonic",
    "main_validation_complete_monotonic", "planner_result_publish_monotonic",
    "relay_source_receive_monotonic", "relay_publish_monotonic",
    "gate_relay_receive_monotonic", "gate_output_publish_monotonic",
)

PIPELINE = (
    ("T0", "cloud_receive_monotonic"),
    ("T1", "scan_callback_receive_monotonic"),
    ("T2", "scan_update_monotonic"),
    ("T3", "request_created_monotonic"),
    ("T4", "request_dispatched_monotonic"),
    ("T5", "worker_request_received_monotonic"),
    ("T6", "worker_solve_start_monotonic"),
    ("T7", "worker_solve_end_monotonic"),
    ("T8", "worker_response_sent_monotonic"),
    ("T9", "main_response_received_monotonic"),
    ("T10", "main_validation_complete_monotonic"),
    ("T11", "planner_result_publish_monotonic"),
    ("T12", "relay_source_receive_monotonic"),
    ("T13", "relay_publish_monotonic"),
    ("T14", "gate_relay_receive_monotonic"),
    ("T15", "gate_output_publish_monotonic"),
)

GAP_THRESHOLD_SEC = 0.25
SAFETY_TIMEOUT_SEC = 0.30


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _clean(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if hasattr(value, "item"):
        try:
            return _clean(value.item())
        except Exception:
            pass
    return value


def _stamp(message: Any) -> float:
    header = getattr(message, "header", None)
    if header is None:
        return -1.0
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def _stats(values: list[float]) -> dict[str, Any]:
    finite = sorted(float(value) for value in values if _finite(value))
    if not finite:
        return {"count": 0, "total": len(values), "nonfinite": len(values)}
    return {
        "count": len(finite),
        "total": len(values),
        "nonfinite": len(values) - len(finite),
        "min": finite[0],
        "max": finite[-1],
        "mean": statistics.fmean(finite),
        "p95": finite[int(0.95 * (len(finite) - 1))],
        "p99": finite[int(0.99 * (len(finite) - 1))],
    }


def _read(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _psi(path: Path) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    content = _read(path)
    if not content:
        return result
    for line in content.splitlines():
        parts = line.split()
        if not parts:
            continue
        values: dict[str, float] = {}
        for token in parts[1:]:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            try:
                values[key] = float(value)
            except ValueError:
                continue
        result[parts[0]] = values
    return result


def _cgroup_state() -> dict[str, Any]:
    cgroup = _read(Path("/proc/self/cgroup")) or ""
    relative = ""
    for line in cgroup.splitlines():
        if line.startswith("0::"):
            relative = line[3:]
            break
    root = Path("/sys/fs/cgroup") / relative.lstrip("/")
    current = _read(root / "pids.current")
    maximum = _read(root / "pids.max")
    return {
        "path": relative or None,
        "tasks_current": int(current) if current and current.isdigit() else None,
        "tasks_max": (int(maximum) if maximum and maximum.isdigit() else maximum),
    }


def _classify(command: str) -> str | None:
    value = command.lower()
    patterns = (
        ("gazebo_server", ("gz sim", "gzserver", "gz sim-server")),
        ("ros_gz_bridge", ("parameter_bridge",)),
        ("pointcloud_to_laserscan", ("pointcloud_to_laserscan",)),
        ("n1_main", ("n1_16_node", "n1_16_production_bridge", "n1_15_node")),
        ("whole_core_worker", ("r1_n1_16_whole_core",)),
        ("command_relay", ("n2_3_command_relay",)),
        ("safety_gate", ("r1_safety_gate", "r1_n2_pre_safety_gate")),
        ("audit", ("n2_pre3_audit", "n2_4c_runtime_attribution")),
        ("scenario", ("n2_pre_scenarios",)),
        ("rviz", ("rviz2",)),
    )
    for label, needles in patterns:
        if any(needle in value for needle in needles):
            return label
    return None


class RuntimeAttributionCollector:
    """Attach observations to an existing ROS node without adding a process."""

    def __init__(self, node: Node, output_file: str, run_label: str = "unknown") -> None:
        self._node = node
        self._started = time.monotonic()
        self._output = Path(str(output_file)).expanduser().resolve()
        self._run_label = str(run_label)
        self._clouds: list[dict[str, Any]] = []
        self._scans: list[dict[str, Any]] = []
        self._scan_counter_observer = 0
        self._timings: list[dict[str, Any]] = []
        self._sources: list[dict[str, Any]] = []
        self._relays: list[dict[str, Any]] = []
        self._gates: list[dict[str, Any]] = []
        self._bridges: list[dict[str, Any]] = []
        self._commands: list[dict[str, Any]] = []
        self._resources: list[dict[str, Any]] = []
        self._cpu_previous: dict[int, tuple[float, float]] = {}
        self._known_worker_pids: set[int] = set()
        self._finalized = False
        self._overhead = self._measure_overhead()

        sensor_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._node.create_subscription(
            PointCloud2, "/lidar/points",
            self._cloud_callback, sensor_qos,
        )
        self._node.create_subscription(
            LaserScan, "/scan_neupan",
            self._scan_callback, sensor_qos,
        )
        self._node.create_subscription(
            Float64MultiArray, "/neupan_timing",
            self._timing_callback, 100,
        )
        self._node.create_subscription(String, "/neupan_planner_result", self._source_callback, 100)
        self._node.create_subscription(String, "/neupan_cmd_envelope", self._relay_callback, 100)
        self._node.create_subscription(String, "/r1_safety_gate_status", self._gate_callback, 100)
        self._node.create_subscription(String, "/neupan_bridge_status", self._bridge_callback, 100)
        self._node.create_subscription(Twist, "/cmd_vel", self._command_callback, 100)
        # Five hertz is sufficient for resource attribution and keeps the
        # observer itself below the timing scale under investigation.
        self._resource_timer = self._node.create_timer(0.2, self._resource_callback)

    @staticmethod
    def _measure_overhead() -> dict[str, Any]:
        started = time.perf_counter_ns()
        for _ in range(10000):
            time.monotonic()
        elapsed = (time.perf_counter_ns() - started) / 1.0e6
        return {
            "monotonic_calls": 10000,
            "monotonic_total_ms": elapsed,
            "monotonic_per_call_ns": elapsed * 100000.0,
            "resource_sample_hz": 5.0,
            "resource_sampler": "psutil process_iter + procfs PSI/cgroup; no profiler",
        }

    def _event_time(self) -> float:
        return time.monotonic()

    @staticmethod
    def _json_message(message: String) -> dict[str, Any]:
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {"malformed": True}
        except (TypeError, ValueError, json.JSONDecodeError):
            return {"malformed": True, "raw": str(message.data)}

    def _cloud_callback(self, message: PointCloud2) -> None:
        receive = self._event_time()
        self._clouds.append({
            "receive_monotonic": receive,
            "header_stamp_sec": _stamp(message),
            "frame_id": str(message.header.frame_id),
            "point_count": int(message.width) * int(message.height),
            "point_step": int(message.point_step),
            "row_step": int(message.row_step),
            "is_dense": bool(message.is_dense),
        })

    def _scan_callback(self, message: LaserScan) -> None:
        receive = self._event_time()
        self._scan_counter_observer += 1
        finite = [float(value) for value in message.ranges if _finite(value)]
        self._scans.append({
            "receive_monotonic": receive,
            "header_stamp_sec": _stamp(message),
            "frame_id": str(message.header.frame_id),
            "range_count": len(message.ranges),
            "finite_count": len(finite),
            "min_range": min(finite) if finite else None,
            "scan_counter_observer": self._scan_counter_observer,
        })

    def _timing_callback(self, message: Float64MultiArray) -> None:
        receive = self._event_time()
        data = list(message.data)
        if len(data) < len(TIMING_FIELDS) or float(data[0]) != 4.0:
            return
        record = {field: float(value) for field, value in zip(TIMING_FIELDS, data)}
        worker_pid = int(record.get("worker_pid", -1.0))
        if worker_pid > 0:
            self._known_worker_pids.add(worker_pid)
        record["audit_receive_monotonic"] = receive
        self._timings.append(record)

    def _source_callback(self, message: String) -> None:
        self._sources.append({
            "receive_monotonic": self._event_time(),
            "payload": self._json_message(message),
        })

    def _relay_callback(self, message: String) -> None:
        self._relays.append({
            "receive_monotonic": self._event_time(),
            "payload": self._json_message(message),
        })

    def _gate_callback(self, message: String) -> None:
        self._gates.append({
            "receive_monotonic": self._event_time(),
            "payload": self._json_message(message),
        })

    def _bridge_callback(self, message: String) -> None:
        self._bridges.append({
            "receive_monotonic": self._event_time(),
            "payload": self._json_message(message),
        })

    def _command_callback(self, message: Twist) -> None:
        self._commands.append({
            "receive_monotonic": self._event_time(),
            "linear_x": float(message.linear.x),
            "angular_z": float(message.angular.z),
        })

    def _resource_callback(self) -> None:
        started = time.perf_counter_ns()
        now = time.monotonic()
        processes: list[dict[str, Any]] = []
        process_count = 0
        thread_count = 0
        for process in psutil.process_iter(["pid", "name", "cmdline", "num_threads"]):
            try:
                process_count += 1
                thread_count += int(process.info.get("num_threads") or 0)
                pid = int(process.info["pid"])
                command = " ".join(process.info.get("cmdline") or [])
                label = (
                    "whole_core_worker" if pid in self._known_worker_pids else
                    _classify(command or str(process.info.get("name") or ""))
                )
                if label is None:
                    continue
                cpu_times = process.cpu_times()
                cpu_total = float(cpu_times.user + cpu_times.system)
                previous = self._cpu_previous.get(pid)
                cpu_percent = None
                if previous is not None:
                    previous_cpu, previous_wall = previous
                    wall_delta = now - previous_wall
                    if wall_delta > 0.0:
                        cpu_percent = max(0.0, (cpu_total - previous_cpu) / wall_delta * 100.0)
                self._cpu_previous[pid] = (cpu_total, now)
                memory = process.memory_info()
                context = process.num_ctx_switches()
                processes.append({
                    "label": label,
                    "pid": pid,
                    "command": command[:500],
                    "cpu_percent": cpu_percent,
                    "rss_bytes": int(memory.rss),
                    "thread_count": int(process.info.get("num_threads") or 0),
                    "voluntary_context_switches": int(context.voluntary),
                    "involuntary_context_switches": int(context.involuntary),
                })
            except (psutil.Error, OSError, ValueError):
                continue
        load = os.getloadavg() if hasattr(os, "getloadavg") else (None, None, None)
        memory = psutil.virtual_memory()
        swap = psutil.swap_memory()
        sample = {
            "monotonic": now,
            "loadavg": list(load),
            "runnable_processes": _read(Path("/proc/loadavg")),
            "process_count": process_count,
            "system_thread_count": thread_count,
            "cpu_count": psutil.cpu_count() or 1,
            "memory": {
                "total_bytes": int(memory.total), "available_bytes": int(memory.available),
                "percent": float(memory.percent),
            },
            "swap": {
                "total_bytes": int(swap.total), "used_bytes": int(swap.used),
                "percent": float(swap.percent),
            },
            "psi": {
                "cpu": _psi(Path("/proc/pressure/cpu")),
                "memory": _psi(Path("/proc/pressure/memory")),
                "io": _psi(Path("/proc/pressure/io")),
            },
            "cgroup": _cgroup_state(),
            "processes": processes,
        }
        sample["sample_duration_ms"] = (time.perf_counter_ns() - started) / 1.0e6
        self._resources.append(sample)

    @staticmethod
    def _valid_timestamp(value: Any) -> bool:
        return _finite(value) and float(value) > 0.0

    @staticmethod
    def _payload_cycle(item: dict[str, Any]) -> int | None:
        payload = item.get("payload", {})
        try:
            return int(payload.get("cycle_id"))
        except (TypeError, ValueError):
            return None

    def _cloud_for_scan(self, scan_stamp: float, scan_receive: float) -> dict[str, Any] | None:
        candidates = [
            item for item in self._clouds
            if self._valid_timestamp(scan_stamp)
            and abs(float(item.get("header_stamp_sec", -1.0)) - scan_stamp) <= 1.0e-6
        ]
        if candidates:
            return min(candidates, key=lambda item: abs(item["receive_monotonic"] - scan_receive))
        preceding = [item for item in self._clouds if item["receive_monotonic"] <= scan_receive]
        return preceding[-1] if preceding else None

    def _relay_for_cycle(self, cycle: int) -> dict[str, Any] | None:
        candidates = [item for item in self._relays if self._payload_cycle(item) == cycle]
        return min(candidates, key=lambda item: item["receive_monotonic"]) if candidates else None

    def _gate_for_cycle(self, cycle: int) -> dict[str, Any] | None:
        candidates = []
        for item in self._gates:
            payload = item.get("payload", {})
            metadata = (payload.get("freshness") or {}).get("source_metadata") or {}
            try:
                if int(metadata.get("cycle_id")) == cycle:
                    candidates.append(item)
            except (TypeError, ValueError):
                continue
        return min(candidates, key=lambda item: item["receive_monotonic"]) if candidates else None

    def _timeline(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for timing in sorted(self._timings, key=lambda item: item.get("cycle_id", -1.0)):
            cycle = int(timing["cycle_id"])
            scan_receive = float(timing.get("scan_receive_monotonic", -1.0))
            cloud = self._cloud_for_scan(float(timing.get("scan_stamp_sec", -1.0)), scan_receive)
            relay = self._relay_for_cycle(cycle)
            gate = self._gate_for_cycle(cycle)
            row: dict[str, Any] = {
                "cycle_id": cycle,
                "request_id": int(timing.get("request_id", -1.0)),
                "scan_counter": int(timing.get("scan_counter", -1.0)),
                "scan_stamp_sec": float(timing.get("scan_stamp_sec", -1.0)),
                "scan_source_header_stamp_sec": cloud.get("header_stamp_sec") if cloud else None,
                "scan_source_observed": cloud is not None,
                "scan_receive_observer_monotonic": (
                    cloud.get("receive_monotonic") if cloud else None
                ),
                "scan_counter_observer": None,
                "scan_age_sec_at_planner_start": (
                    float(timing.get("worker_core_start_monotonic", -1.0))
                    - float(timing.get("scan_receive_monotonic", -1.0))
                    if self._valid_timestamp(timing.get("worker_core_start_monotonic"))
                    and self._valid_timestamp(timing.get("scan_receive_monotonic")) else None
                ),
                "planner_source_age_sec": None,
                "relay_age_sec": None,
                "timing_age_sec": None,
                "queue_depth": float(timing.get("queue_depth", 0.0)),
                "worker_core_total_ms": float(timing.get("worker_core_total_ms", 0.0)),
                "worker_core_cpu_ms": float(timing.get("worker_core_cpu_ms", 0.0)),
                "main_full_cycle_ms": float(timing.get("main_full_cycle_ms", 0.0)),
                "worker_pid": int(timing.get("worker_pid", -1.0)),
                "worker_generation": int(timing.get("worker_generation", -1.0)),
                "worker_failure": float(timing.get("worker_failure", 0.0)),
                "finite": float(timing.get("finite", 0.0)),
                "nan_inf": float(timing.get("nan_inf", 0.0)),
                "stale_response": float(timing.get("stale_response", 0.0)),
                "watchdog_reasons": [],
                "emergency_stop": None,
                "failure_latched": None,
            }
            for label, field in PIPELINE:
                if label == "T0":
                    row[field] = cloud.get("receive_monotonic") if cloud else None
                elif label in {"T12", "T13"}:
                    payload = relay.get("payload", {}) if relay else {}
                    row[field] = payload.get(field)
                elif label in {"T14", "T15"}:
                    payload = gate.get("payload", {}) if gate else {}
                    callback = payload.get("callback_receive_monotonic") or {}
                    row[field] = (
                        callback.get("relay_receive_monotonic")
                        if field == "gate_relay_receive_monotonic" else
                        payload.get("output_publish_monotonic")
                    )
                else:
                    row[field] = timing.get(field)
                row[label] = row[field]
            if gate:
                payload = gate.get("payload", {})
                ages = payload.get("ages_sec") or {}
                row["planner_source_age_sec"] = ages.get("planner_result")
                row["relay_age_sec"] = ages.get("relay")
                row["timing_age_sec"] = ages.get("timing")
                row["watchdog_reasons"] = list(payload.get("watchdog_triggered") or [])
                row["emergency_stop"] = payload.get("emergency_stop")
                row["failure_latched"] = payload.get("failure_latched")
                row["gate_status_receive_monotonic"] = gate.get("receive_monotonic")
            if cloud is not None:
                row["scan_counter_observer"] = cloud.get("scan_counter_observer")
            row.update(self._stage_durations(row))
            rows.append(row)
        self._assign_windows(rows)
        return rows

    def _assign_windows(self, rows: list[dict[str, Any]]) -> None:
        active_start = None
        failure_start = None
        shutdown_start = None
        for item in sorted(self._gates, key=lambda value: value["receive_monotonic"]):
            payload = item.get("payload", {})
            stamp = float(item["receive_monotonic"])
            if payload.get("failure_latched") is True and failure_start is None:
                failure_start = stamp
            if payload.get("emergency_stop") is False and payload.get("failure_latched") is False:
                active_start = stamp if active_start is None else active_start
            elif active_start is not None and stamp >= active_start and shutdown_start is None:
                if payload.get("failure_latched") is True or payload.get("emergency_stop") is True:
                    shutdown_start = stamp
        for row in rows:
            stamp = row.get("T11") or row.get("T9") or row.get("T1")
            if not self._valid_timestamp(stamp) or active_start is None or float(stamp) < active_start:
                row["window"] = "STARTUP"
            elif failure_start is not None and float(stamp) >= failure_start:
                row["window"] = "SHUTDOWN"
            elif shutdown_start is not None and float(stamp) >= shutdown_start:
                row["window"] = "SHUTDOWN"
            else:
                row["window"] = "ACTIVE"

    @staticmethod
    def _stage_durations(row: dict[str, Any]) -> dict[str, float | None]:
        values = {label: row.get(field) for label, field in PIPELINE}
        pairs = (
            ("T0_to_T1", "T0", "T1"), ("T1_to_T2", "T1", "T2"),
            ("T2_to_T3", "T2", "T3"), ("T3_to_T4", "T3", "T4"),
            ("T4_to_T5", "T4", "T5"), ("T5_to_T6", "T5", "T6"),
            ("T5_to_T7", "T5", "T7"), ("T6_to_T7", "T6", "T7"),
            ("T7_to_T8", "T7", "T8"), ("T8_to_T9", "T8", "T9"),
            ("T9_to_T10", "T9", "T10"), ("T9_to_T11", "T9", "T11"),
            ("T10_to_T11", "T10", "T11"), ("T11_to_T12", "T11", "T12"),
            ("T12_to_T13", "T12", "T13"), ("T13_to_T14", "T13", "T14"),
            ("T14_to_T15", "T14", "T15"),
        )
        result: dict[str, float | None] = {}
        for name, left, right in pairs:
            if RuntimeAttributionCollector._valid_timestamp(values[left]) and RuntimeAttributionCollector._valid_timestamp(values[right]):
                result[name] = float(values[right]) - float(values[left])
            else:
                result[name] = None
        return result

    @staticmethod
    def _first_boundary(previous: dict[str, Any], current: dict[str, Any]) -> tuple[str, str]:
        previous_t1 = previous.get("T1")
        previous_t2 = previous.get("T2")
        current_t1 = current.get("T1")
        current_t2 = current.get("T2")
        current_t3 = current.get("T3")
        # A new callback receipt with an unchanged T2 means the callback has
        # begun but its latest-state update was not complete by request
        # creation. This is the first observable B boundary, even though the
        # later T2->T3 interval is the numeric interval that exceeds 250 ms.
        if (
            RuntimeAttributionCollector._valid_timestamp(previous_t1)
            and RuntimeAttributionCollector._valid_timestamp(previous_t2)
            and RuntimeAttributionCollector._valid_timestamp(current_t1)
            and RuntimeAttributionCollector._valid_timestamp(current_t2)
            and RuntimeAttributionCollector._valid_timestamp(current_t3)
            and float(current_t1) > float(previous_t1)
            and float(current_t2) <= float(previous_t2)
            and float(current_t1) > float(current_t2)
            and float(current_t3) - float(current_t2) > GAP_THRESHOLD_SEC
        ):
            return "B", "ROS CALLBACK / EXECUTOR STARVATION"
        for label, field in PIPELINE:
            left, right = previous.get(field), current.get(field)
            if RuntimeAttributionCollector._valid_timestamp(left) and RuntimeAttributionCollector._valid_timestamp(right):
                if float(right) - float(left) > GAP_THRESHOLD_SEC:
                    categories = {
                        "T0": ("A", "SENSOR SOURCE GAP"),
                        "T1": ("B", "ROS CALLBACK / EXECUTOR STARVATION"),
                        "T2": ("C", "REQUEST DISPATCH GAP"),
                        "T3": ("C", "REQUEST DISPATCH GAP"),
                        "T4": ("D", "WORKER IPC / SCHEDULING GAP"),
                        "T5": ("D", "WORKER IPC / SCHEDULING GAP"),
                        "T6": ("D", "WORKER IPC / SCHEDULING GAP"),
                        "T7": ("F", "RESPONSE HANDLING GAP"),
                        "T8": ("F", "RESPONSE HANDLING GAP"),
                        "T9": ("G", "RESULT PUBLICATION GAP"),
                        "T10": ("G", "RESULT PUBLICATION GAP"),
                        "T11": ("D", "RELAY RECEIVE GAP"),
                        "T12": ("D", "RELAY PUBLICATION GAP"),
                        "T13": ("F", "SAFETY GATE RECEIVE GAP"),
                        "T14": ("F", "SAFETY GATE OUTPUT GAP"),
                        "T15": ("J", "UNKNOWN"),
                    }
                    return categories.get(label, ("J", "UNKNOWN"))
        return "J", "UNKNOWN"

    @staticmethod
    def _boundary_evidence(previous: dict[str, Any], current: dict[str, Any]) -> dict[str, Any]:
        values = {
            "previous_t1": previous.get("T1"),
            "previous_t2": previous.get("T2"),
            "current_t1": current.get("T1"),
            "current_t2": current.get("T2"),
            "current_t3": current.get("T3"),
        }
        if (
            all(RuntimeAttributionCollector._valid_timestamp(value) for value in values.values())
            and float(values["current_t1"]) > float(values["previous_t1"])
            and float(values["current_t2"]) <= float(values["previous_t2"])
            and float(values["current_t1"]) > float(values["current_t2"])
            and float(values["current_t3"]) - float(values["current_t2"]) > GAP_THRESHOLD_SEC
        ):
            return {
                "observed_boundary": "T1->T2",
                "reason": "new scan callback receipt advanced while latest state-update timestamp did not advance before request creation",
                "state_update_nonadvance": True,
                "callback_receipt_to_request_sec": float(values["current_t3"]) - float(values["current_t1"]),
                "current_callback_to_state_update_sec": float(values["current_t2"]) - float(values["current_t1"]),
                "request_from_previous_state_update_sec": float(values["current_t3"]) - float(values["current_t2"]),
                **values,
            }
        return {}

    def _gap_events(self, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        ordered = sorted(rows, key=lambda item: item.get("cycle_id", -1))
        for index in range(1, len(ordered)):
            previous, current = ordered[index - 1], ordered[index]
            left, right = previous.get("T11"), current.get("T11")
            if self._valid_timestamp(left) and self._valid_timestamp(right):
                gap = float(right) - float(left)
                if gap > GAP_THRESHOLD_SEC:
                    letter, boundary = self._first_boundary(previous, current)
                    events.append(self._bundle(
                        len(events) + 1, "planner_result_interval", gap,
                        previous, current, index, ordered, letter, boundary,
                    ))
        last_source = None
        source_episode_event_emitted = False
        for gate in sorted(self._gates, key=lambda item: item["receive_monotonic"]):
            payload = gate.get("payload", {})
            ages = payload.get("ages_sec") or {}
            age = ages.get("planner_result")
            if not self._valid_timestamp(age) or float(age) <= GAP_THRESHOLD_SEC:
                continue
            source_metadata = (payload.get("freshness") or {}).get("source_metadata") or {}
            source_stamp = source_metadata.get("planner_result_monotonic")
            if not self._valid_timestamp(source_stamp):
                continue
            source_stamp = float(source_stamp)
            if last_source is None or source_stamp != last_source:
                last_source = source_stamp
                source_episode_event_emitted = False
            gap = float(gate["receive_monotonic"]) - last_source
            if gap <= GAP_THRESHOLD_SEC or source_episode_event_emitted:
                continue
            # A planner-result interval already represents this stale-source
            # episode. Do not emit one extra event for every 50 Hz watchdog
            # sample until the source advances or the run ends.
            covered = any(
                event.get("trigger") == "planner_result_interval"
                and abs(float(event.get("start_monotonic", -1.0)) - last_source) <= 1.0e-3
                and float(event.get("start_monotonic", -1.0)) <= float(gate["receive_monotonic"]) <= float(event.get("end_monotonic", -1.0))
                for event in events
            )
            if covered:
                source_episode_event_emitted = True
                continue
            matching = [row for row in rows if self._valid_timestamp(row.get("T11"))]
            anchor = min(
                matching,
                key=lambda row: abs(float(row["T11"]) - last_source),
                default=(rows[-1] if rows else {}),
            )
            anchor_index = ordered.index(anchor) if anchor in ordered else max(0, len(ordered) - 1)
            events.append(self._bundle(
                len(events) + 1, "planner_source_watchdog", gap,
                anchor, anchor, anchor_index, ordered,
                "J", "UNKNOWN: source stopped before a second published result",
                start_monotonic=last_source,
                end_monotonic=float(gate["receive_monotonic"]),
                watchdog_payload=payload,
            ))
            source_episode_event_emitted = True
        return events

    def _bundle(
        self, event_id: int, trigger: str, gap: float,
        previous: dict[str, Any], current: dict[str, Any], index: int,
        ordered: list[dict[str, Any]], letter: str, boundary: str,
        start_monotonic: float | None = None,
        end_monotonic: float | None = None,
        watchdog_payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        start = float(start_monotonic if start_monotonic is not None else previous.get("T11") or previous.get("T9") or previous.get("T1") or 0.0)
        end = float(end_monotonic if end_monotonic is not None else current.get("T11") or current.get("T9") or 0.0)
        lo = max(0, index - 20)
        hi = min(len(ordered), index + 21)
        related_resources = [
            item for item in self._resources
            if start - 0.5 <= float(item["monotonic"]) <= end + 0.5
        ]
        related_gates = [
            item for item in self._gates
            if start - 0.5 <= float(item["receive_monotonic"]) <= end + 0.5
        ]
        return {
            "event_id": event_id,
            "trigger": trigger,
            "threshold_sec": GAP_THRESHOLD_SEC,
            "start_monotonic": start,
            "end_monotonic": end,
            "gap_sec": gap,
            "first_abnormal_boundary": {"code": letter, "label": boundary},
            "boundary_evidence": self._boundary_evidence(previous, current),
            "previous_cycle_id": previous.get("cycle_id"),
            "current_cycle_id": current.get("cycle_id"),
            "start_window": previous.get("window"),
            "end_window": current.get("window"),
            "window": previous.get("window") or current.get("window"),
            "watchdog": watchdog_payload,
            "timeline_window_cycles": ordered[lo:hi],
            "resource_samples": related_resources,
            "gate_observations": related_gates,
        }

    def _rate_summary(self, values: list[dict[str, Any]], timestamp_key: str) -> dict[str, Any]:
        times = [float(item[timestamp_key]) for item in values if self._valid_timestamp(item.get(timestamp_key))]
        intervals = [right - left for left, right in zip(times, times[1:]) if right >= left]
        return {"count": len(times), "interval_sec": _stats(intervals), "rate_hz": (1.0 / statistics.fmean(intervals) if intervals and statistics.fmean(intervals) > 0 else None)}

    def _write_gap_bundles(self, events: list[dict[str, Any]]) -> list[str]:
        directory = self._output.parent / (self._output.stem + "_gap_events")
        directory.mkdir(parents=True, exist_ok=True)
        paths: list[str] = []
        for event in events:
            path = directory / f"gap_{int(event['event_id']):04d}.json"
            path.write_text(json.dumps(_clean(event), indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
            paths.append(str(path.resolve()))
        return paths

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        timeline = self._timeline()
        events = self._gap_events(timeline)
        bundle_paths = self._write_gap_bundles(events)
        delays = {name: [value for value in (self._stage_durations(row).get(name) for row in timeline) if value is not None] for name in self._stage_durations(timeline[0]).keys()} if timeline else {}
        output = {
            "schema": "r1_n2_4c_runtime_attribution_v1",
            "run_label": self._run_label,
            "monotonic_clock": "CLOCK_MONOTONIC via time.monotonic(); all causal timestamps use this domain",
            "t0_boundary": "earliest observable /lidar/points receipt in audit process; Gazebo production timestamp unavailable",
            "frozen_thresholds": {
                "gap_event_threshold_sec": GAP_THRESHOLD_SEC,
                "planner_freshness_timeout_sec": SAFETY_TIMEOUT_SEC,
                "relay_timeout_sec": SAFETY_TIMEOUT_SEC,
                "timing_timeout_sec": 0.50,
            },
            "timeline_definition": [
                {"label": label, "timestamp_field": field} for label, field in PIPELINE
            ],
            "instrumentation": self._overhead,
            "counts": {
                "cloud_observations": len(self._clouds),
                "scan_observations": len(self._scans),
                "timing_records": len(self._timings),
                "source_observations": len(self._sources),
                "relay_observations": len(self._relays),
                "gate_observations": len(self._gates),
                "bridge_observations": len(self._bridges),
                "command_observations": len(self._commands),
                "resource_samples": len(self._resources),
            },
            "rates": {
                "lidar_points_receive": self._rate_summary(self._clouds, "receive_monotonic"),
                "scan_neupan_receive": self._rate_summary(self._scans, "receive_monotonic"),
                "planner_timing_receive": self._rate_summary(self._timings, "audit_receive_monotonic"),
                "planner_source_receive": self._rate_summary(self._sources, "receive_monotonic"),
                "relay_receive": self._rate_summary(self._relays, "receive_monotonic"),
            },
            "window_boundaries": {
                "first_gate_release_observed_monotonic": next((item["receive_monotonic"] for item in self._gates if item.get("payload", {}).get("emergency_stop") is False), None),
                "first_failure_latch_observed_monotonic": next((item["receive_monotonic"] for item in self._gates if item.get("payload", {}).get("failure_latched") is True), None),
            },
            "stage_delay_stats_sec": {key: _stats(value) for key, value in delays.items()},
            "gap_events": [{key: value for key, value in event.items() if key not in {"timeline_window_cycles", "resource_samples", "gate_observations"}} for event in events],
            "gap_event_bundle_paths": bundle_paths,
            "timeline": timeline,
            "resource_summary": {
                "sample_duration_ms": _stats([float(item.get("sample_duration_ms", 0.0)) for item in self._resources]),
                "samples": self._resources,
            },
            "raw_observations": {
                "clouds": self._clouds,
                "scans": self._scans,
                "sources": self._sources,
                "relays": self._relays,
                "gates": self._gates,
                "bridges": self._bridges,
                "commands": self._commands,
            },
        }
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._output.write_text(json.dumps(_clean(output), indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
        self._node.get_logger().info(f"Wrote N2.4C-R runtime attribution to {self._output}")


class RuntimeAttribution(Node):
    """Standalone wrapper retained for launch/debug use."""

    def __init__(self) -> None:
        super().__init__("r1_n2_4c_runtime_attribution")
        for name, default in (
            ("output_file", "n2_4c_runtime/run.json"),
            ("run_label", "unknown"),
        ):
            self.declare_parameter(name, default)
        self.collector = RuntimeAttributionCollector(
            self,
            str(self.get_parameter("output_file").value),
            str(self.get_parameter("run_label").value),
        )

    def destroy_node(self):
        self.collector.finalize()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuntimeAttribution()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
