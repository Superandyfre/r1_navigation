"""Production-grade single-flight bridge validation for N1.16.

The worker entry point is shared with N1.15, but this client treats every
response as an authenticated freshness envelope.  Any protocol or worker
failure quarantines the worker and leaves the ROS side without a planner
fallback.  Recovery/restart is intentionally outside this phase.
"""

from __future__ import annotations

import json
import os
import pickle
import time
from pathlib import Path
from typing import Any

import numpy as np

from r1_neupan_integration.n1_15_whole_core import (
    _json_safe,
    _main_inventory,
    _pack_array,
    _stats,
    _unpack_array,
    _worker_main,
)
from r1_neupan_integration.n2_pre_semantics import (
    DEFAULT_GEAR,
    navigation_loop,
    validate_gear,
)


N116_SCHEMA = 4.0
DEFAULT_TIMEOUT_SEC = 0.25
REQUIRED_OUTPUTS = {"action", "ref_state", "opt_state"}


class BridgeProtocolError(RuntimeError):
    """A bounded, fail-safe bridge protocol error."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class ProductionWholeCoreClient:
    """One long-lived spawn worker with no automatic restart."""

    def __init__(
        self,
        output_dir: str,
        config_path: str,
        checkpoint_path: str,
        pan: dict[str, Any],
        receding: int | None,
        torch_intraop: int = 1,
        torch_interop: int = 24,
        warmup_count: int = 30,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        exact_capture_dir: str = "",
        exact_capture_max_records: int = 512,
        capture_metadata: dict[str, Any] | None = None,
        fault_mode: str = "",
        fault_request_id: int = -1,
        fault_sleep_sec: float = 1.0,
    ) -> None:
        import multiprocessing as mp

        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.config_path = str(config_path)
        self.checkpoint_path = str(checkpoint_path)
        self.pan = dict(pan)
        self.receding = receding
        self.torch_intraop = int(torch_intraop)
        self.torch_interop = int(torch_interop)
        self.warmup_count = int(warmup_count)
        self.timeout_sec = float(timeout_sec)
        self.exact_capture_dir = str(exact_capture_dir)
        self.exact_capture_max_records = max(1, int(exact_capture_max_records))
        self.capture_metadata = dict(capture_metadata or {})
        self.fault_mode = str(fault_mode)
        self.fault_request_id = int(fault_request_id)
        self.fault_sleep_sec = float(fault_sleep_sec)
        self.context = mp.get_context("spawn")
        self.parent_conn = None
        self.process = None
        self.ready: dict[str, Any] = {}
        self.records: list[dict[str, Any]] = []
        self.samples: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.status_counts: dict[str, int] = {}
        self.request_id = 0
        self.requests_sent = 0
        self.requests_completed = 0
        self.worker_failures = 0
        self.queue_depth_max = 0
        self.stale_responses = 0
        self.nan_inf = 0
        self.protocol_errors = 0
        self.scan_accounting = {
            "scans_received": 0,
            "unique_processed": 0,
            "obsolete_dropped": 0,
            "duplicate": 0,
        }
        self.main_inventory_before = _main_inventory()
        self.worker_generation = (os.getpid() << 32) ^ time.time_ns()
        self._last_worker_sequence = -1
        self._closed = False
        self._quarantined = False
        self._finalized = False
        self.shutdown = {
            "requested": False,
            "shutdown_message_sent": False,
            "worker_exited_cleanly": False,
            "pipe_closed": False,
        }

    def _event(self, code: str, message: str, **fields: Any) -> None:
        self.status_counts[code] = self.status_counts.get(code, 0) + 1
        event = {
            "monotonic": time.monotonic(),
            "code": code,
            "status": "fail-safe",
            "message": str(message),
            **fields,
        }
        self.events.append(event)
        with (self.output_dir / "bridge_events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(_json_safe(event), sort_keys=True) + "\n")

    def _quarantine_worker(self, reason: str) -> None:
        """Make a failed pipe unusable and prevent late responses being read."""
        self._quarantined = True
        process = self.process
        try:
            if process is not None and process.is_alive():
                process.terminate()
                process.join(timeout=2.0)
        finally:
            if process is not None and process.is_alive():
                process.kill()
                process.join(timeout=1.0)
            if self.parent_conn is not None:
                try:
                    self.parent_conn.close()
                except Exception:
                    pass
        self._event("worker_quarantined", reason)

    def _start(self, initial_path: np.ndarray, state: np.ndarray, points: np.ndarray,
               path_version: int, navigation_loop_mode: str = "cyclic",
               active_gear: int = DEFAULT_GEAR) -> None:
        self.parent_conn, child_conn = self.context.Pipe(duplex=True)
        spec = {
            "config_path": self.config_path,
            "checkpoint_path": self.checkpoint_path,
            "pan": _json_safe(self.pan),
            "receding": self.receding,
            "torch_intraop": self.torch_intraop,
            "torch_interop": self.torch_interop,
            "warmup_count": self.warmup_count,
            "initial_path": _pack_array(initial_path),
            "initial_path_version": int(path_version),
            "navigation_loop": navigation_loop(navigation_loop_mode),
            "active_gear": validate_gear(active_gear),
            "warmup_state": _pack_array(state),
            "warmup_points": _pack_array(points),
            "worker_generation": int(self.worker_generation),
            "fault_mode": self.fault_mode,
            "fault_request_id": self.fault_request_id,
            "fault_sleep_sec": self.fault_sleep_sec,
            "exact_capture_dir": self.exact_capture_dir,
            "exact_capture_max_records": self.exact_capture_max_records,
            "capture_metadata": _json_safe(self.capture_metadata),
        }
        self.process = self.context.Process(
            target=_worker_main,
            args=(child_conn, spec),
            name="r1_n1_16_whole_core",
            daemon=True,
        )
        started = time.perf_counter_ns()
        try:
            self.process.start()
            child_conn.close()
            ready_timeout = max(15.0, self.timeout_sec * 6.0)
            if not self.parent_conn.poll(ready_timeout):
                raise BridgeProtocolError("worker_startup_timeout", "worker did not send READY")
            try:
                self.ready = pickle.loads(self.parent_conn.recv_bytes())
            except Exception as exc:
                raise BridgeProtocolError("malformed_ready", f"READY decode failed: {exc}") from exc
            self.startup_ms = (time.perf_counter_ns() - started) / 1_000_000.0
            if self.ready.get("status") != "READY":
                raise BridgeProtocolError("worker_startup_failure", repr(self.ready))
            if int(self.ready.get("worker_generation", -1)) != int(self.worker_generation):
                raise BridgeProtocolError("generation_mismatch", "READY generation mismatch")
        except Exception as exc:
            code = exc.code if isinstance(exc, BridgeProtocolError) else "worker_startup_failure"
            self.worker_failures += 1
            self._event(code, str(exc))
            self._quarantine_worker(code)
            raise

    def startup(self, initial_path: np.ndarray, state: np.ndarray,
                points: np.ndarray, path_version: int,
                navigation_loop_mode: str = "cyclic",
                active_gear: int = DEFAULT_GEAR) -> None:
        if self.process is None:
            self._start(
                initial_path, state, points, path_version,
                navigation_loop_mode, active_gear,
            )

    def _fail(self, code: str, message: str, **fields: Any) -> None:
        self.worker_failures += 1
        if code == "nan_inf":
            self.nan_inf += 1
        if code in {"malformed_response", "request_id_mismatch", "generation_mismatch",
                    "cycle_id_mismatch", "scan_counter_mismatch", "path_version_mismatch",
                    "goal_version_mismatch", "stale_response"}:
            self.protocol_errors += 1
        if code == "stale_response":
            self.stale_responses += 1
        self._event(code, message, **fields)
        self._quarantine_worker(code)
        raise BridgeProtocolError(code, message)

    def solve(
        self,
        state: np.ndarray,
        points: np.ndarray,
        path: np.ndarray,
        path_version: int,
        scan_counter: int,
        scan_stamp_sec: float,
        goal: np.ndarray | None = None,
        goal_version: int = -1,
        cycle_id: int = -1,
        navigation_loop_mode: str = "cyclic",
        active_gear: int = DEFAULT_GEAR,
        restore_planner_state: dict[str, Any] | None = None,
        scan_callback_receive_monotonic: float = -1.0,
        scan_update_monotonic: float = -1.0,
        reference_speed_profile: np.ndarray | None = None,
        measured_velocity: np.ndarray | None = None,
        local_reference_path: np.ndarray | None = None,
        local_reference_id: int = 0,
        bypass_mode: bool = False,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        if self._closed:
            self._fail("bridge_closed", "bridge is closed")
        if self._quarantined or self.process is None or not self.process.is_alive():
            self._fail("worker_crash", "worker is not alive; no fallback is permitted")
        self.request_id += 1
        request_id = self.request_id
        request = {
            "type": "solve",
            "request_id": request_id,
            "cycle_id": int(cycle_id),
            "scan_counter": int(scan_counter),
            "scan_stamp_sec": float(scan_stamp_sec),
            "path_version": int(path_version),
            "goal_version": int(goal_version),
            "state": _pack_array(state),
            "obstacle_points": _pack_array(points),
            "path": _pack_array(path),
            "goal": _pack_array(goal),
            "navigation_loop": navigation_loop(navigation_loop_mode),
            "active_gear": validate_gear(active_gear),
            "restore_planner_state": restore_planner_state,
            "bypass_mode": bool(bypass_mode),
        }
        if reference_speed_profile is not None:
            request["reference_speed_profile"] = _pack_array(reference_speed_profile)
        if measured_velocity is not None:
            request["measured_velocity"] = _pack_array(measured_velocity)
        if local_reference_path is not None:
            request["local_reference_path"] = _pack_array(local_reference_path)
            request["local_reference_id"] = int(local_reference_id)
        request["request_created_monotonic"] = time.monotonic()
        prep_started = time.perf_counter_ns()
        request["main_send_started_monotonic"] = time.monotonic()
        try:
            raw_request = pickle.dumps(request, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as exc:
            self._fail("request_serialization_failure", str(exc), request_id=request_id)
        prep_finished = time.perf_counter_ns()
        send_started = time.perf_counter_ns()
        transport_send_started = time.monotonic()
        try:
            self.parent_conn.send_bytes(raw_request)
        except (EOFError, OSError, BrokenPipeError) as exc:
            if self.process is not None:
                self.process.join(timeout=0.2)
            code = "worker_crash" if (
                self.process is None
                or not self.process.is_alive()
                or self.process.exitcode is not None
            ) else "pipe_broken"
            self._fail(code, str(exc), request_id=request_id)
        send_finished = time.perf_counter_ns()
        transport_send_finished = time.monotonic()
        self.requests_sent += 1
        self.queue_depth_max = max(self.queue_depth_max, 1)
        transport_wait_started = transport_send_finished
        if not self.parent_conn.poll(self.timeout_sec):
            code = "worker_crash" if self.process is None or not self.process.is_alive() else "worker_timeout"
            self._fail(code, f"no response within {self.timeout_sec:.3f}s", request_id=request_id)
        transport_response_available = time.monotonic()
        receive_started = time.perf_counter_ns()
        try:
            raw_response = self.parent_conn.recv_bytes()
            transport_response_received = time.monotonic()
            response = pickle.loads(raw_response)
        except (EOFError, OSError, BrokenPipeError) as exc:
            if self.process is not None:
                self.process.join(timeout=0.2)
            code = "worker_crash" if (
                self.process is None
                or not self.process.is_alive()
                or self.process.exitcode is not None
            ) else "pipe_broken"
            self._fail(code, str(exc), request_id=request_id)
        except Exception as exc:
            self._fail("malformed_response", str(exc), request_id=request_id)
        receive_finished = time.perf_counter_ns()
        transport_response_decoded = time.monotonic()
        if not isinstance(response, dict) or response.get("type") != "response":
            self._fail("malformed_response", "response envelope is not a response dict", request_id=request_id)

        checks = (
            ("worker_generation", self.worker_generation, "generation_mismatch"),
            ("request_id", request_id, "request_id_mismatch"),
            ("cycle_id", int(cycle_id), "cycle_id_mismatch"),
            ("scan_counter", int(scan_counter), "scan_counter_mismatch"),
            ("path_version", int(path_version), "path_version_mismatch"),
            ("goal_version", int(goal_version), "goal_version_mismatch"),
        )
        for field, expected, code in checks:
            actual = response.get(field)
            if actual != expected:
                stale = field in {"cycle_id", "scan_counter"} and isinstance(actual, int) and actual < expected
                actual_code = "stale_response" if stale else code
                self._fail(actual_code, f"{field} expected {expected}, got {actual}",
                           request_id=request_id, expected=expected, actual=actual)
        if response.get("status") != "return-success":
            self._fail("worker_response_failure", str(response.get("exception")), request_id=request_id)
        worker_sequence = int(response.get("worker_sequence", -1))
        if worker_sequence <= self._last_worker_sequence:
            self._fail("stale_response", "worker sequence did not advance", request_id=request_id)
        self._last_worker_sequence = worker_sequence
        if not response.get("finite"):
            self._fail("nan_inf", "worker marked response non-finite", request_id=request_id)
        packed = response.get("outputs")
        if not isinstance(packed, dict) or not REQUIRED_OUTPUTS.issubset(packed):
            self._fail("malformed_response", "required numeric outputs are missing", request_id=request_id)
        try:
            outputs = {key: _unpack_array(value) for key, value in packed.items()}
        except Exception as exc:
            self._fail("malformed_response", f"numeric payload decode failed: {exc}", request_id=request_id)
        if not all(np.isfinite(value).all() for value in outputs.values()):
            self._fail("nan_inf", "decoded response contains NaN/Inf", request_id=request_id)
        reconstruction_finished = time.perf_counter_ns()
        worker_timings = dict(response.get("timings", {}))
        timings = {
            **worker_timings,
            "main_request_prep_ms": (prep_finished - prep_started) / 1_000_000.0,
            "main_request_serialization_ms": 0.0,
            "main_request_send_ms": (send_finished - send_started) / 1_000_000.0,
            "main_wait_ms": (receive_started - send_finished) / 1_000_000.0,
            "main_response_receive_ms": (receive_finished - receive_started) / 1_000_000.0,
            "main_reconstruction_ms": (reconstruction_finished - receive_finished) / 1_000_000.0,
            "main_ipc_roundtrip_ms": (receive_finished - send_started) / 1_000_000.0,
            "main_full_call_ms": (reconstruction_finished - prep_started) / 1_000_000.0,
            "transport_request_send_started_monotonic": transport_send_started,
            "transport_request_send_finished_monotonic": transport_send_finished,
            "transport_wait_started_monotonic": transport_wait_started,
            "transport_response_available_monotonic": transport_response_available,
            "transport_response_received_monotonic": transport_response_received,
            "transport_response_decoded_monotonic": transport_response_decoded,
            "scan_callback_receive_monotonic": float(scan_callback_receive_monotonic),
            "scan_update_monotonic": float(scan_update_monotonic),
            "request_created_monotonic": float(request["request_created_monotonic"]),
            "request_dispatched_monotonic": float(transport_send_finished),
            "main_response_received_monotonic": float(transport_response_received),
        }
        semantic = dict(response.get("semantic", {}))
        if isinstance(semantic.get("initial_path"), dict):
            semantic["initial_path"] = _unpack_array(semantic["initial_path"])
        record = {
            "schema": N116_SCHEMA,
            "request_id": request_id,
            "cycle_id": int(cycle_id),
            "scan_counter": int(scan_counter),
            "scan_stamp_sec": float(scan_stamp_sec),
            "path_version": int(path_version),
            "goal_version": int(goal_version),
            "worker_generation": int(response["worker_generation"]),
            "worker_pid": response.get("worker_pid"),
            "worker_sequence": worker_sequence,
            "status": response.get("status"),
            "finite": True,
            "info": response.get("info", {}),
            "semantic": semantic,
            "timings": timings,
            "outputs": {key: {"shape": list(value.shape), "dtype": str(value.dtype)}
                        for key, value in outputs.items()},
        }
        self.records.append(record)
        self.requests_completed += 1
        if len(self.samples) < 3:
            self.samples.append({"request": record, "outputs": outputs})
        return outputs, record

    def set_scan_accounting(self, **values: int) -> None:
        self.scan_accounting.update({key: int(value) for key, value in values.items()})

    def dump_exact_capture(self, metadata: dict[str, Any] | None = None) -> dict[str, Any]:
        """Flush the worker capture without changing planner state or inputs."""
        if self.process is None or not self.process.is_alive() or self.parent_conn is None:
            return {"status": "UNAVAILABLE"}
        try:
            self.parent_conn.send_bytes(pickle.dumps({
                "type": "dump_exact_capture",
                "metadata": dict(metadata or {}),
            }, protocol=pickle.HIGHEST_PROTOCOL))
            if not self.parent_conn.poll(self.timeout_sec):
                return {"status": "TIMEOUT"}
            return pickle.loads(self.parent_conn.recv_bytes())
        except (EOFError, OSError, BrokenPipeError) as exc:
            return {"status": "ERROR", "error": str(exc)}

    def mark_stale_response(self, reason: str, record: dict[str, Any]) -> None:
        """Record a valid worker result rejected by the ROS freshness gate."""
        self.stale_responses += 1
        self.protocol_errors += 1
        self._event(
            "stale_response", reason,
            request_id=record.get("request_id"),
            cycle_id=record.get("cycle_id"),
            scan_counter=record.get("scan_counter"),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.shutdown["requested"] = True
        process = self.process
        if process is not None and process.is_alive() and self.parent_conn is not None and not self._quarantined:
            try:
                self.parent_conn.send_bytes(pickle.dumps({
                    "type": "shutdown",
                    "metadata": self.capture_metadata,
                }))
                self.shutdown["shutdown_message_sent"] = True
                process.join(timeout=20.0 if self.exact_capture_dir else 2.0)
            except (EOFError, OSError, BrokenPipeError):
                pass
        try:
            if process is not None and process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
            self.shutdown["worker_exited_cleanly"] = (
                process is None or (not process.is_alive() and process.exitcode == 0)
            )
        finally:
            if self.parent_conn is not None:
                try:
                    self.parent_conn.close()
                    self.shutdown["pipe_closed"] = True
                except Exception:
                    pass

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self.close()
        values: dict[str, list[float]] = {}
        for record in self.records:
            for key, value in record.get("timings", {}).items():
                if isinstance(value, (int, float)) and key.endswith("_ms"):
                    values.setdefault(key, []).append(float(value))
        summary = {
            "schema": "r1_n1_16_production_bridge_v1",
            "start_method": "spawn",
            "single_flight": True,
            "queue_depth_contract": "<=1",
            "nested_solver_process": False,
            "automatic_restart": False,
            "worker_generation": int(self.worker_generation),
            "startup_ms": getattr(self, "startup_ms", None),
            "ready": self.ready,
            "main_inventory_before": self.main_inventory_before,
            "main_inventory_after": _main_inventory(),
            "requests_sent": self.requests_sent,
            "requests_completed": self.requests_completed,
            "worker_failures": self.worker_failures,
            "queue_depth_max": self.queue_depth_max,
            "scan_accounting": self.scan_accounting,
            "failure_counts": self.status_counts,
            "stale_responses": self.stale_responses,
            "nan_inf": self.nan_inf,
            "protocol_errors": self.protocol_errors,
            "fault_injection": {
                "mode": self.fault_mode,
                "request_id": self.fault_request_id,
                "sleep_sec": self.fault_sleep_sec,
            },
            "shutdown": self.shutdown,
            "timing": {key: _stats(value) for key, value in sorted(values.items())},
            "records": self.records,
            "events": self.events,
            "exact_capture": {
                "directory": self.exact_capture_dir or None,
                "max_records": self.exact_capture_max_records,
                "metadata": self.capture_metadata,
            },
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(_json_safe(summary), indent=2, sort_keys=True), encoding="utf-8"
        )
        with (self.output_dir / "requests.jsonl").open("w", encoding="utf-8") as stream:
            for record in self.records:
                stream.write(json.dumps(_json_safe(record), sort_keys=True) + "\n")
