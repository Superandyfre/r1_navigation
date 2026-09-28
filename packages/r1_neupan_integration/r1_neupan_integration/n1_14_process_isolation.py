"""N1.14 dedicated-process boundary for the live NRMP CvxpyLayer.

Only numerical parameter values cross the process boundary.  The worker owns
its own NeuPAN-derived CvxpyLayer and is started once with ``spawn``.  The
module is deliberately independent of ROS so it can also run the offline
numerical-equivalence check against an N1.11 parameter snapshot.
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import pickle
import platform
import resource
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


GRAD_INDICES = {1, 2, 33, 34, 35, 36, 37}
DEFAULT_TIMEOUT_SEC = 30.0


def _env_snapshot() -> dict[str, str | None]:
    return {
        name: os.environ.get(name)
        for name in (
            "OMP_NUM_THREADS",
            "MKL_NUM_THREADS",
            "OPENBLAS_NUM_THREADS",
            "NUMEXPR_NUM_THREADS",
        )
    }


def _rss_kb() -> int:
    try:
        return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    except (AttributeError, ValueError):
        return -1


def runtime_inventory() -> dict[str, Any]:
    import torch
    import threadpoolctl

    try:
        load_average = list(os.getloadavg())
    except OSError:
        load_average = None
    return {
        "pid": os.getpid(),
        "python_executable": sys.executable,
        "python_version": sys.version,
        "platform": platform.platform(),
        "torch_version": str(torch.__version__),
        "torch_intraop_threads": int(torch.get_num_threads()),
        "torch_interop_threads": int(torch.get_num_interop_threads()),
        "environment": _env_snapshot(),
        "threadpoolctl": threadpoolctl.threadpool_info(),
        "max_rss_kb": _rss_kb(),
        "load_average": load_average,
        "process_time_sec": time.process_time(),
    }


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return repr(value)


def _set_torch_policy(intraop: int, interop: int) -> dict[str, Any]:
    import torch

    errors = []
    if intraop > 0:
        try:
            torch.set_num_threads(intraop)
        except RuntimeError as exc:
            errors.append(f"intraop: {type(exc).__name__}: {exc}")
    if interop > 0:
        try:
            torch.set_num_interop_threads(interop)
        except RuntimeError as exc:
            errors.append(f"interop: {type(exc).__name__}: {exc}")
    return {"errors": errors, "inventory": runtime_inventory()}


def _build_layer(config_path: str, checkpoint_path: str,
                 pan: dict[str, Any], receding: int | None) -> Any:
    from neupan import neupan

    kwargs: dict[str, Any] = {
        "pan": dict(pan),
        "device": "cpu",
    }
    if receding is not None:
        kwargs["receding"] = int(receding)
    kwargs["pan"]["dune_checkpoint"] = checkpoint_path
    planner = neupan.init_from_yaml(config_path, **kwargs)
    return planner.pan.nrmp_layer.nrmp_layer


def _parameter_from_array(array: np.ndarray, requires_grad: bool) -> Any:
    import torch

    # np.ascontiguousarray promotes a zero-dimensional ndarray to shape (1,)
    # on this NumPy version.  Preserve scalar CVXPY parameters exactly.
    value = torch.from_numpy(np.array(array, copy=True, order="C"))
    if requires_grad:
        value.requires_grad_(True)
    return value


def _default_parameters(layer: Any) -> tuple[Any, ...]:
    values = []
    for index, parameter in enumerate(layer.ctx.parameters):
        value = parameter.value
        if value is None:
            value = np.zeros(parameter.shape or (), dtype=np.float32)
        value = np.asarray(value)
        if value.dtype.kind not in "fc":
            value = value.astype(np.float32)
        else:
            value = value.astype(np.float32, copy=False)
        values.append(_parameter_from_array(value, index in GRAD_INDICES))
    return tuple(values)


def _encode_parameters(params: tuple[Any, ...]) -> tuple[list[dict[str, Any]], int, int]:
    records = []
    total_elements = 0
    total_bytes = 0
    for index, value in enumerate(params):
        array = np.array(value.detach().cpu().numpy(), copy=True, order="C")
        raw = array.tobytes(order="C")
        nbytes = len(raw)
        total_elements += int(array.size)
        total_bytes += nbytes
        records.append({
            "index": index,
            "shape": list(array.shape),
            "dtype": str(array.dtype),
            "requires_grad": bool(value.requires_grad),
            "nbytes": nbytes,
            "data": raw,
        })
    return records, total_elements, total_bytes


def _decode_parameters(records: list[dict[str, Any]]) -> tuple[Any, ...]:
    values = []
    for expected_index, record in enumerate(records):
        if int(record["index"]) != expected_index:
            raise ValueError("parameter indices are not contiguous")
        dtype = np.dtype(record["dtype"])
        shape = tuple(int(value) for value in record["shape"])
        raw = record["data"]
        if int(record["nbytes"]) != len(raw):
            raise ValueError("parameter payload byte count mismatch")
        array = np.frombuffer(raw, dtype=dtype).reshape(shape).copy()
        values.append(_parameter_from_array(array, bool(record["requires_grad"])))
    return tuple(values)


def _pack_outputs(outputs: Any) -> list[dict[str, Any]]:
    values = outputs if isinstance(outputs, (tuple, list)) else (outputs,)
    packed = []
    for index, value in enumerate(values):
        if not hasattr(value, "detach"):
            raise TypeError(f"solver output {index} is not a tensor")
        array = np.ascontiguousarray(value.detach().cpu().numpy())
        raw = array.tobytes(order="C")
        packed.append({
            "index": index,
            "shape": list(array.shape),
            "dtype": str(array.dtype),
            "requires_grad": bool(value.requires_grad),
            "nbytes": len(raw),
            "data": raw,
        })
    return packed


def _decode_outputs(records: list[dict[str, Any]]) -> tuple[Any, ...]:
    return tuple(
        _parameter_from_array(
            np.frombuffer(record["data"], dtype=np.dtype(record["dtype"]))
            .reshape(tuple(int(value) for value in record["shape"]))
            .copy(),
            bool(record["requires_grad"]),
        )
        for record in records
    )


def _finite_output_records(records: list[dict[str, Any]]) -> bool:
    for record in records:
        array = np.frombuffer(record["data"], dtype=np.dtype(record["dtype"]))
        if not bool(np.isfinite(array).all()):
            return False
    return True


def _send_response(conn: Any, response: dict[str, Any]) -> None:
    raw = pickle.dumps(response, protocol=pickle.HIGHEST_PROTOCOL)
    conn.send_bytes(raw)


def _worker_main(conn: Any, spec: dict[str, Any]) -> None:
    """Spawn target.  Keep all Torch/NeuPAN imports inside this process."""
    try:
        policy = _set_torch_policy(
            int(spec["torch_intraop"]), int(spec["torch_interop"])
        )
        layer = _build_layer(
            str(spec["config_path"]),
            str(spec["checkpoint_path"]),
            dict(spec["pan"]),
            spec.get("receding"),
        )
        warmup_params = _default_parameters(layer)
        warmup_rows = []
        import torch
        for sequence in range(int(spec["warmup_count"])):
            started_wall = time.perf_counter_ns()
            started_cpu = time.process_time_ns()
            with torch.set_grad_enabled(True):
                outputs = layer(*warmup_params, solver_args={"solve_method": "ECOS"})
            warmup_rows.append({
                "sequence": sequence,
                "wall_ms": (time.perf_counter_ns() - started_wall) / 1_000_000.0,
                "cpu_ms": (time.process_time_ns() - started_cpu) / 1_000_000.0,
                "finite": all(bool(torch.isfinite(value).all().item()) for value in outputs),
            })
            del outputs
        _send_response(conn, {
            "type": "ready",
            "status": "READY",
            "pid": os.getpid(),
            "worker_inventory": runtime_inventory(),
            "torch_policy": policy,
            "warmup_count": len(warmup_rows),
            "warmup": warmup_rows,
            "layer_parameter_count": len(layer.ctx.parameters),
            "layer_output_count": len(layer.ctx.variables),
        })
        worker_sequence = 0
        while True:
            # Exclude idle time between requests from receive/decode.  The
            # explicit request timestamp is reported separately as queue wait.
            conn.poll(None)
            receive_started = time.perf_counter_ns()
            raw_request = conn.recv_bytes()
            receive_finished = time.perf_counter_ns()
            request = pickle.loads(raw_request)
            if request.get("type") == "shutdown":
                return
            request_id = int(request["request_id"])
            decode_started = time.perf_counter_ns()
            params = _decode_parameters(request["parameters"])
            decode_finished = time.perf_counter_ns()
            solve_started_wall = time.perf_counter_ns()
            solve_started_cpu = time.process_time_ns()
            status = "return-success"
            exception = None
            outputs = None
            try:
                with torch.set_grad_enabled(bool(request["grad_enabled"])):
                    outputs = layer(*params, solver_args=dict(request["solver_args"]))
            except Exception as exc:
                status = "exception"
                exception = f"{type(exc).__name__}: {exc}"
            solve_finished_wall = time.perf_counter_ns()
            solve_finished_cpu = time.process_time_ns()
            result_pack_started = time.perf_counter_ns()
            output_records = _pack_outputs(outputs) if outputs is not None else []
            finite = _finite_output_records(output_records) if output_records else None
            result_pack_finished = time.perf_counter_ns()
            response = {
                "type": "response",
                "request_id": request_id,
                "solver_worker_sequence": worker_sequence,
                "status": status,
                "exception": exception,
                "finite": finite,
                "outputs": output_records,
                "worker_pid": os.getpid(),
                "timings": {
                    "worker_receive_ms": (receive_finished - receive_started) / 1_000_000.0,
                    "worker_queue_wait_ms": max(
                        0.0,
                        (receive_started - int(request["main_send_started_perf_ns"]))
                        / 1_000_000.0,
                    ),
                    "worker_decode_ms": (decode_finished - decode_started) / 1_000_000.0,
                    "worker_solve_wall_ms": (solve_finished_wall - solve_started_wall) / 1_000_000.0,
                    "worker_solve_cpu_ms": (solve_finished_cpu - solve_started_cpu) / 1_000_000.0,
                    "worker_result_pack_ms": (result_pack_finished - result_pack_started) / 1_000_000.0,
                },
            }
            response_send_started = time.perf_counter_ns()
            response["response_send_started_perf_ns"] = response_send_started
            response_serialization_started = time.perf_counter_ns()
            raw_response = pickle.dumps(response, protocol=pickle.HIGHEST_PROTOCOL)
            response_serialization_finished = time.perf_counter_ns()
            response["timings"]["worker_result_serialization_ms"] = (
                response_serialization_finished - response_serialization_started
            ) / 1_000_000.0
            # Re-serialize once so the serialization timing is part of the
            # returned record and the measured send contains the final bytes.
            raw_response = pickle.dumps(response, protocol=pickle.HIGHEST_PROTOCOL)
            send_started = time.perf_counter_ns()
            conn.send_bytes(raw_response)
            send_finished = time.perf_counter_ns()
            response["timings"]["worker_response_send_ms"] = (
                send_finished - send_started
            ) / 1_000_000.0
            worker_sequence += 1
            del params, outputs
    except Exception as exc:
        try:
            _send_response(conn, {
                "type": "worker_error",
                "status": "worker-error",
                "worker_pid": os.getpid(),
                "exception": f"{type(exc).__name__}: {exc}",
                "traceback": repr(exc),
            })
        except Exception:
            pass
        raise
    finally:
        conn.close()


def _stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0, "median_ms": None, "p95_ms": None, "p99_ms": None,
            "max_ms": None, "over_20_ms": 0, "over_100_ms": 0,
            "over_500_ms": 0, "over_1000_ms": 0,
        }
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "median_ms": float(np.percentile(array, 50)),
        "p95_ms": float(np.percentile(array, 95)),
        "p99_ms": float(np.percentile(array, 99)),
        "max_ms": float(np.max(array)),
        "over_20_ms": int(np.sum(array > 20.0)),
        "over_100_ms": int(np.sum(array > 100.0)),
        "over_500_ms": int(np.sum(array >= 500.0)),
        "over_1000_ms": int(np.sum(array >= 1000.0)),
    }


class IsolatedSolverClient:
    """Synchronous single-flight client for one long-lived solver process."""

    def __init__(self, output_dir: str, config_path: str, checkpoint_path: str,
                 pan: dict[str, Any], receding: int | None,
                 torch_intraop: int = 1, torch_interop: int = 24,
                 warmup_count: int = 30,
                 timeout_sec: float = DEFAULT_TIMEOUT_SEC) -> None:
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.timeout_sec = float(timeout_sec)
        self.records: list[dict[str, Any]] = []
        self.cycle_annotations: list[dict[str, Any]] = []
        self.samples: list[dict[str, Any]] = []
        self.request_id = 0
        self.cycle_id = -1
        self.scan_counter = -1
        self.scan_stamp_sec = -1.0
        self.requests_sent = 0
        self.requests_completed = 0
        self.dropped_superseded = 0
        self.queue_depth_max = 0
        self.worker_failures = 0
        self._closed = False
        self.main_inventory_before = runtime_inventory()
        self.context = mp.get_context("spawn")
        self.parent_conn, child_conn = self.context.Pipe(duplex=True)
        spec = {
            "config_path": str(config_path),
            "checkpoint_path": str(checkpoint_path),
            "pan": _json_safe(dict(pan)),
            "receding": receding,
            "torch_intraop": int(torch_intraop),
            "torch_interop": int(torch_interop),
            "warmup_count": int(warmup_count),
        }
        self.process = self.context.Process(
            target=_worker_main,
            args=(child_conn, spec),
            name="r1_n1_14_solver",
            daemon=True,
        )
        started = time.perf_counter_ns()
        self.process.start()
        child_conn.close()
        if not self.parent_conn.poll(self.timeout_sec * 6.0):
            self.process.join(timeout=1.0)
            raise RuntimeError("solver worker did not send READY")
        ready = pickle.loads(self.parent_conn.recv_bytes())
        self.startup_ms = (time.perf_counter_ns() - started) / 1_000_000.0
        if ready.get("status") != "READY":
            raise RuntimeError(f"solver worker startup failed: {ready}")
        self.ready = ready

    def begin_cycle(self, cycle_id: int, scan_counter: int, scan_stamp_sec: float) -> None:
        self.cycle_id = int(cycle_id)
        self.scan_counter = int(scan_counter)
        self.scan_stamp_sec = float(scan_stamp_sec)

    def annotate_cycle(self, cycle_id: int, timing: dict[str, Any]) -> None:
        planner_start = float(timing.get("planner_start_monotonic", 0.0))
        planner_end = float(timing.get("planner_end_monotonic", 0.0))
        if planner_end <= planner_start:
            return
        row = {
            "planner_cycle_id": int(cycle_id),
            "scan_counter": int(timing.get("scan_counter", -1.0)),
            "planner_total_ms": (planner_end - planner_start) * 1000.0,
            "cycle_total_ms": (
                float(timing.get("output_publish_monotonic", planner_end))
                - float(timing.get("cycle_start_monotonic", planner_start))
            ) * 1000.0,
            "worker_request_count": sum(
                1 for record in self.records
                if record.get("planner_cycle_id") == int(cycle_id)
            ),
        }
        self.cycle_annotations.append(row)

    def solve(self, params: tuple[Any, ...], solver_args: dict[str, Any] | None) -> tuple[tuple[Any, ...], dict[str, Any]]:
        if self._closed or not self.process.is_alive():
            self.worker_failures += 1
            raise RuntimeError("N1.14 solver worker is not alive; no fallback is permitted")
        self.request_id += 1
        request_id = self.request_id
        self.requests_sent += 1
        self.queue_depth_max = max(self.queue_depth_max, 1)
        extraction_started = time.perf_counter_ns()
        parameter_records, element_count, payload_bytes = _encode_parameters(params)
        extraction_finished = time.perf_counter_ns()
        request = {
            "type": "solve",
            "request_id": request_id,
            "planner_cycle_id": self.cycle_id,
            "scan_counter": self.scan_counter,
            "scan_stamp_sec": self.scan_stamp_sec,
            # The live NeuPAN inference path enters the layer with gradients
            # enabled.  The worker has no original grad_fn to preserve.
            "grad_enabled": True,
            "solver_args": _json_safe(dict(solver_args or {})),
            "parameters": parameter_records,
        }
        send_started = time.perf_counter_ns()
        request["main_send_started_perf_ns"] = send_started
        serialization_started = time.perf_counter_ns()
        raw_request = pickle.dumps(request, protocol=pickle.HIGHEST_PROTOCOL)
        serialization_finished = time.perf_counter_ns()
        self.parent_conn.send_bytes(raw_request)
        send_finished = time.perf_counter_ns()
        if not self.parent_conn.poll(self.timeout_sec):
            self.worker_failures += 1
            raise TimeoutError(
                f"N1.14 solver timeout request_id={request_id} cycle={self.cycle_id}"
            )
        receive_started = time.perf_counter_ns()
        raw_response = self.parent_conn.recv_bytes()
        receive_finished = time.perf_counter_ns()
        response = pickle.loads(raw_response)
        if int(response.get("request_id", -1)) != request_id:
            self.worker_failures += 1
            raise RuntimeError(f"N1.14 response id mismatch: {response}")
        response_status = response.get("status")
        reconstruction_started = time.perf_counter_ns()
        outputs = _decode_outputs(response.get("outputs", []))
        reconstruction_finished = time.perf_counter_ns()
        timings = dict(response.get("timings", {}))
        timings.update({
            "parameter_extraction_ms": (extraction_finished - extraction_started) / 1_000_000.0,
            "request_serialization_ms": (serialization_finished - serialization_started) / 1_000_000.0,
            "request_send_ms": (send_finished - send_started) / 1_000_000.0,
            "main_wait_ms": (receive_finished - send_finished) / 1_000_000.0,
            "response_return_ms": max(
                0.0,
                (receive_started - int(response.get("response_send_started_perf_ns", receive_started)))
                / 1_000_000.0,
            ),
            "response_receive_ms": (receive_finished - receive_started) / 1_000_000.0,
            "main_reconstruction_ms": (reconstruction_finished - reconstruction_started) / 1_000_000.0,
            "ipc_roundtrip_ms": (receive_finished - send_started) / 1_000_000.0,
            "full_call_ms": (reconstruction_finished - extraction_started) / 1_000_000.0,
            "parameter_count": len(parameter_records),
            "parameter_elements": element_count,
            "payload_bytes": payload_bytes,
        })
        record = {
            "request_id": request_id,
            "planner_cycle_id": self.cycle_id,
            "scan_counter": self.scan_counter,
            "scan_stamp_sec": self.scan_stamp_sec,
            "solver_worker_sequence": response.get("solver_worker_sequence"),
            "status": response_status,
            "exception": response.get("exception"),
            "finite": response.get("finite"),
            "worker_pid": response.get("worker_pid"),
            "parameter_count": len(parameter_records),
            "parameter_elements": element_count,
            "payload_bytes": payload_bytes,
            "parameter_dtypes": [row["dtype"] for row in parameter_records],
            "parameter_shapes": [row["shape"] for row in parameter_records],
            "parameter_requires_grad": [row["requires_grad"] for row in parameter_records],
            "timings": timings,
        }
        self.records.append(record)
        self.requests_completed += 1
        sample_number = len(self.samples)
        if sample_number < 5 or request_id % 10 == 0:
            self.samples.append({
                "request_id": request_id,
                "parameters": [value.detach().cpu().numpy().copy() for value in params],
                "outputs": [value.detach().cpu().numpy().copy() for value in outputs],
            })
        if response_status != "return-success":
            self.worker_failures += 1
            raise RuntimeError(
                f"N1.14 solver worker failed request_id={request_id}: {response.get('exception')}"
            )
        return outputs, record

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self.process.is_alive():
                self.parent_conn.send_bytes(pickle.dumps({"type": "shutdown"}))
                self.process.join(timeout=5.0)
        finally:
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2.0)
            self.parent_conn.close()

    def finalize(self) -> None:
        self.close()
        by_metric: dict[str, list[float]] = {}
        for record in self.records:
            for key, value in record.get("timings", {}).items():
                if isinstance(value, (int, float)) and key.endswith("_ms"):
                    by_metric.setdefault(key, []).append(float(value))
        summary = {
            "schema": "r1_n1_14_process_isolation_v2",
            "start_method": "spawn",
            "startup_ms": self.startup_ms,
            "ready": self.ready,
            "main_inventory_before": self.main_inventory_before,
            "main_inventory_after": runtime_inventory(),
            "requests_sent": self.requests_sent,
            "requests_completed": self.requests_completed,
            "dropped_superseded": self.dropped_superseded,
            "queue_depth_max": self.queue_depth_max,
            "worker_failures": self.worker_failures,
            "single_flight": True,
            "metrics": {key: _stats(value) for key, value in sorted(by_metric.items())},
            "planner_total_ms": _stats([
                float(row["planner_total_ms"]) for row in self.cycle_annotations
            ]),
            "cycle_total_ms": _stats([
                float(row["cycle_total_ms"]) for row in self.cycle_annotations
            ]),
            "cycle_annotations": self.cycle_annotations,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(_json_safe(summary), indent=2, sort_keys=True), encoding="utf-8"
        )
        with (self.output_dir / "requests.jsonl").open("w", encoding="utf-8") as stream:
            for record in self.records:
                stream.write(json.dumps(_json_safe(record), sort_keys=True) + "\n")
        (self.output_dir / "cycle_timing.json").write_text(
            json.dumps(_json_safe(self.cycle_annotations), indent=2, sort_keys=True),
            encoding="utf-8",
        )
        for sample in self.samples:
            payload = {}
            for index, value in enumerate(sample["parameters"]):
                payload[f"parameter_{index:03d}"] = value
            for index, value in enumerate(sample["outputs"]):
                payload[f"output_{index:03d}"] = value
            np.savez_compressed(
                self.output_dir / f"online_sample_{int(sample['request_id']):06d}.npz",
                request_id=np.asarray(sample["request_id"], dtype=np.int64),
                **payload,
            )


def _load_snapshot_params(layer: Any, snapshot: Path) -> tuple[Any, ...]:
    files = sorted(snapshot.glob("layer_000_parameter_*.npy"))
    if len(files) != len(layer.ctx.parameters):
        raise RuntimeError(
            f"snapshot has {len(files)} parameters, layer expects {len(layer.ctx.parameters)}"
        )
    import torch

    values = []
    for index, path in enumerate(files):
        value = torch.from_numpy(np.load(path, allow_pickle=False).copy())
        value.requires_grad_(index in GRAD_INDICES)
        values.append(value)
    return tuple(values)


def run_equivalence(args: argparse.Namespace) -> None:
    import torch

    if args.torch_intraop > 0:
        torch.set_num_threads(args.torch_intraop)
    if args.torch_interop > 0:
        torch.set_num_interop_threads(args.torch_interop)
    config_path = str(Path(args.config).resolve())
    checkpoint_path = str(Path(args.checkpoint).resolve())
    with open(config_path, "r", encoding="utf-8") as stream:
        import yaml
        config = yaml.safe_load(stream) or {}
    pan = dict(config.get("pan") or {})
    pan["dune_checkpoint"] = checkpoint_path
    layer = _build_layer(config_path, checkpoint_path, pan, None)
    params = _load_snapshot_params(layer, Path(args.snapshot).resolve())
    main_outputs = layer(*params, solver_args={"solve_method": "ECOS"})
    client = IsolatedSolverClient(
        args.output, config_path, checkpoint_path, pan, None,
        args.torch_intraop, args.torch_interop, args.warmup_count,
    )
    try:
        worker_outputs, record = client.solve(params, {"solve_method": "ECOS"})
        comparisons = []
        for index, (main_value, worker_value) in enumerate(zip(main_outputs, worker_outputs)):
            main_array = main_value.detach().cpu().numpy()
            worker_array = worker_value.detach().cpu().numpy()
            diff = np.abs(main_array.astype(np.float64) - worker_array.astype(np.float64))
            comparisons.append({
                "index": index,
                "shape_main": list(main_array.shape),
                "shape_worker": list(worker_array.shape),
                "dtype_main": str(main_array.dtype),
                "dtype_worker": str(worker_array.dtype),
                "finite_main": bool(np.isfinite(main_array).all()),
                "finite_worker": bool(np.isfinite(worker_array).all()),
                "max_abs_diff": float(np.max(diff)),
                "mean_abs_diff": float(np.mean(diff)),
            })
        result = {
            "schema": "r1_n1_14_numerical_equivalence_v1",
            "snapshot": str(Path(args.snapshot).resolve()),
            "start_method": "spawn",
            "solver_args": {"solve_method": "ECOS"},
            "parameter_count": len(params),
            "parameter_shapes": [list(value.shape) for value in params],
            "parameter_dtypes": [str(value.dtype) for value in params],
            "parameter_requires_grad": [bool(value.requires_grad) for value in params],
            "worker_request": record,
            "outputs": comparisons,
            "pass": all(
                row["shape_main"] == row["shape_worker"]
                and row["dtype_main"] == row["dtype_worker"]
                and row["finite_main"] and row["finite_worker"]
                and row["max_abs_diff"] <= float(args.abs_tolerance)
                for row in comparisons
            ),
            "absolute_tolerance": float(args.abs_tolerance),
        }
        out = Path(args.output).resolve()
        out.mkdir(parents=True, exist_ok=True)
        (out / "equivalence.json").write_text(
            json.dumps(_json_safe(result), indent=2, sort_keys=True), encoding="utf-8"
        )
    finally:
        client.finalize()
        del main_outputs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["equivalence"], required=True)
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--warmup-count", type=int, default=30)
    parser.add_argument("--torch-intraop", type=int, default=1)
    parser.add_argument("--torch-interop", type=int, default=24)
    parser.add_argument("--abs-tolerance", type=float, default=1e-4)
    args = parser.parse_args()
    if args.mode == "equivalence":
        run_equivalence(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
