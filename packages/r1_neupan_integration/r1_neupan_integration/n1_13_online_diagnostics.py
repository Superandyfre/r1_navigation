"""In-process N1.13 attribution for the live NeuPAN CvxpyLayer call.

The module intentionally keeps the primary path scalar-only.  Selected calls
are diagnosed after the primary layer returns, using the original callable and
parameter tuple before those objects leave the callback.  Persistent records
contain only JSON-safe metadata; Tensor objects are never retained in the
diagnostic history.
"""

from __future__ import annotations

import copy
import json
import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

import torch


NORMAL_MAX_MS = 20.0
SLOW_MIN_MS = 100.0
SEVERE_MIN_MS = 500.0
GRAPH_MAX_NODES = 1000
GRAPH_MAX_DEPTH = 5
NORMAL_SAMPLE_EVERY = 20
DIAGNOSTIC_COOLDOWN_CYCLES = 5
GRAD_INDICES = {1, 2, 33, 34, 35, 36, 37}

_state: "OnlineDiagnostics | None" = None


def _thread_info() -> dict[str, Any]:
    return {
        "python_thread_id": threading.get_ident(),
        "native_thread_id": threading.get_native_id(),
        "pid": os.getpid(),
    }


def _torch_state() -> dict[str, Any]:
    state = {
        "grad_enabled": bool(torch.is_grad_enabled()),
        "inference_mode_enabled": bool(
            torch.is_inference_mode_enabled()
            if hasattr(torch, "is_inference_mode_enabled") else False
        ),
        "intraop_threads": int(torch.get_num_threads()),
        "interop_threads": int(torch.get_num_interop_threads()),
    }
    try:
        state["autocast_cpu_enabled"] = bool(torch.is_autocast_cpu_enabled())
    except (AttributeError, RuntimeError):
        state["autocast_cpu_enabled"] = None
    try:
        state["autocast_cuda_enabled"] = bool(torch.is_autocast_enabled())
    except (AttributeError, RuntimeError):
        state["autocast_cuda_enabled"] = None
    return state


def _finite_outputs(outputs: Any) -> bool | None:
    if outputs is None:
        return None
    values = outputs if isinstance(outputs, (tuple, list)) else (outputs,)
    try:
        return all(
            bool(torch.isfinite(value).all().item())
            for value in values if isinstance(value, torch.Tensor)
        )
    except (RuntimeError, TypeError):
        return None


def _graph_summary(grad_fn: Any) -> dict[str, Any] | None:
    if grad_fn is None:
        return None
    queue_nodes = [(grad_fn, 0)]
    seen: set[int] = set()
    max_depth = 0
    edge_count = 0
    truncated = False
    roots = []
    while queue_nodes:
        node, depth = queue_nodes.pop(0)
        node_id = id(node)
        if node_id in seen:
            continue
        if len(seen) >= GRAPH_MAX_NODES:
            truncated = True
            break
        seen.add(node_id)
        max_depth = max(max_depth, depth)
        next_functions = list(getattr(node, "next_functions", ()) or ())
        edge_count += len(next_functions)
        if depth < GRAPH_MAX_DEPTH:
            for child, _ in next_functions:
                if child is not None:
                    queue_nodes.append((child, depth + 1))
        elif next_functions:
            truncated = True
        if depth <= 1 and len(roots) < 32:
            roots.append({
                "type": type(node).__name__,
                "id": node_id,
                "next_count": len(next_functions),
            })
    return {
        "reachable_nodes_bounded": len(seen),
        "depth_bounded": max_depth,
        "edge_count_bounded": edge_count,
        "truncated": truncated,
        "root_summary": roots,
    }


def _parameter_metadata(layer: Any, params: tuple[Any, ...]) -> list[dict[str, Any]]:
    parameter_objects = list(getattr(getattr(layer, "ctx", None), "parameters", []) or [])
    metadata = []
    for index, value in enumerate(params):
        if not isinstance(value, torch.Tensor):
            metadata.append({"index": index, "type": type(value).__name__})
            continue
        try:
            storage_ptr = int(value.untyped_storage().data_ptr())
        except (AttributeError, RuntimeError):
            storage_ptr = None
        grad_fn = value.grad_fn
        base = value._base
        row = {
            "index": index,
            "name": (
                parameter_objects[index].name()
                if index < len(parameter_objects) else f"parameter_{index}"
            ),
            "object_id": id(value),
            "data_ptr": int(value.data_ptr()),
            "storage_ptr": storage_ptr,
            "storage_offset": int(value.storage_offset()),
            "shape": list(value.shape),
            "stride": list(value.stride()),
            "contiguous": bool(value.is_contiguous()),
            "dtype": str(value.dtype),
            "device": str(value.device),
            "requires_grad": bool(value.requires_grad),
            "is_leaf": bool(value.is_leaf),
            "version": int(value._version),
            "grad_fn_type": type(grad_fn).__name__ if grad_fn is not None else None,
            "grad_fn_id": id(grad_fn) if grad_fn is not None else None,
            "has_base": base is not None,
            "base_id": id(base) if base is not None else None,
            "python_refcount": int(sys.getrefcount(value)),
        }
        if index in GRAD_INDICES or value.requires_grad:
            row["grad_graph"] = _graph_summary(grad_fn)
        metadata.append(row)
    return metadata


def _clone_parameters(params: tuple[Any, ...]) -> tuple[Any, ...]:
    cloned = []
    for value in params:
        if not isinstance(value, torch.Tensor):
            cloned.append(value)
            continue
        clone = value.detach().clone()
        clone.requires_grad_(bool(value.requires_grad))
        cloned.append(clone)
    return tuple(cloned)


def _cache_identity(layer: Any) -> dict[str, Any]:
    result = {}
    for name in ("_q_scipy", "_A_scipy", "_P_scipy"):
        value = getattr(layer, name, None)
        if value is None:
            result[name] = None
            continue
        result[name] = {
            "object_id": id(value),
            "data_id": id(getattr(value, "data", None)),
            "indices_id": id(getattr(value, "indices", None)),
            "indptr_id": id(getattr(value, "indptr", None)),
        }
    return result


def _parameter_identity_equal(before: list[dict[str, Any]], after: list[dict[str, Any]]) -> bool:
    fields = ("index", "object_id", "data_ptr", "storage_ptr", "storage_offset", "version")
    return [tuple(row.get(field) for field in fields) for row in before] == [
        tuple(row.get(field) for field in fields) for row in after
    ]


def _call_timed(call: Callable[..., Any], params: tuple[Any, ...], solver_args: dict[str, Any],
                label: str) -> dict[str, Any]:
    started_wall = time.perf_counter_ns()
    started_cpu = time.process_time_ns()
    thread = _thread_info()
    torch_state = _torch_state()
    status = "return-success"
    outputs = None
    exception = None
    try:
        outputs = call(*params, solver_args=dict(solver_args))
    except Exception as exc:  # diagnostic-only; the primary result already exists
        status = "exception"
        exception = f"{type(exc).__name__}: {exc}"
    ended_wall = time.perf_counter_ns()
    ended_cpu = time.process_time_ns()
    result = {
        "label": label,
        "wall_ms": (ended_wall - started_wall) / 1_000_000.0,
        "cpu_ms": (ended_cpu - started_cpu) / 1_000_000.0,
        "status": status,
        "exception": exception,
        "finite": _finite_outputs(outputs),
        "thread": thread,
        "torch_state": torch_state,
    }
    del outputs
    return result


class _Worker:
    def __init__(self) -> None:
        self._requests: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._run, name="n1_13_shadow_worker", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            request = self._requests.get()
            if request is None:
                return
            call, params, solver_args, result_queue = request
            result_queue.put(_call_timed(call, params, solver_args, "worker_same_object"))

    def call(self, call: Callable[..., Any], params: tuple[Any, ...], solver_args: dict[str, Any]) -> dict[str, Any]:
        result_queue: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._requests.put((call, params, solver_args, result_queue))
        return result_queue.get()

    def close(self) -> None:
        self._requests.put(None)
        self._thread.join(timeout=2.0)


class OnlineDiagnostics:
    def __init__(self, output_dir: str, layer: Any, direct_problem: Any = None,
                 shadows_enabled: bool = True) -> None:
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.layer = layer
        self.shadows_enabled = bool(shadows_enabled)
        self.worker = _Worker()
        self.records: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._cycle_id = 0
        self._scan_counter = -1
        self._scan_stamp_sec = -1.0
        self._cycle_start_perf_ns = 0
        self._last_layer_end_perf_ns = 0
        self._last_diagnostic_cycle = -DIAGNOSTIC_COOLDOWN_CYCLES
        self._last_signature: dict[int, dict[str, Any]] = {}
        self._finalized = False
        self.direct_problem = None
        self.direct_parameters = []
        self.direct_error = None
        try:
            import cvxpy as cp
            # The diagnostic Problem is constructed once by the owning NRMP
            # object.  Deep-copying CVXPY 1.9.2's C++ expression graph is unsafe
            # and can segfault during a later canonicalization.
            self.direct_problem = direct_problem
            if self.direct_problem is None:
                raise RuntimeError("dedicated diagnostic CVXPY problem was not provided")
            copied_by_name = {
                parameter.name(): parameter
                for parameter in self.direct_problem.parameters()
            }
            self.direct_parameters = [
                copied_by_name[parameter.name()]
                for parameter in layer.ctx.parameters
            ]
            self.direct_solver = cp.ECOS
        except Exception as exc:  # Keep CvxpyLayer diagnostics usable if copy fails.
            self.direct_error = f"{type(exc).__name__}: {exc}"

    def begin_cycle(self, cycle_id: int, scan_counter: int = -1,
                    scan_stamp_sec: float = -1.0) -> None:
        self._cycle_id = int(cycle_id)
        self._scan_counter = int(scan_counter)
        self._scan_stamp_sec = float(scan_stamp_sec)
        self._cycle_start_perf_ns = time.perf_counter_ns()
        self._last_layer_end_perf_ns = self._cycle_start_perf_ns

    def primary_start(self, layer: Any, params: tuple[Any, ...], solver_args: dict[str, Any] | None) -> dict[str, Any]:
        started_wall = time.perf_counter_ns()
        started_cpu = time.process_time_ns()
        return {
            "cycle_id": self._cycle_id,
            "scan_counter": self._scan_counter,
            "scan_stamp_sec": self._scan_stamp_sec,
            "layer": layer,
            "params": tuple(params),
            "solver_args": dict(solver_args or {}),
            "started_wall_ns": started_wall,
            "started_cpu_ns": started_cpu,
            "pre_layer_wall_ms": (started_wall - self._last_layer_end_perf_ns) / 1_000_000.0,
            "thread": _thread_info(),
            "torch_state_before": _torch_state(),
            "layer_id": id(layer),
            "nrmp_id": id(getattr(layer, "_n113_nrmp_owner", None)) if hasattr(layer, "_n113_nrmp_owner") else None,
        }

    def primary_end(self, token: dict[str, Any], outputs: Any, exception: Exception | None) -> None:
        ended_wall = time.perf_counter_ns()
        ended_cpu = time.process_time_ns()
        token["ended_wall_ns"] = ended_wall
        token["primary_wall_ms"] = (ended_wall - token["started_wall_ns"]) / 1_000_000.0
        token["primary_cpu_ms"] = (ended_cpu - token["started_cpu_ns"]) / 1_000_000.0
        token["status"] = "exception" if exception is not None else "return-success"
        token["exception"] = f"{type(exception).__name__}: {exception}" if exception else None
        token["finite"] = _finite_outputs(outputs)
        token["thread_after"] = _thread_info()
        token["torch_state_after"] = _torch_state()
        token["category"] = self.category(token["primary_wall_ms"])
        self._last_layer_end_perf_ns = ended_wall

    @staticmethod
    def category(wall_ms: float) -> str:
        if wall_ms < NORMAL_MAX_MS:
            return "normal"
        if wall_ms >= SEVERE_MIN_MS:
            return "severe"
        if wall_ms > SLOW_MIN_MS:
            return "slow"
        return "intermediate"

    def _should_diagnose(self, token: dict[str, Any]) -> bool:
        if not self.shadows_enabled:
            return False
        category = token["category"]
        cycle_id = int(token["cycle_id"])
        if category == "normal":
            selected = cycle_id % NORMAL_SAMPLE_EVERY == 0
        elif category in ("slow", "severe"):
            selected = True
        else:
            selected = False
        if not selected:
            return False
        if cycle_id - self._last_diagnostic_cycle < DIAGNOSTIC_COOLDOWN_CYCLES:
            return False
        self._last_diagnostic_cycle = cycle_id
        return True

    def after_primary(self, token: dict[str, Any], original_forward: Callable[..., Any], outputs: Any) -> None:
        params = token["params"]
        selected = self._should_diagnose(token)
        cache_before = _cache_identity(token["layer"])
        record = {
            key: value for key, value in token.items()
            if key not in {"layer", "params", "solver_args", "started_wall_ns", "started_cpu_ns", "ended_wall_ns"}
        }
        record["selected_for_shadow"] = bool(selected)
        record["parameter_reuse"] = self._reuse_signature(params)
        if selected:
            record["parameters"] = _parameter_metadata(token["layer"], params)
            solver_args = token["solver_args"]
            record["same_object"] = _call_timed(
                original_forward, params, solver_args, "same_object"
            )
            cloned = _clone_parameters(params)
            try:
                record["clone_value"] = _call_timed(
                    original_forward, cloned, solver_args, "clone_value"
                )
            finally:
                del cloned
            record["worker_same_object"] = self.worker.call(
                original_forward, params, solver_args
            )
            fresh_started = time.perf_counter_ns()
            fresh_error = None
            fresh_layer = None
            try:
                from cvxpylayers.torch import CvxpyLayer
                fresh_layer = CvxpyLayer(
                    token["layer"].ctx.problem,
                    parameters=token["layer"].ctx.parameters,
                    variables=token["layer"].ctx.variables,
                )
                construction_ms = (time.perf_counter_ns() - fresh_started) / 1_000_000.0
                record["fresh_layer"] = _call_timed(
                    fresh_layer, params, solver_args, "fresh_layer"
                )
            except Exception as exc:
                fresh_error = f"{type(exc).__name__}: {exc}"
            finally:
                if fresh_layer is not None:
                    del fresh_layer
            record.setdefault("fresh_layer", {"label": "fresh_layer", "status": "exception"})
            record["fresh_layer_construction_ms"] = construction_ms if fresh_error is None else None
            record["fresh_layer_error"] = fresh_error
            record["direct_cvxpylayer_problem"] = self._direct_cvxpy(params)
            record["post_primary_parameter_metadata"] = _parameter_metadata(token["layer"], params)
            record["object_graph_summary"] = {
                str(item["index"]): item.get("grad_graph")
                for item in record["parameters"]
                if "grad_graph" in item
            }
            record["cache_identity_before_shadow"] = cache_before
            record["cache_identity_after_shadow"] = _cache_identity(token["layer"])
            record["mutation_check"] = {
                "parameter_identity_unchanged": (
                    _parameter_identity_equal(
                        record["parameters"], record["post_primary_parameter_metadata"]
                    )
                ),
                "layer_cache_identity_unchanged": (
                    record["cache_identity_before_shadow"]
                    == record["cache_identity_after_shadow"]
                ),
            }
        else:
            record["cache_identity"] = cache_before
        with self._lock:
            self.records.append(record)
        del outputs

    def _reuse_signature(self, params: tuple[Any, ...]) -> dict[str, Any]:
        current = {}
        reused_object = []
        reused_storage = []
        for index, value in enumerate(params):
            if not isinstance(value, torch.Tensor):
                continue
            try:
                storage_ptr = int(value.untyped_storage().data_ptr())
            except (AttributeError, RuntimeError):
                storage_ptr = None
            signature = {
                "object_id": id(value),
                "data_ptr": int(value.data_ptr()),
                "storage_ptr": storage_ptr,
                "grad_fn_id": id(value.grad_fn) if value.grad_fn is not None else None,
                "version": int(value._version),
            }
            previous = self._last_signature.get(index)
            if previous:
                if previous["object_id"] == signature["object_id"]:
                    reused_object.append(index)
                if previous["storage_ptr"] == signature["storage_ptr"]:
                    reused_storage.append(index)
            current[index] = signature
        self._last_signature = current
        return {
            "object_reused_indices": reused_object,
            "storage_reused_indices": reused_storage,
            "current": current,
        }

    def _direct_cvxpy(self, params: tuple[Any, ...]) -> dict[str, Any]:
        if self.direct_problem is None:
            return {"status": "unavailable", "error": self.direct_error}
        assignment_started = time.perf_counter_ns()
        try:
            for parameter, value in zip(self.direct_parameters, params):
                parameter.value = value.detach().cpu().numpy()
            assignment_ms = (time.perf_counter_ns() - assignment_started) / 1_000_000.0
            solve_started = time.perf_counter_ns()
            self.direct_problem.solve(solver=self.direct_solver, warm_start=False)
            solve_ms = (time.perf_counter_ns() - solve_started) / 1_000_000.0
            recovery_started = time.perf_counter_ns()
            values = [variable.value for variable in self.direct_problem.variables()]
            finite = all(
                value is not None and bool(torch.isfinite(torch.as_tensor(value)).all().item())
                for value in values
            )
            recovery_ms = (time.perf_counter_ns() - recovery_started) / 1_000_000.0
            return {
                "status": "return-success",
                "problem_status": str(self.direct_problem.status),
                "assignment_ms": assignment_ms,
                "solve_ms": solve_ms,
                "recovery_ms": recovery_ms,
                "total_ms": assignment_ms + solve_ms + recovery_ms,
                "finite": finite,
                "thread": _thread_info(),
                "torch_state": _torch_state(),
            }
        except Exception as exc:
            return {
                "status": "exception",
                "error": f"{type(exc).__name__}: {exc}",
                "assignment_ms": (time.perf_counter_ns() - assignment_started) / 1_000_000.0,
            }

    def annotate_cycle(self, cycle_id: int, timing: dict[str, Any]) -> None:
        values = {
            key: float(timing[key]) for key in (
                "core_forward_ms", "pan_ms", "dune_ms", "nrmp_ms",
                "core_forward_cpu_ms", "ecos_solve_ms", "ecos_solve_cpu_ms",
            ) if key in timing
        }
        with self._lock:
            for record in reversed(self.records):
                if int(record.get("cycle_id", -1)) == int(cycle_id):
                    record["stage_timing"] = values
                    break

    def finalize(self) -> None:
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
            records = copy.deepcopy(self.records)
        self.worker.close()
        events_path = self.output_dir / "primary_and_shadow.jsonl"
        with events_path.open("w", encoding="utf-8") as stream:
            for record in records:
                stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        summary = _summary(records)
        summary.update({
            "schema": "r1_n1_13_online_context_v2",
            "thresholds_ms": {
                "normal_lt": NORMAL_MAX_MS,
                "slow_gt": SLOW_MIN_MS,
                "severe_ge": SEVERE_MIN_MS,
            },
            "layer_id": id(self.layer),
            "pid": os.getpid(),
            "direct_problem_error": self.direct_error,
            "shadows_enabled": self.shadows_enabled,
            "persistent_records_hold_tensor_objects": False,
            "executor_context": {
                "executor": "MultiThreadedExecutor(num_threads=2)",
                "control_callback_group": "MutuallyExclusiveCallbackGroup",
                "sensor_callback_group": "ReentrantCallbackGroup",
            },
            "records_file": str(events_path),
        })
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False),
            encoding="utf-8",
        )


def _summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    categories = {name: [] for name in ("normal", "intermediate", "slow", "severe")}
    for record in records:
        categories.setdefault(record.get("category", "unknown"), []).append(record)

    def stats(values: list[float]) -> dict[str, Any]:
        if not values:
            return {"count": 0, "median_ms": None, "p95_ms": None, "max_ms": None}
        ordered = sorted(values)
        return {
            "count": len(values),
            "median_ms": ordered[len(ordered) // 2],
            "p95_ms": ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))],
            "max_ms": max(ordered),
        }

    result = {
        "total_primary_calls": len(records),
        "category_counts": {name: len(rows) for name, rows in categories.items()},
        "primary_wall_ms": stats([float(row["primary_wall_ms"]) for row in records]),
        "primary_cpu_ms": stats([float(row["primary_cpu_ms"]) for row in records]),
        "selected_shadow_calls": sum(1 for row in records if row.get("selected_for_shadow")),
        "selected_by_category": {
            name: sum(1 for row in rows if row.get("selected_for_shadow"))
            for name, rows in categories.items()
        },
    }
    for label, key in (
        ("same_object", "same_object"),
        ("clone_value", "clone_value"),
        ("worker_same_object", "worker_same_object"),
        ("fresh_layer", "fresh_layer"),
    ):
        values = [float(row[key]["wall_ms"]) for row in records if key in row and "wall_ms" in row[key]]
        result[label + "_wall_ms"] = stats(values)
    direct = [row["direct_cvxpylayer_problem"] for row in records if "direct_cvxpylayer_problem" in row]
    result["direct_cvxpy_total_ms"] = stats([
        float(row["total_ms"]) for row in direct if "total_ms" in row
    ])
    result["object_reuse"] = {
        "records_with_object_reuse": sum(
            bool(row.get("parameter_reuse", {}).get("object_reused_indices"))
            for row in records
        ),
        "records_with_storage_reuse": sum(
            bool(row.get("parameter_reuse", {}).get("storage_reused_indices"))
            for row in records
        ),
    }
    return result


def install(output_dir: str, layer: Any, direct_problem: Any = None,
            shadows_enabled: bool = True) -> dict[str, Any]:
    global _state
    if not output_dir:
        return {"enabled": False, "reason": "empty_output_dir"}
    if _state is not None:
        return {"enabled": True, "already_installed": True, "output_dir": str(_state.output_dir)}
    _state = OnlineDiagnostics(output_dir, layer, direct_problem, shadows_enabled)
    return {
        "enabled": True,
        "output_dir": str(_state.output_dir),
        "layer_id": id(layer),
        "direct_problem_available": _state.direct_problem is not None,
        "direct_problem_error": _state.direct_error,
        "shadows_enabled": _state.shadows_enabled,
    }


def begin_cycle(cycle_id: int, scan_counter: int = -1,
                scan_stamp_sec: float = -1.0) -> None:
    if _state is not None:
        _state.begin_cycle(cycle_id, scan_counter, scan_stamp_sec)


def primary_start(layer: Any, params: tuple[Any, ...], solver_args: dict[str, Any] | None) -> dict[str, Any] | None:
    return _state.primary_start(layer, params, solver_args) if _state is not None else None


def primary_end(token: dict[str, Any] | None, outputs: Any, exception: Exception | None) -> None:
    if _state is not None and token is not None:
        _state.primary_end(token, outputs, exception)


def after_primary(token: dict[str, Any] | None, original_forward: Callable[..., Any], outputs: Any) -> None:
    if _state is not None and token is not None:
        _state.after_primary(token, original_forward, outputs)


def annotate_cycle(cycle_id: int, timing: dict[str, Any]) -> None:
    if _state is not None:
        _state.annotate_cycle(cycle_id, timing)


def finalize() -> None:
    if _state is not None:
        _state.finalize()
