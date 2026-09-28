"""Opt-in N1.11 hooks for the installed CVXPYLayers/diffcp/ECOS path.

The hooks are process-local and disabled unless an output directory is passed.
They copy solver inputs only after the native ECOS call returns, so diagnostic
serialization is outside the narrow native-solver timer.  The optimization
problem and solver arguments are untouched.
"""

from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np


_state = threading.local()
_installed = False
_output_dir: Path | None = None
_originals: dict[str, Any] = {}


def _get(name: str, default: Any = None) -> Any:
    return getattr(_state, name, default)


def _set(name: str, value: Any) -> None:
    setattr(_state, name, value)


def _now() -> tuple[int, int]:
    return time.perf_counter_ns(), time.process_time_ns()


def _json_value(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return repr(value)


def _sha_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _array_copy(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.array(value, copy=True)


def _array_meta(value: Any) -> dict[str, Any]:
    array = np.asarray(value)
    return {
        "shape": list(array.shape),
        "dtype": str(array.dtype),
        "raw_byte_hash": _sha_bytes(np.ascontiguousarray(array).tobytes()),
        "c_contiguous": bool(array.flags.c_contiguous),
        "f_contiguous": bool(array.flags.f_contiguous),
        "writeable": bool(array.flags.writeable),
        "strides": list(array.strides),
        "address": int(array.__array_interface__["data"][0]),
        "base_type": type(array.base).__name__ if array.base is not None else None,
    }


def _sparse_copy(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    matrix = value.tocsc(copy=True)
    return {
        "data": np.array(matrix.data, copy=True),
        "indices": np.array(matrix.indices, copy=True),
        "indptr": np.array(matrix.indptr, copy=True),
        "shape": list(matrix.shape),
        "format": "csc",
        "data_meta": _array_meta(matrix.data),
        "indices_meta": _array_meta(matrix.indices),
        "indptr_meta": _array_meta(matrix.indptr),
        "nnz": int(matrix.nnz),
    }


def _sparse_observe_meta(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    matrix = value.tocsc(copy=False)
    return {
        "shape": list(matrix.shape),
        "format": "csc",
        "nnz": int(matrix.nnz),
        "data_meta": _array_meta(matrix.data),
        "indices_meta": _array_meta(matrix.indices),
        "indptr_meta": _array_meta(matrix.indptr),
    }


def _sparse_hash(value: dict[str, Any] | None) -> str | None:
    if value is None:
        return None
    digest = hashlib.sha256()
    digest.update(json.dumps({"shape": value["shape"], "format": value["format"]}, sort_keys=True).encode())
    for key in ("data", "indices", "indptr"):
        array = np.ascontiguousarray(value[key])
        digest.update(str(array.dtype).encode())
        digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
        digest.update(array.tobytes())
    return digest.hexdigest()


def _array_hash(value: Any) -> str:
    array = np.ascontiguousarray(np.asarray(value))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode())
    digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
    digest.update(array.tobytes())
    return digest.hexdigest()


def _cone_hash(value: Any) -> str:
    return _sha_bytes(json.dumps(_json_value(value), sort_keys=True, separators=(",", ":")).encode())


def _current() -> dict[str, Any] | None:
    return _get("cycle")


def begin_cycle(cycle_id: int) -> None:
    if not _installed:
        return
    _set("cycle", {
        "cycle_id": int(cycle_id),
        "started_perf_ns": _now()[0],
        "layer_events": [],
        "diffcp_events": [],
        "ecos_events": [],
        "layer_forward_count": 0,
        "diffcp_batch_count": 0,
        "ecos_call_count": 0,
        "compile_count_at_cycle": int(_get("compile_count", 0)),
        "active_layer": None,
        "active_diffcp": None,
    })


def end_cycle() -> dict[str, Any] | None:
    cycle = _current()
    if cycle is None:
        return None
    cycle["ended_perf_ns"] = _now()[0]
    cycle["object_lifecycle"] = {
        "layer_ids": sorted({event["layer_id"] for event in cycle["layer_events"]}),
        "problem_ids": sorted({event["problem_id"] for event in cycle["layer_events"]}),
        "ctx_ids": sorted({event["ctx_id"] for event in cycle["layer_events"]}),
        "solver_ctx_ids": sorted({event["solver_ctx_id"] for event in cycle["layer_events"]}),
        "layer_forward_count": cycle["layer_forward_count"],
        "diffcp_batch_count": cycle["diffcp_batch_count"],
        "ecos_call_count": cycle["ecos_call_count"],
        "compile_count_at_cycle": cycle["compile_count_at_cycle"],
    }
    _set("cycle", None)
    return cycle


def layer_forward_start(layer: Any, params: tuple[Any, ...], solver_args: dict[str, Any] | None) -> None:
    cycle = _current()
    if cycle is None:
        return
    started_perf, started_cpu = _now()
    ctx = getattr(layer, "ctx", None)
    parameter_objects = list(getattr(ctx, "parameters", []) or [])
    parameter_values = []
    for index, value in enumerate(params):
        array = _array_copy(value)
        parameter_values.append({
            "index": index,
            "name": parameter_objects[index].name() if index < len(parameter_objects) else f"parameter_{index}",
            "object_id": id(parameter_objects[index]) if index < len(parameter_objects) else None,
            "value_ref": value,
            "device": str(getattr(value, "device", "numpy")),
        })
    event = {
        "layer_id": id(layer),
        "problem_id": id(getattr(ctx, "problem", None)),
        "ctx_id": id(ctx),
        "solver_ctx_id": id(getattr(ctx, "solver_ctx", None)),
        "solver": str(getattr(ctx, "solver", "DIFFCP")),
        "parameter_values": parameter_values,
        "solver_args": _json_value(solver_args or {}),
        "started_perf_ns": started_perf,
        "started_cpu_ns": started_cpu,
        "diffcp_event_indices": [],
        "ecos_event_indices": [],
        "canonical_marshal_wall_ms": 0.0,
        "canonical_marshal_cpu_ms": 0.0,
        "result_recovery_wall_ms": 0.0,
        "result_recovery_cpu_ms": 0.0,
        "parameter_validation_wall_ms": 0.0,
        "parameter_flatten_wall_ms": 0.0,
        "sparse_parameter_eval_wall_ms": 0.0,
    }
    cycle["layer_forward_count"] += 1
    cycle["layer_events"].append(event)
    cycle["active_layer"] = event


def layer_forward_end() -> None:
    layer = (_current() or {}).get("active_layer")
    if layer is None:
        return
    for parameter in layer.get("parameter_values", []):
        array = _array_copy(parameter.pop("value_ref"))
        parameter["value"] = array
        parameter["meta"] = _array_meta(array)
        parameter["hash"] = _array_hash(array)
    ended_perf, ended_cpu = _now()
    layer["ended_perf_ns"] = ended_perf
    layer["ended_cpu_ns"] = ended_cpu
    layer["wall_ms"] = (ended_perf - layer["started_perf_ns"]) / 1_000_000.0
    layer["cpu_ms"] = (ended_cpu - layer["started_cpu_ns"]) / 1_000_000.0
    diffcp_wall = sum(
        float((_current() or {}).get("diffcp_events", [])[index].get("wall_ms", 0.0))
        for index in layer.get("diffcp_event_indices", [])
        if index < len((_current() or {}).get("diffcp_events", []))
    )
    layer["parameter_and_layer_overhead_wall_ms"] = max(
        0.0,
        layer["wall_ms"]
        - diffcp_wall
        - layer["result_recovery_wall_ms"],
    )
    layer["parameter_and_layer_overhead_cpu_ms"] = max(
        0.0,
        layer["cpu_ms"]
        - sum(
            float((_current() or {}).get("diffcp_events", [])[index].get("cpu_ms", 0.0))
            for index in layer.get("diffcp_event_indices", [])
            if index < len((_current() or {}).get("diffcp_events", []))
        )
        - layer["result_recovery_cpu_ms"],
    )
    (_current())["active_layer"] = None


def _timed_matrix_builder(original: Any) -> Any:
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        cycle = _current()
        if cycle is None:
            return original(*args, **kwargs)
        started_perf, started_cpu = _now()
        try:
            return original(*args, **kwargs)
        finally:
            ended_perf, ended_cpu = _now()
            layer = cycle.get("active_layer")
            if layer is not None:
                layer["canonical_marshal_wall_ms"] += (
                    ended_perf - started_perf
                ) / 1_000_000.0
                layer["canonical_marshal_cpu_ms"] += (
                    ended_cpu - started_cpu
                ) / 1_000_000.0
    return wrapper


def _timed_result_recovery(original: Any) -> Any:
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        cycle = _current()
        if cycle is None:
            return original(*args, **kwargs)
        started_perf, started_cpu = _now()
        try:
            return original(*args, **kwargs)
        finally:
            ended_perf, ended_cpu = _now()
            layer = cycle.get("active_layer")
            if layer is not None:
                layer["result_recovery_wall_ms"] += (
                    ended_perf - started_perf
                ) / 1_000_000.0
                layer["result_recovery_cpu_ms"] += (
                    ended_cpu - started_cpu
                ) / 1_000_000.0
    return wrapper


def _timed_stage(original: Any, field: str) -> Any:
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        cycle = _current()
        if cycle is None:
            return original(*args, **kwargs)
        started_perf = time.perf_counter_ns()
        try:
            return original(*args, **kwargs)
        finally:
            layer = cycle.get("active_layer")
            if layer is not None:
                layer[field] += (time.perf_counter_ns() - started_perf) / 1_000_000.0
    return wrapper


def _capture_scs_matrix(value: Any) -> dict[str, Any] | None:
    return _sparse_copy(value)


def _finalize_ecos_event(event: dict[str, Any]) -> None:
    components = {
        "c": _array_hash(event["arrays"]["c"]),
        "G": _sparse_hash(event["arrays"]["G"]),
        "h": _array_hash(event["arrays"]["h"]),
        "A": _sparse_hash(event["arrays"]["A"]),
        "b": _array_hash(event["arrays"]["b"]),
        "cone_dimensions": _cone_hash(event["cone_dimensions"]),
        "solver_kwargs": _cone_hash(event["solver_kwargs"]),
    }
    event["component_hashes"] = components
    event["canonical_problem_hash"] = _sha_bytes(
        json.dumps(components, sort_keys=True, separators=(",", ":")).encode()
    )


def _ecos_wrapper(c: Any, G: Any, h: Any, dims: Any, A: Any = None, b: Any = None, **kwargs: Any) -> Any:
    cycle = _current()
    if cycle is None:
        return _originals["ecos.solve"](c, G, h, dims, A, b, **kwargs)
    started_perf, started_cpu = _now()
    event = {
        "cycle_id": cycle["cycle_id"],
        "started_perf_ns": started_perf,
        "started_cpu_ns": started_cpu,
        "layer_index": len(cycle["layer_events"]) - 1,
        "cone_dimensions": _json_value(dims),
        "solver_kwargs": _json_value(kwargs),
        "input_meta": {
            "c": _array_meta(c),
            "G": _sparse_observe_meta(G),
            "h": _array_meta(h),
            "A": _sparse_observe_meta(A),
            "b": _array_meta(b) if b is not None else None,
        },
        "arrays": {},
        "result": None,
    }
    cycle["ecos_call_count"] += 1
    cycle["ecos_events"].append(event)
    event_index = len(cycle["ecos_events"]) - 1
    active_layer = cycle.get("active_layer")
    if active_layer is not None:
        active_layer["ecos_event_indices"].append(event_index)
    try:
        started_perf, started_cpu = _now()
        event["started_perf_ns"] = started_perf
        event["started_cpu_ns"] = started_cpu
        result = _originals["ecos.solve"](c, G, h, dims, A, b, **kwargs)
        event["status"] = "return-success"
        event["result"] = {
            "info": _json_value(result.get("info", {})) if isinstance(result, dict) else None,
        }
        if isinstance(result, dict):
            event["result_arrays"] = {
                key: _array_copy(result[key])
                for key in ("x", "y", "z") if key in result
            }
        return result
    except Exception as exc:
        event["status"] = "exception"
        event["exception"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        ended_perf, ended_cpu = _now()
        event["ended_perf_ns"] = ended_perf
        event["ended_cpu_ns"] = ended_cpu
        event["native_wall_ms"] = (ended_perf - started_perf) / 1_000_000.0
        event["native_cpu_ms"] = (ended_cpu - started_cpu) / 1_000_000.0
        # Copy after the native return. ECOS receives these arrays synchronously
        # and diffcp does not mutate them after the call returns.
        event["arrays"] = {
            "c": _array_copy(c),
            "G": _sparse_copy(G),
            "h": _array_copy(h),
            "A": _sparse_copy(A),
            "b": _array_copy(b) if b is not None else None,
        }
        _finalize_ecos_event(event)


def _diffcp_batch_wrapper(*args: Any, **kwargs: Any) -> Any:
    cycle = _current()
    if cycle is None:
        return _originals["diffcp.solve_only_batch"](*args, **kwargs)
    started_perf, started_cpu = _now()
    As, bs, cs = args[0], args[1], args[2]
    cone_dicts = args[3]
    event = {
        "kind": "solve_only_batch",
        "started_perf_ns": started_perf,
        "started_cpu_ns": started_cpu,
        "batch_size": len(As),
        "solver_kwargs": _json_value(kwargs),
        "layer_index": len(cycle["layer_events"]) - 1,
        "canonical_scs": {
            "A": None,
            "b": None,
            "c": None,
            "cone_dimensions": _json_value(cone_dicts[0]) if cone_dicts else None,
        },
        "_input_refs": (As[0], bs[0], cs[0]) if As else None,
    }
    cycle["diffcp_batch_count"] += 1
    cycle["diffcp_events"].append(event)
    event_index = len(cycle["diffcp_events"]) - 1
    active_layer = cycle.get("active_layer")
    if active_layer is not None:
        active_layer["diffcp_event_indices"].append(event_index)
    try:
        return _originals["diffcp.solve_only_batch"](*args, **kwargs)
    finally:
        ended_perf, ended_cpu = _now()
        event["ended_perf_ns"] = ended_perf
        event["ended_cpu_ns"] = ended_cpu
        event["wall_ms"] = (ended_perf - started_perf) / 1_000_000.0
        event["cpu_ms"] = (ended_cpu - started_cpu) / 1_000_000.0
        if event.get("_input_refs") is not None:
            matrix, vector_b, vector_c = event.pop("_input_refs")
            event["canonical_scs"] = {
                "A": _sparse_copy(matrix),
                "b": _array_copy(vector_b),
                "c": _array_copy(vector_c),
                "cone_dimensions": event["canonical_scs"]["cone_dimensions"],
            }


def _diffcp_derivative_wrapper(*args: Any, **kwargs: Any) -> Any:
    cycle = _current()
    if cycle is None:
        return _originals["diffcp.solve_and_derivative_batch"](*args, **kwargs)
    As, bs, cs = args[0], args[1], args[2]
    cone_dicts = args[3]
    started_perf, started_cpu = _now()
    event = {
        "kind": "solve_and_derivative_batch",
        "started_perf_ns": started_perf,
        "started_cpu_ns": started_cpu,
        "batch_size": len(As),
        "solver_kwargs": _json_value(kwargs),
        "layer_index": len(cycle["layer_events"]) - 1,
        "canonical_scs": {
            "A": None,
            "b": None,
            "c": None,
            "cone_dimensions": _json_value(cone_dicts[0]) if cone_dicts else None,
        },
        "_input_refs": (As[0], bs[0], cs[0]) if As else None,
    }
    cycle["diffcp_batch_count"] += 1
    cycle["diffcp_events"].append(event)
    event_index = len(cycle["diffcp_events"]) - 1
    active_layer = cycle.get("active_layer")
    if active_layer is not None:
        active_layer["diffcp_event_indices"].append(event_index)
    try:
        return _originals["diffcp.solve_and_derivative_batch"](*args, **kwargs)
    finally:
        cycle.setdefault("derivative_batch_events", []).append({
            "started_perf_ns": started_perf,
            "ended_perf_ns": _now()[0],
            "started_cpu_ns": started_cpu,
            "ended_cpu_ns": _now()[1],
            "wall_ms": (time.perf_counter_ns() - started_perf) / 1_000_000.0,
        })
        ended_perf, ended_cpu = _now()
        event["ended_perf_ns"] = ended_perf
        event["ended_cpu_ns"] = ended_cpu
        event["wall_ms"] = (ended_perf - started_perf) / 1_000_000.0
        event["cpu_ms"] = (ended_cpu - started_cpu) / 1_000_000.0
        if event.get("_input_refs") is not None:
            matrix, vector_b, vector_c = event.pop("_input_refs")
            event["canonical_scs"] = {
                "A": _sparse_copy(matrix),
                "b": _array_copy(vector_b),
                "c": _array_copy(vector_c),
                "cone_dimensions": event["canonical_scs"]["cone_dimensions"],
            }


def install(output_dir: str | os.PathLike[str]) -> dict[str, Any]:
    """Install hooks once and return the process-local hook status."""
    global _installed, _output_dir
    if not output_dir:
        return {"enabled": False, "reason": "empty_output_dir"}
    if _installed:
        return {"enabled": True, "output_dir": str(_output_dir), "already_installed": True}
    import diffcp
    import ecos
    from cvxpylayers.interfaces import diffcp_if
    from cvxpylayers.torch import cvxpylayer as torch_cvxpylayer

    _output_dir = Path(output_dir).expanduser().resolve()
    _output_dir.mkdir(parents=True, exist_ok=True)
    _originals["diffcp.solve_only_batch"] = diffcp.solve_only_batch
    _originals["diffcp.solve_and_derivative_batch"] = diffcp.solve_and_derivative_batch
    _originals["ecos.solve"] = ecos.solve
    _originals["diffcp_if._build_diffcp_matrices"] = diffcp_if._build_diffcp_matrices
    _originals["torch_cvxpylayer._recover_results"] = torch_cvxpylayer._recover_results
    _originals["torch_cvxpylayer._flatten_and_batch_params"] = torch_cvxpylayer._flatten_and_batch_params
    _originals["torch_cvxpylayer._ScipySparseMatmul.forward"] = torch_cvxpylayer._ScipySparseMatmul.forward
    diffcp.solve_only_batch = _diffcp_batch_wrapper
    diffcp.solve_and_derivative_batch = _diffcp_derivative_wrapper
    ecos.solve = _ecos_wrapper
    diffcp_if._build_diffcp_matrices = _timed_matrix_builder(
        diffcp_if._build_diffcp_matrices
    )
    torch_cvxpylayer._recover_results = _timed_result_recovery(
        torch_cvxpylayer._recover_results
    )
    torch_cvxpylayer._flatten_and_batch_params = _timed_stage(
        torch_cvxpylayer._flatten_and_batch_params, "parameter_flatten_wall_ms"
    )
    torch_cvxpylayer._ScipySparseMatmul.forward = staticmethod(
        _timed_stage(
            torch_cvxpylayer._ScipySparseMatmul.forward,
            "sparse_parameter_eval_wall_ms",
        )
    )
    _installed = True
    _set("compile_count", 1)
    return {
        "enabled": True,
        "output_dir": str(_output_dir),
        "installed": True,
        "diffcp_module": str(getattr(diffcp, "__file__", "")),
        "ecos_module": str(getattr(ecos, "__file__", "")),
        "cvxpylayers_diffcp_interface": str(getattr(diffcp_if, "__file__", "")),
        "cvxpylayers_torch_interface": str(getattr(torch_cvxpylayer, "__file__", "")),
    }


def write_cycle(record: dict[str, Any], output_dir: str | os.PathLike[str] | None = None) -> dict[str, Any] | None:
    """Write a completed cycle after its planner call has returned."""
    if not record or not _installed:
        return None
    root = Path(output_dir).expanduser().resolve() if output_dir else _output_dir
    if root is None:
        return None
    cycle_dir = root / f"cycle_{int(record['cycle_id']):06d}"
    cycle_dir.mkdir(parents=True, exist_ok=True)
    metadata = {key: value for key, value in record.items() if key not in ("layer_events", "diffcp_events", "ecos_events")}
    metadata["schema"] = "r1_n1_11_canonical_cycle_v1"
    metadata["solver_files"] = []
    metadata["parameter_files"] = []

    for layer_index, layer in enumerate(record.get("layer_events", [])):
        layer_meta = {key: value for key, value in layer.items() if key != "parameter_values"}
        for parameter_index, parameter in enumerate(layer.get("parameter_values", [])):
            name = f"layer_{layer_index:03d}_parameter_{parameter_index:03d}.npy"
            np.save(cycle_dir / name, parameter["value"], allow_pickle=False)
            parameter_meta = {key: value for key, value in parameter.items() if key != "value"}
            parameter_meta["file"] = name
            metadata["parameter_files"].append(parameter_meta)
        metadata.setdefault("layers", []).append(layer_meta)

    for solver_index, event in enumerate(record.get("ecos_events", [])):
        name = f"solver_{solver_index:03d}.npz"
        arrays: dict[str, Any] = {}
        for key, value in event.get("arrays", {}).items():
            if value is None:
                continue
            if isinstance(value, dict) and "data" in value:
                arrays[f"{key}_data"] = value["data"]
                arrays[f"{key}_indices"] = value["indices"]
                arrays[f"{key}_indptr"] = value["indptr"]
            else:
                arrays[key] = value
        scs_event = record.get("diffcp_events", [])
        if event.get("layer_index", -1) < len(scs_event):
            pass
        np.savez(cycle_dir / name, **arrays)
        solver_meta = {key: value for key, value in event.items() if key not in ("arrays", "input_meta", "result_arrays")}
        solver_meta["file"] = name
        solver_meta["array_meta"] = event.get("input_meta")
        solver_meta["result_array_meta"] = {
            key: _array_meta(value) for key, value in event.get("result_arrays", {}).items()
        }
        metadata["solver_files"].append(solver_meta)

    # Keep the SCS canonical input separately; the ECOS binding input above is
    # the primary equivalence artifact.
    for index, event in enumerate(record.get("diffcp_events", [])):
        scs = event.get("canonical_scs", {})
        arrays: dict[str, Any] = {}
        matrix = scs.get("A")
        if matrix is not None:
            arrays.update({"A_data": matrix["data"], "A_indices": matrix["indices"], "A_indptr": matrix["indptr"]})
        if scs.get("b") is not None:
            arrays["b"] = scs["b"]
        if scs.get("c") is not None:
            arrays["c"] = scs["c"]
        name = f"diffcp_{index:03d}.npz"
        np.savez(cycle_dir / name, **arrays)
        metadata.setdefault("diffcp_files", []).append({
            "file": name,
            "kind": event.get("kind"),
            "batch_size": event.get("batch_size"),
            "wall_ms": event.get("wall_ms"),
            "cpu_ms": event.get("cpu_ms"),
            "cone_dimensions": scs.get("cone_dimensions"),
            "A_hash": _sparse_hash(scs.get("A")),
            "b_hash": _array_hash(scs["b"]) if scs.get("b") is not None else None,
            "c_hash": _array_hash(scs["c"]) if scs.get("c") is not None else None,
        })

    metadata_path = cycle_dir / "metadata.json"
    metadata_path.write_text(json.dumps(_json_value(metadata), indent=2, sort_keys=True), encoding="utf-8")
    return {"cycle_dir": str(cycle_dir), "metadata": str(metadata_path), "solver_count": len(metadata["solver_files"])}
