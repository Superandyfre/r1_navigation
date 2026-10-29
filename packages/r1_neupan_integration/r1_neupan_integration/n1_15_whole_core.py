"""N1.15 whole-NeuPAN process isolation experiment.

The ROS process owns only subscriptions, latest-data scheduling, IPC and
publishing.  A single long-lived ``spawn`` worker owns the complete NeuPAN
planner, including PAN/DUNE/NRMP, the CvxpyLayer and all receding-horizon
state.  The module also contains the offline stateful-equivalence harness.
"""

from __future__ import annotations

import argparse
import atexit
import hashlib
import json
import math
import multiprocessing as mp
import os
import pickle
import platform
import resource
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from r1_neupan_integration.n2_pre_semantics import (
    DEFAULT_GEAR,
    normalize_path_matrix,
    navigation_loop,
    validate_gear,
)
from r1_neupan_integration.n2_4_recovery import (
    RecoveryObservation,
    evaluate_trigger,
)


DEFAULT_TIMEOUT_SEC = 30.0
N115_SCHEMA = 3.0


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


def _main_inventory() -> dict[str, Any]:
    try:
        load_average = list(os.getloadavg())
    except OSError:
        load_average = None
    try:
        max_rss_kb = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    except (AttributeError, ValueError):
        max_rss_kb = -1
    def read(path: Path) -> str | None:
        try:
            return path.read_text(encoding="ascii").strip()
        except (OSError, UnicodeError):
            return None

    governors = {
        path.parent.parent.name: read(path)
        for path in sorted(Path("/sys/devices/system/cpu").glob(
            "cpu[0-9]*/cpufreq/scaling_governor"
        ))
    }
    frequencies = {
        path.parent.parent.name: read(path)
        for path in sorted(Path("/sys/devices/system/cpu").glob(
            "cpu[0-9]*/cpufreq/scaling_cur_freq"
        ))
    }
    temperatures = {
        path.name: read(path / "temp")
        for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*"))
    }
    return {
        "pid": os.getpid(),
        "python_executable": sys.executable,
        "python_version": sys.version,
        "platform": platform.platform(),
        "load_average": load_average,
        "max_rss_kb": max_rss_kb,
        "process_time_sec": time.process_time(),
        "cpu_governors": governors,
        "cpu_frequency_raw": frequencies,
        "cpu_temperature_raw": temperatures,
    }


def _worker_inventory() -> dict[str, Any]:
    import torch
    import threadpoolctl

    inventory = _main_inventory()
    inventory.update({
        "torch_version": str(torch.__version__),
        "torch_intraop_threads": int(torch.get_num_threads()),
        "torch_interop_threads": int(torch.get_num_interop_threads()),
        "environment": {
            key: os.environ.get(key)
            for key in (
                "OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                "NUMEXPR_NUM_THREADS",
            )
        },
        "threadpoolctl": threadpoolctl.threadpool_info(),
    })
    return inventory


def _set_torch_policy(intraop: int, interop: int) -> dict[str, Any]:
    import torch

    errors = []
    try:
        torch.set_num_threads(int(intraop))
    except RuntimeError as exc:
        errors.append(f"intraop: {type(exc).__name__}: {exc}")
    try:
        torch.set_num_interop_threads(int(interop))
    except RuntimeError as exc:
        errors.append(f"interop: {type(exc).__name__}: {exc}")
    return {"errors": errors, "inventory": _worker_inventory()}


def _pack_array(value: Any) -> dict[str, Any]:
    if value is None:
        array = np.empty((0,), dtype=np.float64)
    else:
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        array = np.ascontiguousarray(np.asarray(value))
    return {
        "shape": list(array.shape),
        "dtype": str(array.dtype),
        "nbytes": int(array.nbytes),
        "data": array.tobytes(order="C"),
    }


def _unpack_array(record: dict[str, Any]) -> np.ndarray:
    dtype = np.dtype(record["dtype"])
    shape = tuple(int(value) for value in record["shape"])
    raw = record["data"]
    if int(record["nbytes"]) != len(raw):
        raise ValueError("numeric IPC payload byte count mismatch")
    return np.frombuffer(raw, dtype=dtype).reshape(shape).copy()


def _exact_clone(value: Any) -> Any:
    """Clone numeric planner data without retaining torch autograd graphs."""
    if hasattr(value, "detach") and hasattr(value, "cpu"):
        return np.ascontiguousarray(value.detach().cpu().numpy()).copy()
    if isinstance(value, np.ndarray):
        return np.ascontiguousarray(value).copy()
    if isinstance(value, dict):
        return {str(key): _exact_clone(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_exact_clone(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_exact_clone(item) for item in value)
    if isinstance(value, np.generic):
        return value.item()
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return repr(value)


def _exact_encode(value: Any, arrays: dict[str, np.ndarray], prefix: str) -> Any:
    """Replace arrays with NPZ references while retaining JSON shape/dtype metadata."""
    if hasattr(value, "detach") and hasattr(value, "cpu"):
        array = np.ascontiguousarray(value.detach().cpu().numpy()).copy()
        kind = "torch"
        requires_grad = bool(getattr(value, "requires_grad", False))
    elif isinstance(value, np.ndarray):
        array = np.ascontiguousarray(value).copy()
        kind = "numpy"
        requires_grad = False
    else:
        if isinstance(value, dict):
            return {str(key): _exact_encode(item, arrays, f"{prefix}.{key}") for key, item in value.items()}
        if isinstance(value, list):
            return [_exact_encode(item, arrays, f"{prefix}[{index}]") for index, item in enumerate(value)]
        if isinstance(value, tuple):
            return {
                "__tuple__": [
                    _exact_encode(item, arrays, f"{prefix}[{index}]")
                    for index, item in enumerate(value)
                ]
            }
        if isinstance(value, np.generic):
            return value.item()
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        return repr(value)
    arrays[prefix] = array
    return {
        "__array__": prefix,
        "kind": kind,
        "requires_grad": requires_grad,
        "shape": list(array.shape),
        "dtype": str(array.dtype),
        "nbytes": int(array.nbytes),
    }


def _exact_decode(value: Any, arrays: dict[str, np.ndarray]) -> Any:
    if isinstance(value, dict) and "__array__" in value:
        return np.ascontiguousarray(arrays[str(value["__array__"])]).copy()
    if isinstance(value, dict) and "__tuple__" in value:
        return tuple(_exact_decode(item, arrays) for item in value["__tuple__"])
    if isinstance(value, dict):
        return {str(key): _exact_decode(item, arrays) for key, item in value.items()}
    if isinstance(value, list):
        return [_exact_decode(item, arrays) for item in value]
    return value


def _state_attrs(value: Any, names: tuple[str, ...]) -> dict[str, Any]:
    return {name: _exact_clone(getattr(value, name, None)) for name in names}


def _planner_exact_state(planner: Any) -> dict[str, Any]:
    """Snapshot every mutable field observed on the real NeuPAN call chain."""
    ipath = planner.ipath
    pan = planner.pan
    dune = getattr(pan, "dune_layer", None)
    nrmp = getattr(pan, "nrmp_layer", None)
    return {
        "planner": _state_attrs(
            planner,
            ("T", "dt", "ref_speed", "collision_threshold", "cur_vel_array", "info"),
        ),
        "ipath": _state_attrs(
            ipath,
            (
                "T", "dt", "ref_speed", "waypoints", "loop", "curve_style",
                "min_radius", "interval", "arrive_threshold", "close_threshold",
                "ind_range", "arrive_index_threshold", "arrive_flag", "initial_path",
                "curve_list", "curve_index", "point_index",
            ),
        ),
        "pan": _state_attrs(
            pan,
            ("T", "dt", "iter_num", "iter_threshold", "nrmp_max_num", "dune_max_num", "current_nom_values", "printed"),
        ),
        "dune": None if dune is None else _state_attrs(dune, ("obstacle_points", "min_distance")),
        "nrmp": None if nrmp is None else _state_attrs(nrmp, ("obstacle_points",)),
        "state_contract": {
            "planner": ["cur_vel_array", "info"],
            "initial_path": [
                "initial_path", "curve_list", "curve_index", "point_index",
                "waypoints", "interval", "loop", "arrive_flag",
            ],
            "pan": ["current_nom_values", "iter_num", "iter_threshold"],
            "dune": ["obstacle_points", "min_distance"],
            "nrmp": ["obstacle_points"],
            "excluded_static": [
                "robot.G", "robot.h", "DUNE.model", "NRMP.CvxpyLayer",
            ],
        },
    }


def _as_numpy_tree(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return np.ascontiguousarray(value).copy()
    if isinstance(value, dict):
        return {key: _as_numpy_tree(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_as_numpy_tree(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_as_numpy_tree(item) for item in value)
    return value


def _as_torch_tree(value: Any) -> Any:
    import torch

    if isinstance(value, np.ndarray):
        return torch.from_numpy(np.ascontiguousarray(value).copy())
    if isinstance(value, dict):
        return {key: _as_torch_tree(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_as_torch_tree(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_as_torch_tree(item) for item in value)
    return value


def _restore_planner_state(planner: Any, snapshot: dict[str, Any]) -> None:
    """Restore only mutable solve state; static model/solver objects stay newly constructed."""
    planner_state = snapshot.get("planner", {})
    if planner_state.get("cur_vel_array") is not None:
        planner.cur_vel_array = _as_numpy_tree(planner_state["cur_vel_array"])
    if planner_state.get("info") is not None:
        planner.info = _as_torch_tree(planner_state["info"])

    ipath_state = snapshot.get("ipath", {})
    for name in (
        "T", "dt", "ref_speed", "waypoints", "loop", "curve_style", "min_radius",
        "interval", "arrive_threshold", "close_threshold", "ind_range",
        "arrive_index_threshold", "arrive_flag", "initial_path", "curve_list",
        "curve_index", "point_index",
    ):
        if name in ipath_state and ipath_state[name] is not None:
            setattr(planner.ipath, name, _as_numpy_tree(ipath_state[name]) if name in {"waypoints", "initial_path", "curve_list"} else ipath_state[name])

    pan_state = snapshot.get("pan", {})
    if pan_state.get("current_nom_values") is not None:
        planner.pan.current_nom_values = _as_torch_tree(pan_state["current_nom_values"])
    for name in ("T", "dt", "iter_num", "iter_threshold", "nrmp_max_num", "dune_max_num", "printed"):
        if name in pan_state and pan_state[name] is not None:
            setattr(planner.pan, name, pan_state[name])

    for layer_name in ("dune", "nrmp"):
        layer_state = snapshot.get(layer_name)
        layer = getattr(planner.pan, f"{layer_name}_layer", None)
        if layer is None or layer_state is None:
            continue
        for name, value in layer_state.items():
            if value is not None:
                setattr(
                    layer,
                    name,
                    _as_torch_tree(value)
                    if name in {"obstacle_points", "min_distance"}
                    else value,
                )


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _exact_config_identity(planner: Any, config_path: str, checkpoint_path: str,
                           pan: dict[str, Any], receding: int | None,
                           torch_policy: dict[str, Any]) -> dict[str, Any]:
    import torch
    import yaml

    try:
        yaml_config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        yaml_config = {"load_error": f"{type(exc).__name__}: {exc}"}
    effective = dict(yaml_config)
    effective["pan"] = dict(pan)
    if receding is not None:
        effective["receding_override"] = int(receding)
    robot = planner.robot
    source_root = Path(__import__("neupan").__file__).resolve().parents[1]
    try:
        source_commit = subprocess.run(
            ["git", "-C", str(source_root), "rev-parse", "HEAD"],
            check=False, capture_output=True, text=True,
        ).stdout.strip() or None
    except OSError:
        source_commit = None
    integration_source = Path(__file__).resolve()
    return {
        "config_path": str(Path(config_path).resolve()),
        "config_sha256": _sha256_file(config_path),
        "effective_yaml": _json_safe(effective),
        "checkpoint_path": str(Path(checkpoint_path).resolve()),
        "checkpoint_sha256": _sha256_file(checkpoint_path),
        "neupan_source_root": str(source_root),
        "neupan_commit": source_commit,
        "integration_source": str(integration_source),
        "integration_source_sha256": _sha256_file(str(integration_source)),
        "torch_version": str(torch.__version__),
        "torch_dtype": "torch.float32",
        "device": "cpu",
        "thread_policy": {
            "errors": list(torch_policy.get("errors", [])),
            "torch_intraop_threads": int(
                torch_policy.get("inventory", {}).get("torch_intraop_threads", -1)
            ),
            "torch_interop_threads": int(
                torch_policy.get("inventory", {}).get("torch_interop_threads", -1)
            ),
            "environment": _json_safe(
                torch_policy.get("inventory", {}).get("environment", {})
            ),
            "threadpoolctl": _json_safe(
                torch_policy.get("inventory", {}).get("threadpoolctl", [])
            ),
        },
        "robot_geometry": {
            "kinematics": str(robot.kinematics),
            "length": float(yaml_config.get("robot", {}).get("length", robot.vertices[:, 0].max() - robot.vertices[:, 0].min())),
            "width": float(yaml_config.get("robot", {}).get("width", robot.vertices[:, 1].max() - robot.vertices[:, 1].min())),
            "vertices": _exact_clone(robot.vertices),
            "G": _exact_clone(robot.G),
            "h": _exact_clone(robot.h),
        },
        "pan_runtime": {
            "iter_num": int(planner.pan.iter_num),
            "dune_max_num": int(planner.pan.dune_max_num),
            "nrmp_max_num": int(planner.pan.nrmp_max_num),
            "iter_threshold": float(planner.pan.iter_threshold),
            "receding": int(planner.T),
            "step_time": float(planner.dt),
        },
    }


class _ExactCaptureRecorder:
    """Keep exact solve records in memory and persist them only at shutdown."""

    def __init__(self, directory: str, max_records: int = 2048,
                 capture_metadata: dict[str, Any] | None = None) -> None:
        self.directory = Path(directory).expanduser().resolve() if directory else None
        self.max_records = max(1, int(max_records))
        self.capture_metadata = dict(capture_metadata or {})
        self.records: list[dict[str, Any]] = []
        self.current: dict[str, Any] | None = None
        self.identity: dict[str, Any] = {}
        self.hook_enabled = self.directory is not None
        self.trigger_history: list[RecoveryObservation] = []

    def begin(self, request: dict[str, Any], planner: Any, state: np.ndarray, points: Any) -> None:
        if not self.hook_enabled:
            return
        self.current = {
            "request": {
                "request_id": int(request["request_id"]),
                "cycle_id": int(request.get("cycle_id", -1)),
                "scan_counter": int(request.get("scan_counter", -1)),
                "scan_stamp_sec": float(request.get("scan_stamp_sec", -1.0)),
                "path_version": int(request.get("path_version", 0)),
                "goal_version": int(request.get("goal_version", -1)),
                "navigation_loop": bool(request.get("navigation_loop", True)),
                "active_gear": int(request.get("active_gear", DEFAULT_GEAR)),
                "main_send_started_monotonic": float(
                    request.get("main_send_started_monotonic", -1.0)
                ),
            },
            "input": {
                "state": _exact_clone(state),
                "obstacle_points": _exact_clone(points),
                "effective_path": _exact_clone(_planner_path_matrix(planner)),
                "goal": _exact_clone(_unpack_array(request.get("goal", _pack_array(None)))),
            },
            "planner_state_pre": _planner_exact_state(planner),
            "pan_calls": [],
            "dune_calls": [],
            "nrmp_calls": [],
            "stop_criteria_calls": [],
        }

    def _append(self, key: str, value: Any) -> None:
        if self.current is not None:
            self.current[key].append(_exact_clone(value))

    def pan_call(self, args: tuple[Any, ...], kwargs: dict[str, Any], result: Any = None) -> None:
        self._append("pan_calls", {"args": args, "kwargs": kwargs, "result": result})

    def dune_call(self, args: tuple[Any, ...], kwargs: dict[str, Any], result: Any) -> None:
        self._append("dune_calls", {"args": args, "kwargs": kwargs, "result": result})

    def nrmp_call(self, args: tuple[Any, ...], kwargs: dict[str, Any], result: Any) -> None:
        self._append("nrmp_calls", {"args": args, "kwargs": kwargs, "result": result})

    def stop_call(self, args: tuple[Any, ...], kwargs: dict[str, Any], result: Any) -> None:
        self._append("stop_criteria_calls", {"args": args, "kwargs": kwargs, "result": bool(result)})

    @staticmethod
    def _path_xy(path: Any) -> tuple[tuple[float, float], ...]:
        value = np.asarray(path)
        if value.ndim != 2 or value.shape[0] < 2:
            return ()
        return tuple((float(point[0]), float(point[1])) for point in value[:2].T)

    @staticmethod
    def _path_deviation(state: Any, path: tuple[tuple[float, float], ...]) -> float:
        value = np.asarray(state).reshape(-1)
        if value.size < 2 or len(path) < 2:
            return math.inf
        x, y = float(value[0]), float(value[1])
        best = math.inf
        for left, right in zip(path, path[1:]):
            dx, dy = right[0] - left[0], right[1] - left[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                distance = math.hypot(x - left[0], y - left[1])
            else:
                fraction = max(
                    0.0,
                    min(1.0, ((x - left[0]) * dx + (y - left[1]) * dy) / length_sq),
                )
                distance = math.hypot(
                    x - (left[0] + fraction * dx),
                    y - (left[1] + fraction * dy),
                )
            best = min(best, distance)
        return best

    @staticmethod
    def _scalar(value: Any) -> float | None:
        if value is None:
            return None
        value = np.asarray(value)
        if value.size != 1:
            return None
        try:
            result = float(value.reshape(-1)[0])
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    def _formal_trigger(self, request: dict[str, Any], planner_state: dict[str, Any],
                        state: Any, action: Any, info: dict[str, Any],
                        outputs: dict[str, Any]) -> dict[str, Any]:
        """Observe the frozen trigger after the real solve, without side effects."""
        started = time.perf_counter_ns()
        path = self._path_xy(self.current["input"]["effective_path"])
        state_value = np.asarray(state).reshape(-1)
        action_value = np.asarray(action).reshape(-1)
        point_index = int(planner_state.get("ipath", {}).get("point_index", -1))
        goal_value = np.asarray(self.current["input"].get("goal", np.empty((0, 0))))
        if goal_value.size and goal_value.reshape(-1).size >= 2:
            goal_x, goal_y = (float(goal_value.reshape(-1)[0]), float(goal_value.reshape(-1)[1]))
        elif path:
            goal_x, goal_y = path[-1]
        else:
            goal_x, goal_y = math.nan, math.nan
        reference = np.asarray(outputs.get("ref_state", np.empty((0, 0))))
        optimized = np.asarray(outputs.get("opt_state", np.empty((0, 0))))
        reference_points = (
            tuple((float(point[0]), float(point[1])) for point in reference[:2].T)
            if reference.ndim == 2 and reference.shape[0] >= 2 else ()
        )
        optimized_points = (
            tuple((float(point[0]), float(point[1])) for point in optimized[:2].T)
            if optimized.ndim == 2 and optimized.shape[0] >= 2 else ()
        )
        dune = planner_state.get("dune") or {}
        observation = RecoveryObservation(
            cycle_id=int(request.get("cycle_id", -1)),
            monotonic=float(request.get("main_send_started_monotonic", -1.0)),
            raw_linear=float(action_value[0]) if action_value.size >= 1 else math.nan,
            raw_angular=float(action_value[1]) if action_value.size >= 2 else math.nan,
            pose=(
                float(state_value[0]) if state_value.size >= 1 else math.nan,
                float(state_value[1]) if state_value.size >= 2 else math.nan,
                float(state_value[2]) if state_value.size >= 3 else math.nan,
            ),
            path=path,
            active_path_index=point_index,
            path_deviation_m=self._path_deviation(state, path),
            goal_distance_m=(
                math.hypot(float(state_value[0]) - goal_x, float(state_value[1]) - goal_y)
                if state_value.size >= 2 and math.isfinite(goal_x) and math.isfinite(goal_y)
                else None
            ),
            stop=bool(info.get("stop", False)),
            arrive=bool(info.get("arrive", False)),
            planner_fresh=True,
            path_valid=len(path) >= 2,
            obstacle_clearance_m=self._scalar(dune.get("min_distance")),
            corridor_margin_m=self._scalar(request.get("n2_4_corridor_margin_m")),
            reference_points=reference_points,
            optimized_points=optimized_points,
        )
        decision = evaluate_trigger(self.trigger_history, observation)
        self.trigger_history.append(observation)
        elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000.0
        return {
            "detector": "r1_neupan_integration.n2_4_recovery.evaluate_trigger",
            "detected": bool(decision["trigger"]),
            "trigger_cycle": int(observation.cycle_id) if decision["trigger"] else None,
            "trigger_reason": str(decision.get("trigger_reason", "")),
            "decision": decision,
            "observation": {
                "cycle_id": int(observation.cycle_id),
                "monotonic": float(observation.monotonic),
                "raw_linear": float(observation.raw_linear),
                "raw_angular": float(observation.raw_angular),
                "pose": [float(value) for value in observation.pose],
                "path": [[float(x), float(y)] for x, y in observation.path],
                "active_path_index": int(observation.active_path_index),
                "path_deviation_m": float(observation.path_deviation_m),
                "goal_distance_m": observation.goal_distance_m,
                "stop": bool(observation.stop),
                "arrive": bool(observation.arrive),
                "planner_fresh": bool(observation.planner_fresh),
                "path_valid": bool(observation.path_valid),
                "obstacle_clearance_m": observation.obstacle_clearance_m,
                "corridor_margin_m": observation.corridor_margin_m,
            },
            "detector_latency_ms": float(elapsed_ms),
            "side_effects": {
                "command_published_by_detector": False,
                "planner_input_modified": False,
                "planner_state_modified": False,
                "recovery_state_modified": False,
            },
        }

    def finish(self, planner: Any, action: Any, info: dict[str, Any], packed: dict[str, Any], response: dict[str, Any]) -> None:
        if self.current is None:
            return
        outputs = {key: _unpack_array(value) for key, value in packed.items()}
        planner_state_post = _planner_exact_state(planner)
        formal_trigger = self._formal_trigger(
            self.current["request"], planner_state_post,
            self.current["input"]["state"], action, info, outputs,
        )
        self.current.update({
            "planner_state_post": planner_state_post,
            "n2_4_trigger": formal_trigger,
            "original_result": {
                "action": _exact_clone(action),
                "info": _exact_clone({key: info.get(key) for key in ("stop", "arrive", "collision")}),
                "outputs": _exact_clone(outputs),
                "timings": _exact_clone(response.get("timings", {})),
                "semantic": _exact_clone(response.get("semantic", {})),
                "finite": bool(response.get("finite", False)),
            },
        })
        self.records.append(self.current)
        if len(self.records) > self.max_records:
            self.records = self.records[-self.max_records:]
        self.current = None

    def dump(self, extra_metadata: dict[str, Any] | None = None) -> dict[str, Any]:
        if not self.hook_enabled or self.directory is None:
            return {"status": "DISABLED"}
        self.directory.mkdir(parents=True, exist_ok=True)
        arrays: dict[str, np.ndarray] = {}
        encoded_records = [
            _exact_encode(record, arrays, f"record[{index}]")
            for index, record in enumerate(self.records)
        ]
        numeric_path = self.directory / "numeric_payload.npz"
        np.savez_compressed(numeric_path, **arrays)
        nonfinite = sum(not np.isfinite(array).all() for array in arrays.values() if np.issubdtype(array.dtype, np.number))
        merged_metadata = dict(self.capture_metadata)
        merged_metadata.update(dict(extra_metadata or {}))
        metadata = {
            "schema": "r1_n2_4_exact_state_capture_v1",
            "record_count": len(encoded_records),
            "ring_capacity": self.max_records,
            "records": encoded_records,
            "numeric_payload": {
                "path": str(numeric_path.resolve()),
                "sha256": _sha256_file(str(numeric_path)),
                "array_count": len(arrays),
                "nonfinite_array_count": int(nonfinite),
            },
            "config_identity": _json_safe(self.identity),
            "capture_contract": {
                "source_boundary": "worker request decode -> planner(state, points) -> response pack",
                "robot_state": "request state passed directly to neupan.forward",
                "path": "effective planner.ipath state plus request path/version/goal metadata",
                "obstacles": "request points plus DUNE point-flow/mu/lam/sorted-point calls",
                "cross_cycle": "planner.cur_vel_array, InitialPath progress, PAN.current_nom_values, layer caches",
                "static_solver_objects": "reconstructed from config/checkpoint; CvxpyLayer expression is not serialized",
                "formal_trigger": "online frozen evaluate_trigger decision plus exact observation captured after the real solve",
                "trigger_instrumentation": "read-only; no command, planner input, planner state, recovery state, or FSM mutation",
            },
            "extra_metadata": _json_safe(merged_metadata),
        }
        manifest_path = self.directory / "capture_manifest.json"
        manifest_path.write_text(json.dumps(_json_safe(metadata), indent=2, sort_keys=True), encoding="utf-8")
        return {
            "status": "DUMPED",
            "manifest": str(manifest_path.resolve()),
            "numeric_payload": str(numeric_path.resolve()),
            "record_count": len(encoded_records),
            "nonfinite_array_count": int(nonfinite),
        }


def _install_exact_hooks(planner: Any, recorder: _ExactCaptureRecorder) -> None:
    if not recorder.hook_enabled:
        return
    original_pan = planner.pan.forward

    def pan_forward(*args: Any, **kwargs: Any) -> Any:
        result = original_pan(*args, **kwargs)
        recorder.pan_call(args, kwargs, result)
        return result

    planner.pan.forward = pan_forward
    original_stop = planner.pan.stop_criteria

    def stop_criteria(*args: Any, **kwargs: Any) -> Any:
        result = original_stop(*args, **kwargs)
        recorder.stop_call(args, kwargs, result)
        return result

    planner.pan.stop_criteria = stop_criteria
    if planner.pan.dune_layer is not None:
        original_dune = planner.pan.dune_layer.forward

        def dune_forward(*args: Any, **kwargs: Any) -> Any:
            result = original_dune(*args, **kwargs)
            recorder.dune_call(args, kwargs, result)
            return result

        planner.pan.dune_layer.forward = dune_forward
    original_nrmp = planner.pan.nrmp_layer.forward

    def nrmp_forward(*args: Any, **kwargs: Any) -> Any:
        result = original_nrmp(*args, **kwargs)
        recorder.nrmp_call(args, kwargs, result)
        return result

    planner.pan.nrmp_layer.forward = nrmp_forward


def _pack_tensor(value: Any) -> dict[str, Any]:
    return _pack_array(value)


def _set_path(planner: Any, path_matrix: np.ndarray) -> np.ndarray:
    path_matrix = normalize_path_matrix(path_matrix)
    path = [
        path_matrix[:, index:index + 1]
        for index in range(path_matrix.shape[1])
    ]
    planner.set_initial_path(path)
    return path_matrix


def _set_navigation_loop(planner: Any, loop: bool) -> None:
    """Select cyclic versus terminal behavior without changing upstream code."""
    planner.ipath.loop = bool(loop)


def _planner_path_matrix(planner: Any) -> np.ndarray:
    ipath = getattr(planner, "ipath", None)
    initial_path = getattr(ipath, "initial_path", None)
    if not initial_path:
        return np.empty((0, 0), dtype=np.float64)
    return np.hstack([np.asarray(point, dtype=np.float64) for point in initial_path])


def same_terminal_pose(previous: np.ndarray | None,
                       current: np.ndarray | None) -> bool:
    if previous is None or current is None:
        return False
    previous = np.asarray(previous, dtype=np.float64).reshape(-1)
    current = np.asarray(current, dtype=np.float64).reshape(-1)
    if previous.size < 3 or current.size < 3:
        return False
    heading_error = math.atan2(math.sin(previous[2] - current[2]),
                               math.cos(previous[2] - current[2]))
    return (np.isfinite(previous[:3]).all() and np.isfinite(current[:3]).all()
            and np.linalg.norm(previous[:2] - current[:2]) <= 1.0e-3
            and abs(heading_error) <= 1.0e-2)


def _reset_planner_state(planner: Any) -> None:
    """Reset state mutated by a forward while retaining the loaded model."""
    planner.reset()
    planner.cur_vel_array = np.zeros_like(planner.cur_vel_array)
    planner.pan.current_nom_values = [None, None, None, None]
    planner.info = {"stop": False, "arrive": False, "collision": False}


def _build_planner(config_path: str, checkpoint_path: str,
                   pan: dict[str, Any], receding: int | None) -> Any:
    from neupan import neupan

    kwargs: dict[str, Any] = {"pan": dict(pan), "device": "cpu"}
    kwargs["pan"]["dune_checkpoint"] = checkpoint_path
    if receding is not None:
        kwargs["receding"] = int(receding)
    return neupan.init_from_yaml(config_path, **kwargs)


class _WorkerTiming:
    def __init__(self, planner: Any) -> None:
        self.values: dict[str, float] = {}
        self.pan_iterations = 0
        self.pan_early_converged = False
        self._install(planner)

    def reset(self) -> None:
        self.values = {
            "worker_dune_ms": 0.0,
            "worker_nrmp_ms": 0.0,
            "worker_cvxpy_ms": 0.0,
            "worker_pan_ms": 0.0,
        }
        self.pan_iterations = 0
        self.pan_early_converged = False

    def _wrap(self, module: Any, attribute: str, key: str) -> None:
        if module is None:
            return
        original = getattr(module, attribute, None)
        if original is None:
            return

        def timed(*args: Any, **kwargs: Any) -> Any:
            started = time.perf_counter_ns()
            try:
                return original(*args, **kwargs)
            finally:
                self.values[key] = self.values.get(key, 0.0) + (
                    time.perf_counter_ns() - started
                ) / 1_000_000.0

        setattr(module, attribute, timed)

    def _install(self, planner: Any) -> None:
        pan = planner.pan
        original_stop = pan.stop_criteria

        def counted_stop(*args: Any, **kwargs: Any) -> Any:
            self.pan_iterations += 1
            result = original_stop(*args, **kwargs)
            if result:
                self.pan_early_converged = True
            return result

        pan.stop_criteria = counted_stop
        self._wrap(pan, "forward", "worker_pan_ms")
        self._wrap(pan.dune_layer, "forward", "worker_dune_ms")
        self._wrap(pan.nrmp_layer, "forward", "worker_nrmp_ms")
        self._wrap(pan.nrmp_layer.nrmp_layer, "forward", "worker_cvxpy_ms")
        self.reset()


def _pack_planner_result(action: Any, info: dict[str, Any]) -> dict[str, Any]:
    def stack_list(values: Any) -> np.ndarray:
        if not values:
            return np.empty((0, 0), dtype=np.float64)
        return np.hstack([np.asarray(value) for value in values])

    return {
        "action": _pack_tensor(action),
        "ref_state": _pack_tensor(stack_list(info.get("ref_state_list", []))),
        "ref_speed": _pack_tensor(info.get("ref_speed_tensor")),
        "opt_state": _pack_tensor(stack_list(info.get("opt_state_list", []))),
        "optimized_velocity": _pack_tensor(info.get("vel_tensor")),
        "distance": _pack_tensor(info.get("distance_tensor")),
    }


def _planner_semantics(planner: Any) -> dict[str, Any]:
    """Expose numeric path state for offline safety attribution only."""
    ipath = getattr(planner, "ipath", None)
    initial_path = getattr(ipath, "initial_path", None)
    if initial_path:
        path = np.hstack([np.asarray(point) for point in initial_path])
    else:
        path = np.empty((0, 0), dtype=np.float64)
    dune = getattr(getattr(planner, "pan", None), "dune_layer", None)
    dune_min_distance = getattr(dune, "min_distance", None)
    if hasattr(dune_min_distance, "detach"):
        dune_min_distance = dune_min_distance.detach().cpu().numpy()
    dune_min_distance = np.asarray(dune_min_distance) if dune_min_distance is not None else np.empty(0)
    return {
        "initial_path": _pack_array(path),
        "point_index": int(getattr(ipath, "point_index", -1)),
        "curve_index": int(getattr(ipath, "curve_index", -1)),
        "curve_length": int(len(getattr(ipath, "cur_curve", []))) if ipath is not None and getattr(ipath, "curve_list", None) else 0,
        "curve_number": int(getattr(ipath, "curve_number", 0)) if ipath is not None else 0,
        "dune_min_distance_m": (
            float(dune_min_distance.reshape(-1)[0])
            if dune_min_distance.size == 1 and np.isfinite(dune_min_distance).all()
            else None
        ),
        "dune_obstacle_points_count": int(
            getattr(getattr(dune, "obstacle_points", None), "shape", (0, 0))[1]
            if dune is not None else 0
        ),
    }


def _finite_packed(records: dict[str, dict[str, Any]]) -> bool:
    return all(bool(np.isfinite(_unpack_array(record)).all()) for record in records.values())



try:  # optional read-only diagnostic; its absence must never break the worker
    from .ecos_failure_sampler import install_from_env as _install_ecos_sampler_raw
except Exception:  # pragma: no cover - defensive
    _install_ecos_sampler_raw = None


def _install_ecos_sampler():
    """Install the ECOS sampler when COGNAV_ECOS_SAMPLER_DIR is set."""
    if _install_ecos_sampler_raw is None:
        return None
    try:
        return _install_ecos_sampler_raw()
    except Exception:
        return None


def _worker_main(conn: Any, spec: dict[str, Any]) -> None:
    """Spawn target; all NeuPAN imports and planner objects stay here."""
    try:
        worker_generation = int(spec.get("worker_generation", 0))
        fault_mode = str(spec.get("fault_mode", ""))
        fault_request_id = int(spec.get("fault_request_id", -1))
        fault_sleep_sec = float(spec.get("fault_sleep_sec", 0.0))
        fault_injected = False
        policy = _set_torch_policy(int(spec["torch_intraop"]), int(spec["torch_interop"]))
        planner = _build_planner(
            str(spec["config_path"]), str(spec["checkpoint_path"]),
            dict(spec["pan"]), spec.get("receding"),
        )
        loop = bool(spec.get("navigation_loop", True))
        active_gear = validate_gear(spec.get("active_gear", DEFAULT_GEAR))
        _set_navigation_loop(planner, loop)
        _set_path(planner, _unpack_array(spec["initial_path"]))
        timing = _WorkerTiming(planner)
        ecos_sampler = _install_ecos_sampler()
        if ecos_sampler is not None:
            ecos_sampler.set_phase("warmup")
        import torch

        warmup_rows = []
        warmup_state = _unpack_array(spec["warmup_state"])
        warmup_points = _unpack_array(spec["warmup_points"])
        if warmup_points.size == 0:
            warmup_points = None
        for sequence in range(int(spec["warmup_count"])):
            started_wall = time.perf_counter_ns()
            started_cpu = time.process_time_ns()
            with torch.set_grad_enabled(True):
                planner(warmup_state, warmup_points)
            warmup_rows.append({
                "sequence": sequence,
                "wall_ms": (time.perf_counter_ns() - started_wall) / 1_000_000.0,
                "cpu_ms": (time.process_time_ns() - started_cpu) / 1_000_000.0,
            })
        if ecos_sampler is not None:
            ecos_sampler.set_phase("online")
        _reset_planner_state(planner)
        recorder = _ExactCaptureRecorder(
            str(spec.get("exact_capture_dir", "")),
            int(spec.get("exact_capture_max_records", 512)),
            dict(spec.get("capture_metadata") or {}),
        )
        recorder.identity = _exact_config_identity(
            planner,
            str(spec["config_path"]),
            str(spec["checkpoint_path"]),
            dict(spec["pan"]),
            spec.get("receding"),
            policy,
        )
        _install_exact_hooks(planner, recorder)
        conn.send_bytes(pickle.dumps({
            "type": "ready",
            "status": "READY",
            "pid": os.getpid(),
            "worker_generation": worker_generation,
            "start_method": "spawn",
            "worker_inventory": _worker_inventory(),
            "torch_policy": policy,
            "warmup_count": len(warmup_rows),
            "warmup": warmup_rows,
            "planner_state_reset_after_warmup": True,
            "nested_solver_process": False,
            "exact_capture_enabled": bool(recorder.hook_enabled),
            "exact_config_identity": _json_safe(recorder.identity),
        }, protocol=pickle.HIGHEST_PROTOCOL))

        path_version = int(spec.get("initial_path_version", 0))
        goal_version = -1
        active_goal_pose = _planner_path_matrix(planner)[:3, -1:].copy()
        active_local_reference_id = 0
        sequence = 0
        while True:
            raw_request = conn.recv_bytes()
            worker_request_received_monotonic = time.monotonic()
            decode_started = time.perf_counter_ns()
            request = pickle.loads(raw_request)
            if request.get("type") == "shutdown":
                recorder.dump({
                    "reason": "shutdown",
                    "worker_generation": worker_generation,
                    "worker_pid": os.getpid(),
                    "worker_sequence": sequence,
                })
                return
            if request.get("type") == "dump_exact_capture":
                result = recorder.dump({
                    "reason": "explicit_request",
                    "worker_generation": worker_generation,
                    "worker_pid": os.getpid(),
                    "worker_sequence": sequence,
                    **dict(request.get("metadata") or {}),
                })
                conn.send_bytes(pickle.dumps({
                    "type": "exact_capture_dump",
                    **result,
                }, protocol=pickle.HIGHEST_PROTOCOL))
                continue
            request_id = int(request["request_id"])
            if ecos_sampler is not None:
                ecos_sampler.set_context(
                    request_id=request_id,
                    worker_generation=worker_generation,
                )
            loop = bool(request.get("navigation_loop", loop))
            active_gear = validate_gear(request.get("active_gear", active_gear))
            _set_navigation_loop(planner, loop)
            if (
                fault_mode == "sleep_once"
                and request_id == fault_request_id
                and not fault_injected
            ):
                fault_injected = True
                time.sleep(max(0.0, fault_sleep_sec))
            if (
                fault_mode == "crash_once"
                and request_id == fault_request_id
                and not fault_injected
            ):
                fault_injected = True
                os._exit(42)
            requested_path_version = int(request.get("path_version", path_version))
            requested_goal_version = int(request.get("goal_version", -1))
            requested_goal = (_unpack_array(request["goal"])
                              if request.get("goal") is not None else np.empty((0,)))
            next_goal_pose = (requested_goal if requested_goal.size >= 3
                              else normalize_path_matrix(_unpack_array(request["path"]))[:3, -1:])
            same_goal = same_terminal_pose(active_goal_pose, next_goal_pose)
            if requested_path_version > path_version:
                next_path = normalize_path_matrix(_unpack_array(request["path"]))
                preserve_bypass = (same_goal and request.get("bypass_mode", False)
                                   and getattr(planner, "bypass_guide", None) is not None)
                if preserve_bypass:
                    planner.ipath.set_initial_path([
                        next_path[:, index:index + 1]
                        for index in range(next_path.shape[1])
                    ])
                    planner.ipath.arrive_flag = False
                    planner.cur_vel_array = np.zeros_like(planner.cur_vel_array)
                else:
                    _set_path(planner, next_path)
                    planner.reset()
                planner.pan.current_nom_values = [None, None, None, None]
                path_version = requested_path_version
                active_local_reference_id = 0
            elif not same_goal:
                planner.reset()
                planner.pan.current_nom_values = [None, None, None, None]
            active_goal_pose = np.asarray(next_goal_pose, dtype=np.float64).reshape(3, 1).copy()
            local_reference = request.get("local_reference_path")
            local_reference_id = int(request.get("local_reference_id", 0))
            if local_reference is not None and local_reference_id > 0:
                if active_local_reference_id != local_reference_id:
                    _set_path(planner, _unpack_array(local_reference))
                    planner.reset()
                    planner.pan.current_nom_values = [None, None, None, None]
                    active_local_reference_id = local_reference_id
            elif active_local_reference_id:
                _set_path(planner, _unpack_array(request["path"]))
                planner.reset()
                planner.pan.current_nom_values = [None, None, None, None]
                active_local_reference_id = 0
            if requested_goal_version > goal_version:
                # The endpoint is envelope metadata only. The corresponding
                # SMAC path was already supplied above; regenerating a direct
                # state-to-goal path here discards its obstacle-safe bends.
                goal_version = requested_goal_version

            if request.get("restore_planner_state") is not None:
                _restore_planner_state(planner, request["restore_planner_state"])

            state = _unpack_array(request["state"])
            points = _unpack_array(request["obstacle_points"])
            if points.size == 0:
                points = None
            speed_record = request.get("reference_speed_profile")
            speed_profile = _unpack_array(speed_record) if speed_record is not None else None
            measured_record = request.get("measured_velocity")
            measured_velocity = _unpack_array(measured_record) if measured_record is not None else None
            decode_finished = time.perf_counter_ns()
            timing.reset()
            core_wall_started = time.perf_counter_ns()
            core_cpu_started = time.process_time_ns()
            core_started_monotonic = time.monotonic()
            status = "return-success"
            exception = None
            packed = {}
            action = None
            info = getattr(planner, "info", {})
            recorder.begin(request, planner, state, points)
            try:
                with torch.set_grad_enabled(True):
                    action, info = planner(
                        state, points,
                        reference_speed_profile=speed_profile,
                        measured_velocity=measured_velocity,
                        bypass_mode=bool(request.get("bypass_mode", False)),
                        bypass_route_clear=bool(request.get("bypass_route_clear", False)),
                    )
            except Exception as exc:
                status = "exception"
                exception = f"{type(exc).__name__}: {exc}"
            core_wall_finished = time.perf_counter_ns()
            core_cpu_finished = time.process_time_ns()
            core_finished_monotonic = time.monotonic()
            result_pack_started = time.perf_counter_ns()
            if action is not None:
                packed = _pack_planner_result(action, info)
            result_pack_finished = time.perf_counter_ns()
            finite = _finite_packed(packed) if packed else False
            dune_points = getattr(planner.pan.dune_layer, "obstacle_points", None)
            nrmp_points = getattr(planner.pan.nrmp_layer, "obstacle_points", None)
            response = {
                "type": "response",
                "request_id": request_id,
                "worker_generation": worker_generation,
                "worker_sequence": sequence,
                "cycle_id": int(request.get("cycle_id", -1)),
                "scan_counter": int(request.get("scan_counter", -1)),
                "scan_stamp_sec": float(request.get("scan_stamp_sec", -1.0)),
                "status": status,
                "exception": exception,
                "finite": finite,
                "outputs": packed,
                "info": {
                    "stop": bool(getattr(planner, "info", {}).get("stop", False)),
                    "arrive": bool(getattr(planner, "info", {}).get("arrive", False)),
                    "bypass_valid": bool(getattr(planner, "info", {}).get("bypass_valid", False)),
                    "bypass_side": int(getattr(planner, "info", {}).get("bypass_side", 0)),
                    "bypass_phase": str(getattr(planner, "info", {}).get("bypass_phase", "none")),
                    "bypass_clearance_m": getattr(planner, "info", {}).get("bypass_clearance_m"),
                    "bypass_solver_failures": int(getattr(planner, "info", {}).get("bypass_solver_failures", 0)),
                    "bypass_candidates": getattr(planner, "info", {}).get("bypass_candidates", []),
                    "bypass_rebuilds": int(getattr(planner, "info", {}).get("bypass_rebuilds", 0)),
                    "bypass_recovery_exhausted": bool(getattr(planner, "info", {}).get("bypass_recovery_exhausted", False)),
                    "stop_reason": str(getattr(planner, "info", {}).get("stop_reason", "none")),
                },
                "semantic": _planner_semantics(planner),
                "worker_pid": os.getpid(),
                "path_version": path_version,
                "goal_version": goal_version,
                "navigation_loop": bool(getattr(planner.ipath, "loop", loop)),
                "active_gear": active_gear,
                "timings": {
                    "worker_decode_input_ms": (decode_finished - decode_started) / 1_000_000.0,
                    "worker_core_total_ms": (core_wall_finished - core_wall_started) / 1_000_000.0,
                    "worker_core_cpu_ms": (core_cpu_finished - core_cpu_started) / 1_000_000.0,
                    "worker_core_start_monotonic": core_started_monotonic,
                    "worker_core_end_monotonic": core_finished_monotonic,
                    "worker_request_received_monotonic": worker_request_received_monotonic,
                    "worker_solve_start_monotonic": core_started_monotonic,
                    "worker_solve_end_monotonic": core_finished_monotonic,
                    "worker_result_pack_ms": (result_pack_finished - result_pack_started) / 1_000_000.0,
                    **timing.values,
                    "worker_pan_iterations": float(timing.pan_iterations),
                    "worker_pan_early_converged": float(timing.pan_early_converged),
                    "worker_dune_effective_points_count": float(
                        getattr(dune_points, "shape", (0, 0))[1] if dune_points is not None else 0
                    ),
                    "worker_nrmp_effective_points_count": float(
                        getattr(nrmp_points, "shape", (0, 0))[1] if nrmp_points is not None else 0
                    ),
                },
            }
            recorder.finish(planner, action, info, packed, response)
            if fault_mode == "malformed_once" and request_id == fault_request_id and not fault_injected:
                fault_injected = True
                conn.send_bytes(b"N1.16 malformed response")
                return
            if fault_mode == "wrong_request_id_once" and request_id == fault_request_id and not fault_injected:
                fault_injected = True
                response["request_id"] = request_id + 1000000
            if fault_mode == "stale_response_once" and request_id == fault_request_id and not fault_injected:
                fault_injected = True
                response["cycle_id"] = int(request.get("cycle_id", -1)) - 1
                response["scan_counter"] = int(request.get("scan_counter", -1)) - 1
            if fault_mode == "nan_inf_once" and request_id == fault_request_id and not fault_injected:
                fault_injected = True
                response["outputs"]["action"] = _pack_array(
                    np.asarray([np.nan, np.inf], dtype=np.float64)
                )
                response["finite"] = False
            send_started = time.perf_counter_ns()
            response["worker_response_send_started_monotonic"] = time.monotonic()
            raw_response = pickle.dumps(response, protocol=pickle.HIGHEST_PROTOCOL)
            serialized_finished = time.perf_counter_ns()
            response["timings"]["worker_result_serialization_ms"] = (
                serialized_finished - send_started
            ) / 1_000_000.0
            response["timings"]["worker_response_sent_monotonic"] = time.monotonic()
            raw_response = pickle.dumps(response, protocol=pickle.HIGHEST_PROTOCOL)
            actual_send_started = time.perf_counter_ns()
            conn.send_bytes(raw_response)
            response["timings"]["worker_response_send_ms"] = (
                time.perf_counter_ns() - actual_send_started
            ) / 1_000_000.0
            sequence += 1
    except Exception as exc:
        try:
            conn.send_bytes(pickle.dumps({
                "type": "worker_error",
                "status": "worker-error",
            "worker_pid": os.getpid(),
            "worker_generation": worker_generation,
                "exception": f"{type(exc).__name__}: {exc}",
            }, protocol=pickle.HIGHEST_PROTOCOL))
        except Exception:
            pass
        raise
    finally:
        conn.close()


def _stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0, "mean_ms": None, "median_ms": None, "p95_ms": None,
            "p99_ms": None, "max_ms": None, "over_20_ms": 0,
            "over_50_ms": 0, "over_100_ms": 0, "over_500_ms": 0,
            "over_1000_ms": 0,
        }
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "mean_ms": float(np.mean(array)),
        "median_ms": float(np.percentile(array, 50)),
        "p95_ms": float(np.percentile(array, 95)),
        "p99_ms": float(np.percentile(array, 99)),
        "max_ms": float(np.max(array)),
        "over_20_ms": int(np.sum(array > 20.0)),
        "over_50_ms": int(np.sum(array > 50.0)),
        "over_100_ms": int(np.sum(array > 100.0)),
        "over_500_ms": int(np.sum(array >= 500.0)),
        "over_1000_ms": int(np.sum(array >= 1000.0)),
    }


class WholeCoreClient:
    """Synchronous numeric-only, single-flight client for one core worker."""

    def __init__(self, output_dir: str, config_path: str, checkpoint_path: str,
                 pan: dict[str, Any], receding: int | None,
                 torch_intraop: int = 1, torch_interop: int = 24,
                 warmup_count: int = 30,
                 timeout_sec: float = DEFAULT_TIMEOUT_SEC,
                 exact_capture_dir: str = "",
                 exact_capture_max_records: int = 512,
                 capture_metadata: dict[str, Any] | None = None) -> None:
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
        self.context = mp.get_context("spawn")
        self.parent_conn = None
        self.process = None
        self.ready: dict[str, Any] = {}
        self.records: list[dict[str, Any]] = []
        self.samples: list[dict[str, Any]] = []
        self.request_id = 0
        self.requests_sent = 0
        self.requests_completed = 0
        self.worker_failures = 0
        self.queue_depth_max = 0
        self.scan_accounting = {
            "scans_received": 0,
            "unique_processed": 0,
            "obsolete_dropped": 0,
            "duplicate": 0,
        }
        self.main_inventory_before = _main_inventory()
        self._closed = False

    def _start(self, initial_path: np.ndarray, state: np.ndarray,
               points: np.ndarray, path_version: int,
               navigation_loop_mode: str = "cyclic",
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
            "exact_capture_dir": self.exact_capture_dir,
            "exact_capture_max_records": self.exact_capture_max_records,
            "capture_metadata": _json_safe(self.capture_metadata),
        }
        self.process = self.context.Process(
            target=_worker_main,
            args=(child_conn, spec),
            name="r1_n1_15_whole_core",
            daemon=True,
        )
        started = time.perf_counter_ns()
        self.process.start()
        child_conn.close()
        if not self.parent_conn.poll(self.timeout_sec * 6.0):
            self.process.join(timeout=1.0)
            raise RuntimeError("N1.15 whole-core worker did not send READY")
        self.ready = pickle.loads(self.parent_conn.recv_bytes())
        self.startup_ms = (time.perf_counter_ns() - started) / 1_000_000.0
        if self.ready.get("status") != "READY":
            raise RuntimeError(f"N1.15 worker startup failed: {self.ready}")

    def solve(self, state: np.ndarray, points: np.ndarray,
              path: np.ndarray, path_version: int,
              scan_counter: int, scan_stamp_sec: float,
              goal: np.ndarray | None = None, goal_version: int = -1,
              cycle_id: int = -1,
              navigation_loop_mode: str = "cyclic",
              active_gear: int = DEFAULT_GEAR,
              restore_planner_state: dict[str, Any] | None = None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        if self.process is None:
            self._start(path, state, points, path_version, navigation_loop_mode, active_gear)
        if self._closed or self.process is None or not self.process.is_alive():
            self.worker_failures += 1
            raise RuntimeError("N1.15 whole-core worker is not alive; no fallback is permitted")
        self.request_id += 1
        request_id = self.request_id
        prep_started = time.perf_counter_ns()
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
        }
        prep_finished = time.perf_counter_ns()
        request_serial_started = time.perf_counter_ns()
        request["main_send_started_monotonic"] = time.monotonic()
        raw_request = pickle.dumps(request, protocol=pickle.HIGHEST_PROTOCOL)
        request_serial_finished = time.perf_counter_ns()
        send_started = time.perf_counter_ns()
        self.parent_conn.send_bytes(raw_request)
        send_finished = time.perf_counter_ns()
        self.requests_sent += 1
        self.queue_depth_max = max(self.queue_depth_max, 1)
        if not self.parent_conn.poll(self.timeout_sec):
            self.worker_failures += 1
            raise TimeoutError(f"N1.15 worker timeout request_id={request_id}")
        receive_started = time.perf_counter_ns()
        raw_response = self.parent_conn.recv_bytes()
        receive_finished = time.perf_counter_ns()
        response = pickle.loads(raw_response)
        if int(response.get("request_id", -1)) != request_id:
            self.worker_failures += 1
            raise RuntimeError(f"N1.15 response id mismatch: {response}")
        reconstruction_started = time.perf_counter_ns()
        outputs = {
            key: _unpack_array(value) for key, value in response.get("outputs", {}).items()
        }
        reconstruction_finished = time.perf_counter_ns()
        worker_timings = dict(response.get("timings", {}))
        timings = {
            **worker_timings,
            "main_input_extraction_ms": 0.0,
            "main_request_prep_ms": (prep_finished - prep_started) / 1_000_000.0,
            "main_request_serialization_ms": (request_serial_finished - request_serial_started) / 1_000_000.0,
            "main_request_send_ms": (send_finished - send_started) / 1_000_000.0,
            "main_wait_ms": (receive_started - send_finished) / 1_000_000.0,
            "main_response_receive_ms": (receive_finished - receive_started) / 1_000_000.0,
            "main_reconstruction_ms": (reconstruction_finished - reconstruction_started) / 1_000_000.0,
            "main_ipc_roundtrip_ms": (receive_finished - send_started) / 1_000_000.0,
            "main_full_call_ms": (reconstruction_finished - prep_started) / 1_000_000.0,
        }
        record = {
            "request_id": request_id,
            "cycle_id": int(cycle_id),
            "scan_counter": int(scan_counter),
            "scan_stamp_sec": float(scan_stamp_sec),
            "path_version": int(path_version),
            "goal_version": int(goal_version),
            "navigation_loop": bool(response.get("navigation_loop", navigation_loop_mode == "cyclic")),
            "active_gear": int(response.get("active_gear", active_gear)),
            "worker_pid": response.get("worker_pid"),
            "worker_sequence": response.get("worker_sequence"),
            "status": response.get("status"),
            "exception": response.get("exception"),
            "finite": response.get("finite"),
            "outputs": {
                key: {"shape": list(value.shape), "dtype": str(value.dtype)}
                for key, value in outputs.items()
            },
            "info": response.get("info", {}),
            "semantic": response.get("semantic", {}),
            "timings": timings,
        }
        self.records.append(record)
        self.requests_completed += 1
        if len(self.samples) < 3:
            self.samples.append({"request": record, "outputs": outputs})
        if response.get("status") != "return-success" or not response.get("finite"):
            self.worker_failures += 1
            raise RuntimeError(
                f"N1.15 whole-core failure request_id={request_id}: {response.get('exception')}"
            )
        return outputs, record

    def startup(self, initial_path: np.ndarray, state: np.ndarray,
                points: np.ndarray, path_version: int,
                navigation_loop_mode: str = "cyclic",
                active_gear: int = DEFAULT_GEAR) -> None:
        """Start and warm the worker before the first formal cycle."""
        if self.process is None:
            self._start(
                initial_path, state, points, path_version,
                navigation_loop_mode, active_gear,
            )

    def set_scan_accounting(self, **values: int) -> None:
        self.scan_accounting.update({key: int(value) for key, value in values.items()})

    def dump_exact_capture(self, metadata: dict[str, Any] | None = None) -> dict[str, Any]:
        """Flush the worker capture without changing planner state or inputs."""
        if self.process is None or not self.process.is_alive() or self.parent_conn is None:
            return {"status": "UNAVAILABLE"}
        self.parent_conn.send_bytes(pickle.dumps({
            "type": "dump_exact_capture",
            "metadata": dict(metadata or {}),
        }, protocol=pickle.HIGHEST_PROTOCOL))
        if not self.parent_conn.poll(self.timeout_sec):
            return {"status": "TIMEOUT"}
        return pickle.loads(self.parent_conn.recv_bytes())

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self.process is not None and self.process.is_alive():
                self.parent_conn.send_bytes(pickle.dumps({
                    "type": "shutdown",
                    "metadata": self.capture_metadata,
                }))
                self.process.join(timeout=5.0)
        finally:
            if self.process is not None and self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2.0)
            if self.parent_conn is not None:
                self.parent_conn.close()

    def finalize(self) -> None:
        if getattr(self, "_finalized", False):
            return
        self._finalized = True
        self.close()
        values: dict[str, list[float]] = {}
        for record in self.records:
            for key, value in record["timings"].items():
                if isinstance(value, (int, float)) and key.endswith("_ms"):
                    values.setdefault(key, []).append(float(value))
        summary = {
            "schema": "r1_n1_15_whole_core_v1",
            "start_method": "spawn",
            "single_flight": True,
            "nested_solver_process": False,
            "startup_ms": getattr(self, "startup_ms", None),
            "ready": self.ready,
            "main_inventory_before": self.main_inventory_before,
            "main_inventory_after": _main_inventory(),
            "requests_sent": self.requests_sent,
            "requests_completed": self.requests_completed,
            "worker_failures": self.worker_failures,
            "queue_depth_max": self.queue_depth_max,
            "scan_accounting": self.scan_accounting,
            "timing": {key: _stats(value) for key, value in sorted(values.items())},
            "records": self.records,
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


def _planner_from_args(config: str, checkpoint: str, overrides: dict[str, Any]) -> Any:
    with open(config, "r", encoding="utf-8") as stream:
        import yaml
        yaml_config = yaml.safe_load(stream) or {}
    pan = dict(yaml_config.get("pan") or {})
    pan["dune_checkpoint"] = checkpoint
    receding = None
    for key in ("iter_num", "dune_max_num", "nrmp_max_num"):
        if int(overrides.get(key, 0)) > 0:
            pan[key] = int(overrides[key])
    if float(overrides.get("iter_threshold", -1.0)) >= 0.0:
        pan["iter_threshold"] = float(overrides["iter_threshold"])
    if int(overrides.get("receding", 0)) > 0:
        receding = int(overrides["receding"])
    return _build_planner(config, checkpoint, pan, receding)


def _comparison(main: dict[str, np.ndarray], worker: dict[str, np.ndarray]) -> list[dict[str, Any]]:
    rows = []
    for key in sorted(set(main) | set(worker)):
        left = main.get(key, np.empty((0,), dtype=np.float64))
        right = worker.get(key, np.empty((0,), dtype=np.float64))
        if left.shape != right.shape:
            rows.append({"name": key, "shape_equal": False, "dtype_equal": str(left.dtype) == str(right.dtype), "finite_main": bool(np.isfinite(left).all()), "finite_worker": bool(np.isfinite(right).all()), "max_abs_diff": None, "mean_abs_diff": None})
            continue
        diff = np.abs(left.astype(np.float64) - right.astype(np.float64))
        rows.append({
            "name": key,
            "shape_equal": True,
            "dtype_equal": str(left.dtype) == str(right.dtype),
            "shape_main": list(left.shape),
            "shape_worker": list(right.shape),
            "dtype_main": str(left.dtype),
            "dtype_worker": str(right.dtype),
            "finite_main": bool(np.isfinite(left).all()),
            "finite_worker": bool(np.isfinite(right).all()),
            "max_abs_diff": float(np.max(diff)) if diff.size else 0.0,
            "mean_abs_diff": float(np.mean(diff)) if diff.size else 0.0,
        })
    return rows


def run_equivalence(args: argparse.Namespace) -> None:
    import torch

    torch.set_num_threads(args.torch_intraop)
    torch.set_num_interop_threads(args.torch_interop)
    snapshot_paths = sorted({
        path for directory in args.snapshot_dirs
        for path in Path(directory).glob("**/*_high_level.npz")
    })
    if len(snapshot_paths) < args.cycles:
        raise RuntimeError(f"need {args.cycles} high-level real-cycle snapshots, found {len(snapshot_paths)}")
    snapshots = [np.load(path, allow_pickle=False) for path in snapshot_paths[:args.cycles]]
    initial_path = snapshots[0]["initial_path"].astype(np.float64)
    if any(not np.array_equal(initial_path, snapshot["initial_path"]) for snapshot in snapshots):
        raise RuntimeError("stateful equivalence inputs do not share one initial path")
    config = str(Path(args.config).resolve())
    checkpoint = str(Path(args.checkpoint).resolve())
    overrides = {
        "receding": args.receding, "iter_num": args.iter_num,
        "dune_max_num": args.dune_max_num, "nrmp_max_num": args.nrmp_max_num,
        "iter_threshold": args.iter_threshold,
    }
    planner = _planner_from_args(config, checkpoint, overrides)
    _set_path(planner, initial_path)
    _reset_planner_state(planner)
    pan = dict((__import__("yaml").safe_load(Path(config).read_text()) or {}).get("pan") or {})
    pan.update({key: value for key, value in {
        "iter_num": args.iter_num, "dune_max_num": args.dune_max_num,
        "nrmp_max_num": args.nrmp_max_num, "iter_threshold": args.iter_threshold,
    }.items() if (key == "iter_threshold" and value >= 0) or (key != "iter_threshold" and value > 0)})
    pan["dune_checkpoint"] = checkpoint
    client = WholeCoreClient(
        args.output, config, checkpoint, pan,
        args.receding if args.receding > 0 else None,
        args.torch_intraop, args.torch_interop, args.warmup_count,
    )
    cycle_rows = []
    try:
        for cycle_id, snapshot in enumerate(snapshots, 1):
            state = snapshot["robot_state"].astype(np.float64)
            points = snapshot["obstacle_points"].astype(np.float64)
            with torch.set_grad_enabled(True):
                action, info = planner(state, points)
            main_outputs = _pack_planner_result(action, info)
            worker_outputs, record = client.solve(
                state, points, initial_path, 1, cycle_id, float(cycle_id), cycle_id=cycle_id
            )
            main_arrays = {key: _unpack_array(value) for key, value in main_outputs.items()}
            rows = _comparison(main_arrays, worker_outputs)
            main_info = {
                "stop": bool(info.get("stop", False)),
                "arrive": bool(info.get("arrive", False)),
            }
            worker_info = record.get("info", {})
            cycle_rows.append({
                "cycle_id": cycle_id,
                "scan_counter": cycle_id,
                "info_equal": main_info == {
                    "stop": bool(worker_info.get("stop", False)),
                    "arrive": bool(worker_info.get("arrive", False)),
                },
                "pass": main_info == {
                    "stop": bool(worker_info.get("stop", False)),
                    "arrive": bool(worker_info.get("arrive", False)),
                } and all(
                    row["shape_equal"] and row["dtype_equal"]
                    and row["finite_main"] and row["finite_worker"]
                    and row["max_abs_diff"] <= args.abs_tolerance
                    for row in rows
                ),
                "outputs": rows,
                "worker_record": record,
            })
    finally:
        client.finalize()
    result = {
        "schema": "r1_n1_15_stateful_equivalence_v1",
        "start_method": "spawn",
        "cycles": len(cycle_rows),
        "snapshot_paths": [str(path.resolve()) for path in snapshot_paths[:args.cycles]],
        "same_initial_path": True,
        "absolute_tolerance": args.abs_tolerance,
        "cycle_results": cycle_rows,
        "pass": len(cycle_rows) >= 20 and all(row["pass"] for row in cycle_rows),
        "worker_summary": json.loads((Path(args.output) / "summary.json").read_text()),
    }
    Path(args.output).mkdir(parents=True, exist_ok=True)
    (Path(args.output) / "equivalence.json").write_text(
        json.dumps(_json_safe(result), indent=2, sort_keys=True), encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["equivalence"], required=True)
    parser.add_argument("--snapshot-dirs", nargs="+", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--warmup-count", type=int, default=30)
    parser.add_argument("--torch-intraop", type=int, default=1)
    parser.add_argument("--torch-interop", type=int, default=24)
    parser.add_argument("--abs-tolerance", type=float, default=1e-5)
    parser.add_argument("--receding", type=int, default=0)
    parser.add_argument("--iter-num", type=int, default=0)
    parser.add_argument("--dune-max-num", type=int, default=0)
    parser.add_argument("--nrmp-max-num", type=int, default=0)
    parser.add_argument("--iter-threshold", type=float, default=-1.0)
    args = parser.parse_args()
    run_equivalence(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
