#!/usr/bin/env python3
"""N2.4B exact-state capture selection and original-state replay.

This tool is deliberately limited to Gate 1.  It restores the planner state
captured immediately before a live solve and calls the same isolated
``WholeCoreClient`` without publishing ROS commands or changing recovery
state.  It never constructs or executes a recovery counterfactual.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any

import numpy as np

from r1_neupan_integration.n1_15_whole_core import (
    WholeCoreClient,
    _exact_decode,
    _json_safe,
)
from r1_neupan_integration.n2_4_recovery import (
    RecoveryObservation,
    evaluate_trigger,
)


SCENES = ("b1_single_table", "b4_center_obstacle", "d1_corner_obstacle")
DEFAULT_TOLERANCE = 1.0e-5


def load_bundle(directory: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest_path = directory / "capture_manifest.json"
    payload_path = directory / "numeric_payload.npz"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    with np.load(payload_path, allow_pickle=False) as payload:
        arrays = {key: np.ascontiguousarray(payload[key]).copy() for key in payload.files}
    records = [_exact_decode(item, arrays) for item in manifest.get("records", [])]
    if not records:
        raise RuntimeError(f"capture has no records: {directory}")
    return manifest, records


def scalar(value: Any, default: float | None = None) -> float | None:
    if value is None:
        return default
    if isinstance(value, np.ndarray):
        if value.size != 1:
            return default
        value = value.reshape(-1)[0]
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def matrix(value: Any, default: tuple[int, ...] = (0, 0)) -> np.ndarray:
    if value is None:
        return np.empty(default, dtype=np.float64)
    array = np.asarray(value)
    return np.ascontiguousarray(array)


def path_xy(record: dict[str, Any]) -> list[tuple[float, float]]:
    path = matrix(record.get("input", {}).get("effective_path"))
    if path.ndim != 2 or path.shape[0] < 2:
        return []
    return [(float(x), float(y)) for x, y in path[:2].T]


def pose(record: dict[str, Any]) -> tuple[float, float, float]:
    state = matrix(record.get("input", {}).get("state"), (3, 1)).reshape(-1)
    if state.size < 3:
        return (math.nan, math.nan, math.nan)
    return (float(state[0]), float(state[1]), float(state[2]))


def point_segment_distance(point: tuple[float, float], left: tuple[float, float], right: tuple[float, float]) -> float:
    dx = right[0] - left[0]
    dy = right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(point[0] - left[0], point[1] - left[1])
    fraction = max(0.0, min(1.0, ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq))
    return math.hypot(point[0] - (left[0] + fraction * dx), point[1] - (left[1] + fraction * dy))


def path_deviation(record: dict[str, Any], path: list[tuple[float, float]]) -> float:
    current = pose(record)
    if len(path) < 2 or not all(math.isfinite(value) for value in current):
        return math.inf
    return min(point_segment_distance((current[0], current[1]), left, right) for left, right in zip(path, path[1:]))


def output_matrix(record: dict[str, Any], name: str) -> np.ndarray:
    return matrix(record.get("original_result", {}).get("outputs", {}).get(name))


def active_index(record: dict[str, Any]) -> int:
    state = record.get("planner_state_post", {}).get("ipath", {})
    return int(state.get("point_index", -1))


def monotonic(record: dict[str, Any], fallback: int) -> float:
    request = record.get("request", {})
    value = scalar(request.get("main_send_started_monotonic"))
    return float(value if value is not None and value >= 0.0 else fallback * 0.02)


def recovery_observation(record: dict[str, Any], path: list[tuple[float, float]], index: int) -> RecoveryObservation:
    result = record.get("original_result", {})
    action = matrix(result.get("action"), (2, 1)).reshape(-1)
    info = result.get("info", {})
    post = record.get("planner_state_post", {})
    dune = post.get("dune") or {}
    reference = output_matrix(record, "ref_state")
    optimized = output_matrix(record, "opt_state")
    return RecoveryObservation(
        cycle_id=int(record.get("request", {}).get("cycle_id", index + 1)),
        monotonic=monotonic(record, index + 1),
        raw_linear=float(action[0]) if action.size >= 1 else math.nan,
        raw_angular=float(action[1]) if action.size >= 2 else math.nan,
        pose=pose(record),
        path=tuple(path),
        active_path_index=active_index(record),
        path_deviation_m=path_deviation(record, path),
        goal_distance_m=(
            math.hypot(pose(record)[0] - path[-1][0], pose(record)[1] - path[-1][1])
            if path and all(math.isfinite(value) for value in pose(record)) else None
        ),
        stop=bool(info.get("stop", False)),
        arrive=bool(info.get("arrive", False)),
        planner_fresh=True,
        path_valid=len(path) >= 2,
        obstacle_clearance_m=scalar(dune.get("min_distance")),
        corridor_margin_m=None,
        reference_points=tuple((float(x), float(y)) for x, y in reference[:2].T) if reference.ndim == 2 and reference.shape[0] >= 2 else (),
        optimized_points=tuple((float(x), float(y)) for x, y in optimized[:2].T) if optimized.ndim == 2 and optimized.shape[0] >= 2 else (),
    )


def select_windows(scene: str, records: list[dict[str, Any]]) -> dict[str, Any]:
    path = path_xy(records[0])
    observations = [recovery_observation(record, path, index) for index, record in enumerate(records)]
    history: list[RecoveryObservation] = []
    trigger_index: int | None = None
    trigger_decision: dict[str, Any] = {}
    decisions: list[dict[str, Any]] = []
    online_capture_present = []
    online_decision_mismatches = []
    for index, observation in enumerate(observations):
        decision = evaluate_trigger(history, observation)
        decisions.append(decision)
        captured = records[index].get("n2_4_trigger")
        has_online_capture = (
            isinstance(captured, dict)
            and captured.get("detector") == "r1_neupan_integration.n2_4_recovery.evaluate_trigger"
            and isinstance(captured.get("decision"), dict)
            and isinstance(captured.get("observation"), dict)
        )
        online_capture_present.append(has_online_capture)
        if has_online_capture and captured.get("decision") != decision:
            online_decision_mismatches.append(index)
        if trigger_index is None and decision["trigger"] and has_online_capture:
            trigger_index = index
            trigger_decision = decision
        history.append(observation)

    negative_indices = [index for index, item in enumerate(observations) if item.raw_linear < 0.0]
    runs: list[list[int]] = []
    for index in negative_indices:
        if not runs or index != runs[-1][-1] + 1:
            runs.append([index])
        else:
            runs[-1].append(index)
    longest_run = max(runs, key=lambda item: (len(item), item[-1]), default=[])
    stop_indices = [index for index, item in enumerate(observations) if item.stop]
    stop_before_index = None
    first_stop_index = None
    if stop_indices:
        first_stop = stop_indices[0]
        first_stop_index = first_stop
        stop_before_index = first_stop - 1 if first_stop > 0 else None

    def window(center: int | None, radius: int = 1) -> list[int]:
        if center is None:
            return []
        return list(range(max(0, center - radius), min(len(records), center + radius + 1)))

    selected: dict[str, Any] = {
        "trigger_index": trigger_index,
        "trigger_cycle_id": None if trigger_index is None else int(observations[trigger_index].cycle_id),
        "trigger_decision": trigger_decision,
        "trigger_window_indices": window(trigger_index, 1),
        "trigger_window_cycle_ids": [int(observations[index].cycle_id) for index in window(trigger_index, 1)],
        "longest_negative_run_indices": longest_run,
        "longest_negative_run_cycle_ids": [int(observations[index].cycle_id) for index in longest_run],
        "longest_negative_run_length": len(longest_run),
        "stop_before_index": stop_before_index,
        "stop_before_cycle_id": None if stop_before_index is None else int(observations[stop_before_index].cycle_id),
        "stop_before_window_indices": window(stop_before_index, 1),
        "formal_trigger_evaluations": decisions,
        "online_formal_capture_present": all(online_capture_present),
        "online_formal_capture_rows": int(sum(online_capture_present)),
        "online_decision_mismatches": online_decision_mismatches,
        "formal_trigger_window_complete": trigger_index is not None and all(
            online_capture_present[index] for index in window(trigger_index, 1)
        ),
        "d1_trigger_pre_stop": (
            scene != "d1_corner_obstacle"
            or (
                trigger_index is not None
                and (first_stop_index is None or trigger_index < first_stop_index)
            )
        ),
    }
    if scene != "d1_corner_obstacle":
        selected["identity_indices"] = window(trigger_index, 1)
    else:
        selected["identity_indices"] = sorted(set(window(trigger_index, 1) + window(stop_before_index, 1) + window(longest_run[-1] if longest_run else None, 1)))
    if not selected["formal_trigger_window_complete"] or selected["online_decision_mismatches"]:
        selected["trigger_index"] = None
        selected["trigger_cycle_id"] = None
        selected["trigger_decision"] = {}
        selected["trigger_window_indices"] = []
        selected["trigger_window_cycle_ids"] = []
        selected["identity_indices"] = []
    if scene == "d1_corner_obstacle" and not selected["d1_trigger_pre_stop"]:
        selected["trigger_index"] = None
        selected["trigger_cycle_id"] = None
        selected["trigger_decision"] = {}
        selected["trigger_window_indices"] = []
        selected["trigger_window_cycle_ids"] = []
        selected["identity_indices"] = []
    return selected


def diff_array(expected: np.ndarray, actual: np.ndarray) -> dict[str, Any]:
    row: dict[str, Any] = {
        "shape_equal": list(expected.shape) == list(actual.shape),
        "dtype_equal": str(expected.dtype) == str(actual.dtype),
        "expected_shape": list(expected.shape),
        "actual_shape": list(actual.shape),
        "expected_dtype": str(expected.dtype),
        "actual_dtype": str(actual.dtype),
        "expected_finite": bool(np.isfinite(expected).all()),
        "actual_finite": bool(np.isfinite(actual).all()),
    }
    if expected.shape != actual.shape:
        row.update({"max_abs_diff": None, "mean_abs_diff": None})
        return row
    delta = np.abs(expected.astype(np.float64) - actual.astype(np.float64))
    row.update({
        "max_abs_diff": float(np.max(delta)) if delta.size else 0.0,
        "mean_abs_diff": float(np.mean(delta)) if delta.size else 0.0,
    })
    return row


def compare_record(live: dict[str, Any], outputs: dict[str, np.ndarray], replay: dict[str, Any], tolerance: float) -> dict[str, Any]:
    live_result = live.get("original_result", {})
    live_outputs = live_result.get("outputs", {})
    output_rows = {}
    pass_numeric = True
    for name in sorted(set(live_outputs) | set(outputs)):
        expected = matrix(live_outputs.get(name))
        actual = matrix(outputs.get(name))
        row = diff_array(expected, actual)
        output_rows[name] = row
        pass_numeric = pass_numeric and row["shape_equal"] and row["dtype_equal"] and row["expected_finite"] and row["actual_finite"] and (row["max_abs_diff"] is not None and row["max_abs_diff"] <= tolerance)
    live_info = live_result.get("info", {})
    replay_info = replay.get("info", {})
    live_semantic = live_result.get("semantic", {})
    replay_semantic = replay.get("semantic", {})
    exact_fields = {
        "stop": (bool(live_info.get("stop", False)), bool(replay_info.get("stop", False))),
        "arrive": (bool(live_info.get("arrive", False)), bool(replay_info.get("arrive", False))),
        "collision": (bool(live_info.get("collision", False)), bool(replay_info.get("collision", False))),
        "finite": (bool(live_result.get("finite", False)), bool(replay.get("finite", False))),
        "point_index": (active_index(live), int(replay_semantic.get("point_index", -1))),
        "curve_index": (int(live.get("planner_state_post", {}).get("ipath", {}).get("curve_index", -1)), int(replay_semantic.get("curve_index", -1))),
        "path_version": (int(live.get("request", {}).get("path_version", -1)), int(replay.get("path_version", -1))),
        "goal_version": (int(live.get("request", {}).get("goal_version", -1)), int(replay.get("goal_version", -1))),
        "navigation_loop": (bool(live.get("request", {}).get("navigation_loop", True)), bool(replay.get("navigation_loop", True))),
        "active_gear": (int(live.get("request", {}).get("active_gear", 1)), int(replay.get("active_gear", 1))),
    }
    exact_pass = all(left == right for left, right in exact_fields.values())
    live_timing = live_result.get("timings", {})
    replay_timing = replay.get("timings", {})
    pan_fields = {
        "worker_pan_iterations": (live_timing.get("worker_pan_iterations"), replay_timing.get("worker_pan_iterations")),
        "worker_pan_early_converged": (live_timing.get("worker_pan_early_converged"), replay_timing.get("worker_pan_early_converged")),
    }
    pan_pass = all(left == right for left, right in pan_fields.values())
    def action_component(source: dict[str, Any], index: int) -> float | None:
        value = matrix(source.get("action"))
        flat = value.reshape(-1)
        if flat.size <= index:
            return None
        return float(flat[index])

    def minimum_output(name: str, source: dict[str, Any]) -> float | None:
        value = matrix(source.get(name))
        if not value.size or not np.isfinite(value).all():
            return None
        return float(np.min(value))

    def scalar_diff(left: float | None, right: float | None) -> dict[str, Any]:
        equal = left is not None and right is not None
        delta = None if not equal else abs(left - right)
        return {
            "live": left,
            "replay": right,
            "max_abs_diff": delta,
            "exact": bool(equal and left == right),
            "near_exact": bool(equal and delta <= tolerance),
        }

    live_distance = minimum_output("distance", live_outputs)
    replay_distance = minimum_output("distance", outputs)
    field_by_field = {
        "raw_v": scalar_diff(action_component(live_outputs, 0), action_component(outputs, 0)),
        "raw_w": scalar_diff(action_component(live_outputs, 1), action_component(outputs, 1)),
        "reference_state": output_rows.get("ref_state", {}),
        "optimized_state": output_rows.get("opt_state", {}),
        "planner_min_distance": scalar_diff(live_distance, replay_distance),
        "pan_iterations": scalar_diff(
            float(live_timing.get("worker_pan_iterations")) if live_timing.get("worker_pan_iterations") is not None else None,
            float(replay_timing.get("worker_pan_iterations")) if replay_timing.get("worker_pan_iterations") is not None else None,
        ),
    }
    numeric_diffs = [
        row.get("max_abs_diff") for row in output_rows.values()
        if row.get("max_abs_diff") is not None
    ]
    return {
        "cycle_id": int(live.get("request", {}).get("cycle_id", -1)),
        "pass": bool(pass_numeric and exact_pass and pan_pass and all(
            item.get("near_exact", True) for item in field_by_field.values()
        )),
        "outputs": output_rows,
        "field_by_field": field_by_field,
        "max_abs_diff": float(max(numeric_diffs, default=0.0)),
        "exact_fields": exact_fields,
        "pan_fields": pan_fields,
        "replay_timings": {
            "worker_core_total_ms": replay_timing.get("worker_core_total_ms"),
            "worker_pan_ms": replay_timing.get("worker_pan_ms"),
        },
    }


def replay_scene(scene: str, manifest: dict[str, Any], records: list[dict[str, Any]], selection: dict[str, Any], args: argparse.Namespace, output: Path) -> dict[str, Any]:
    identity = manifest.get("config_identity", {})
    selected_indices = selection.get("identity_indices", [])
    trigger_captured = bool(
        selection.get("trigger_index") is not None
        and selection.get("formal_trigger_window_complete")
        and not selection.get("online_decision_mismatches")
        and selection.get("d1_trigger_pre_stop", True)
    )
    if not selected_indices:
        return {
            "scene": scene,
            "gate": "GATE_1",
            "pass": False,
            "classification": "CAPTURE/REPLAY STATE INCOMPLETE",
            "reason": "formal N2.4 trigger window was not captured",
            "formal_trigger_captured": trigger_captured,
            "selection": selection,
        }
    effective_yaml = identity.get("effective_yaml", {})
    # The capture already records the YAML's effective top-level receding
    # value.  Only provide an override when the source config had none.
    receding = None
    if "receding" not in effective_yaml:
        receding = int(identity.get("pan_runtime", {}).get("receding", 0)) or None
    replay_dir = output / scene / "identity_worker"
    if replay_dir.exists():
        shutil.rmtree(replay_dir)
    client = WholeCoreClient(
        str(replay_dir),
        args.config,
        args.checkpoint,
        dict(effective_yaml.get("pan", {})),
        receding,
        args.torch_intraop,
        args.torch_interop,
        0,
        args.timeout,
    )
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        for selected_index in selected_indices:
            live = records[selected_index]
            request = live["request"]
            try:
                outputs, replay_record = client.solve(
                    matrix(live["input"]["state"]),
                    matrix(live["input"].get("obstacle_points")),
                    matrix(live["input"]["effective_path"]),
                    int(request.get("path_version", 1)),
                    int(request.get("scan_counter", request.get("cycle_id", selected_index + 1))),
                    float(request.get("scan_stamp_sec", -1.0)),
                    goal=matrix(live["input"].get("goal")),
                    goal_version=int(request.get("goal_version", -1)),
                    cycle_id=int(request.get("cycle_id", selected_index + 1)),
                    navigation_loop_mode="cyclic" if request.get("navigation_loop", True) else "terminal",
                    active_gear=int(request.get("active_gear", 1)),
                    restore_planner_state=live.get("planner_state_pre"),
                )
                row = compare_record(live, outputs, replay_record, args.tolerance)
                row["capture_index"] = selected_index
                rows.append(row)
            except Exception as exc:
                errors.append(f"cycle {request.get('cycle_id')}: {type(exc).__name__}: {exc}")
    finally:
        client.finalize()
    return {
        "scene": scene,
        "gate": "GATE_1",
        "pass": trigger_captured and not errors and bool(rows) and all(row["pass"] for row in rows),
        "classification": "PASS" if trigger_captured and not errors and bool(rows) and all(row["pass"] for row in rows) else "CAPTURE/REPLAY STATE INCOMPLETE",
        "reason": None if trigger_captured else "formal N2.4 trigger window was not captured; replay rows are diagnostic only",
        "formal_trigger_captured": trigger_captured,
        "identity_replay_scope": "trigger_window" if trigger_captured else "none",
        "selection": selection,
        "identity": identity,
        "replay_worker_ready": client.ready,
        "config_diff": config_diff(identity, client.ready.get("exact_config_identity", {})),
        "rows": rows,
        "errors": errors,
        "tolerance": args.tolerance,
    }


def config_diff(left: Any, right: Any, prefix: str = "") -> dict[str, Any]:
    if isinstance(left, dict) and isinstance(right, dict):
        result: dict[str, Any] = {}
        for key in sorted(set(left) | set(right)):
            child = config_diff(left.get(key), right.get(key), f"{prefix}.{key}" if prefix else str(key))
            if child:
                result.update(child)
        return result
    if isinstance(left, list) and isinstance(right, list):
        if left != right:
            return {prefix: {"capture": left, "replay": right}}
        return {}
    if left != right:
        return {prefix: {"capture": left, "replay": right}}
    return {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", action="append", required=True, metavar="SCENE=DIR")
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--torch-intraop", type=int, default=1)
    parser.add_argument("--torch-interop", type=int, default=24)
    args = parser.parse_args()

    captures: dict[str, Path] = {}
    for value in args.capture:
        scene, separator, directory = value.partition("=")
        if not separator or scene not in SCENES:
            raise SystemExit(f"--capture must be SCENE=DIR for {SCENES}, got {value!r}")
        captures[scene] = Path(directory).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    results: dict[str, Any] = {}
    for scene in SCENES:
        if scene not in captures:
            results[scene] = {
                "scene": scene,
                "gate": "GATE_1",
                "pass": False,
                "classification": "CAPTURE/REPLAY STATE INCOMPLETE",
                "reason": "capture directory was not supplied",
            }
            continue
        manifest, records = load_bundle(captures[scene])
        selection = select_windows(scene, records)
        results[scene] = replay_scene(scene, manifest, records, selection, args, output)
        (output / scene / "selection.json").parent.mkdir(parents=True, exist_ok=True)
        (output / scene / "selection.json").write_text(json.dumps(_json_safe(selection), indent=2, sort_keys=True), encoding="utf-8")
    gate_pass = all(bool(results[scene].get("pass")) for scene in SCENES)
    report = {
        "schema": "r1_n2_4_exact_state_replay_v1",
        "gate": "GATE_1",
        "pass": gate_pass,
        "classification": "GATE_1_PASS" if gate_pass else "CAPTURE/REPLAY STATE INCOMPLETE",
        "recovery_counterfactual_executed": False,
        "ros_commands_published": False,
        "gazebo_motion_executed": False,
        "scenes": results,
    }
    (output / "exact_replay_report.json").write_text(json.dumps(_json_safe(report), indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(_json_safe({"pass": gate_pass, "classification": report["classification"], "output": str(output)}), sort_keys=True))
    return 0 if gate_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
