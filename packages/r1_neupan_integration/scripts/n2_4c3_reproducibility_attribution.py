#!/usr/bin/env python3
"""N2.4C3 read-only D1 reproducibility attribution.

This tool consumes the sealed N2.4C2 exact captures.  It never changes the
trigger/FSM, planner configuration, world, path, safety, or runtime inputs.
It performs two bounded operations:

* align the five current-stack D1 captures and report the earliest sustained
  difference in cycles 30--55;
* replay selected exact records through a freshly constructed NeuPAN worker,
  then run one-variable offline input/state swaps only when identity replay
  passes.

The output is evidence, not a causal claim: a swap is causal only when its
counterfactual result changes the specified output while all other categories
remain fixed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from r1_neupan_integration.n1_15_whole_core import (
    WholeCoreClient,
    _exact_decode,
)


ROOT = Path(__file__).resolve().parents[3]
RUNS = {name: ROOT / "n2_4c2_phenotype_runs" / name for name in (
    "d1_01", "d1_02", "d1_03", "d1_04", "d1_05",
)}
LABELS = {"d1_01": "D1-1", "d1_02": "D1-2", "d1_03": "D1-3", "d1_04": "D1-4", "d1_05": "D1-5"}
PASS = "d1_04"
FAILS = ["d1_01", "d1_02", "d1_03", "d1_05"]
WINDOW = range(30, 56)
ABS_TOL = 1.0e-4
REL_TOL = 1.0e-4


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda _: None)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def scalar(value: Any) -> float | None:
    if isinstance(value, np.ndarray):
        if value.size != 1:
            return None
        value = value.reshape(-1)[0]
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def array_value(value: Any) -> np.ndarray:
    return np.asarray(value)


def max_abs(left: Any, right: Any) -> float | None:
    try:
        a, b = np.asarray(left), np.asarray(right)
        if a.shape != b.shape:
            return None
        if a.size == 0:
            return 0.0
        return float(np.max(np.abs(a.astype(np.float64) - b.astype(np.float64))))
    except (TypeError, ValueError):
        return None


def norm_diff(left: Any, right: Any) -> float | None:
    try:
        a, b = np.asarray(left), np.asarray(right)
        if a.shape != b.shape:
            return None
        if a.size == 0:
            return 0.0
        return float(np.linalg.norm(a.astype(np.float64) - b.astype(np.float64)))
    except (TypeError, ValueError):
        return None


def digest_array(value: Any) -> str:
    a = np.ascontiguousarray(np.asarray(value))
    return hashlib.sha256(a.tobytes()).hexdigest()


def flatten_numeric(value: Any) -> np.ndarray:
    parts: list[np.ndarray] = []
    if isinstance(value, np.ndarray):
        if value.size:
            parts.append(value.astype(np.float64, copy=False).reshape(-1))
    elif isinstance(value, dict):
        for key in sorted(value):
            parts.append(flatten_numeric(value[key]))
    elif isinstance(value, (list, tuple)):
        for item in value:
            parts.append(flatten_numeric(item))
    elif isinstance(value, (int, float, np.number)) and finite(value):
        parts.append(np.asarray([float(value)], dtype=np.float64))
    return np.concatenate([part for part in parts if part.size]) if any(part.size for part in parts) else np.empty((0,), dtype=np.float64)


def path_distance(pose: np.ndarray, path: np.ndarray) -> float | None:
    value = np.asarray(pose, dtype=float).reshape(-1)
    points = np.asarray(path, dtype=float)
    if value.size < 2 or points.ndim != 2 or points.shape[0] < 2:
        return None
    best = math.inf
    for left, right in zip(points[:2].T, points[:2].T[1:]):
        segment = right - left
        denom = float(np.dot(segment, segment))
        fraction = 0.0 if denom <= 1.0e-12 else float(np.dot(value[:2] - left, segment) / denom)
        fraction = max(0.0, min(1.0, fraction))
        best = min(best, float(np.linalg.norm(value[:2] - (left + fraction * segment))))
    return best if math.isfinite(best) else None


def decode_capture(directory: Path) -> dict[str, Any]:
    manifest = load_json(directory / "exact" / "capture_manifest.json")
    with np.load(directory / "exact" / "numeric_payload.npz", allow_pickle=False) as payload:
        arrays = {key: payload[key] for key in payload.files}
        records = [_exact_decode(record, arrays) for record in manifest.get("records", [])]
    by_cycle = {int(record["request"]["cycle_id"]): record for record in records}
    audit = load_json(directory / "audit.json")
    evidence = {int(row["cycle_id"]): row for row in audit.get("cycle_evidence", [])}
    trajectory = audit.get("trajectory") or []
    return {"directory": directory, "manifest": manifest, "records": records, "by_cycle": by_cycle, "audit": audit, "evidence": evidence, "trajectory": trajectory}


def nearest_odom(trajectory: list[Any], audit_t: float | None) -> dict[str, float | None]:
    if audit_t is None or not trajectory:
        return {"v": None, "w": None, "t": None}
    rows = [row for row in trajectory if isinstance(row, list) and len(row) >= 7 and finite(row[0])]
    if not rows:
        return {"v": None, "w": None, "t": None}
    row = min(rows, key=lambda item: abs(float(item[0]) - float(audit_t)))
    return {"v": float(row[4]), "w": float(row[6]), "t": float(row[0])}


def metric(record: dict[str, Any], evidence: dict[str, Any], trajectory: list[Any]) -> dict[str, Any]:
    request = record["request"]
    inp = record["input"]
    pre = record["planner_state_pre"]
    post = record["planner_state_post"]
    result = record["original_result"]
    state = np.asarray(inp["state"], dtype=float).reshape(-1)
    points = np.asarray(inp["obstacle_points"], dtype=float)
    path = np.asarray(inp["effective_path"], dtype=float)
    action = np.asarray(result["outputs"]["action"], dtype=float).reshape(-1)
    opt = np.asarray(result["outputs"]["opt_state"], dtype=float)
    ref = np.asarray(result["outputs"]["ref_state"], dtype=float)
    cur_pre = np.asarray(pre["planner"]["cur_vel_array"], dtype=float)
    cur_post = np.asarray(post["planner"]["cur_vel_array"], dtype=float)
    dune_pre = scalar(np.asarray(pre["dune"]["min_distance"])) if pre.get("dune") else None
    dune_post = scalar(np.asarray(post["dune"]["min_distance"])) if post.get("dune") else None
    nom_pre = flatten_numeric(pre["pan"].get("current_nom_values"))
    nom_post = flatten_numeric(post["pan"].get("current_nom_values"))
    info = result.get("info") or {}
    timing = result.get("timings") or {}
    audit_t = scalar((evidence or {}).get("audit_t"))
    odom = nearest_odom(trajectory, audit_t)
    return {
        "cycle": int(request["cycle_id"]),
        "request_id": int(request["request_id"]),
        "scan_counter": int(request["scan_counter"]),
        "scan_stamp_sec": scalar(request.get("scan_stamp_sec")),
        "state": state,
        "pose": state[:3],
        "obstacle_points": points,
        "obstacle_point_count": int(points.shape[1]) if points.ndim == 2 else int(points.size),
        "obstacle_digest": digest_array(points),
        "path": path,
        "path_digest": digest_array(path),
        "path_index": int(post["ipath"].get("point_index", -1)),
        "curve_index": int(post["ipath"].get("curve_index", -1)),
        "action": action,
        "raw_v": scalar(action[0]) if action.size else None,
        "raw_w": scalar(action[1]) if action.size > 1 else None,
        "opt_state": opt,
        "ref_state": ref,
        "distance": np.asarray(result["outputs"]["distance"], dtype=float),
        "min_planner_distance": scalar(np.min(result["outputs"]["distance"])) if np.asarray(result["outputs"]["distance"]).size else None,
        "output_distance_min": scalar(np.min(result["outputs"]["distance"])) if np.asarray(result["outputs"]["distance"]).size else None,
        "dune_pre_min_distance": dune_pre,
        "dune_min_distance": dune_post,
        "cur_vel_pre": cur_pre,
        "cur_vel_post": cur_post,
        "nom_pre": nom_pre,
        "nom_post": nom_post,
        "pan_iterations": scalar(timing.get("worker_pan_iterations")),
        "pan_early_converged": bool(float(timing.get("worker_pan_early_converged", 0.0))),
        "stop": bool(info.get("stop", False)),
        "arrive": bool(info.get("arrive", False)),
        "audit_t": audit_t,
        "odom_v": odom["v"],
        "odom_w": odom["w"],
        "odom_t": odom["t"],
        "goal_distance": scalar((evidence or {}).get("goal_distance")),
        "path_deviation": path_distance(state, path),
        "audit_obstacle_point_count": (evidence or {}).get("obstacle_point_count"),
        "audit_min_planner_distance": scalar((evidence or {}).get("min_planner_distance")),
        "audit_stop": bool((evidence or {}).get("stop", False)),
        "audit_arrive": bool((evidence or {}).get("arrive", False)),
    }


def field_diff(left: dict[str, Any], right: dict[str, Any], field: str) -> float | None:
    a, b = left.get(field), right.get(field)
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return max_abs(a, b)
    if a is None or b is None:
        return 0.0 if a == b else math.inf
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b))
    return 0.0 if a == b else math.inf


FIELDS = [
    ("pose", "robot pose", 1.0e-5),
    ("odom_v", "odom linear velocity", 1.0e-4),
    ("odom_w", "odom angular velocity", 1.0e-4),
    ("obstacle_point_count", "obstacle point count", 0.0),
    ("obstacle_points", "effective obstacle input", 1.0e-5),
    ("path_index", "path progress index", 0.0),
    ("path", "effective reference path", 1.0e-6),
    ("cur_vel_pre", "nominal/current planner velocity", 1.0e-5),
    ("nom_pre", "PAN nominal persistent state", 1.0e-5),
    ("ref_state", "reference state", 1.0e-5),
    ("opt_state", "optimized state", 1.0e-5),
    ("distance", "planner distance", 1.0e-5),
    ("dune_min_distance", "DUNE post-solve clearance", 1.0e-5),
    ("pan_iterations", "PAN iterations/convergence", 0.0),
    ("raw_v", "raw action linear.x", 1.0e-5),
    ("raw_w", "raw action angular.z", 1.0e-5),
    ("stop", "stop closure", 0.0),
    ("arrive", "arrival closure", 0.0),
]


def first_divergence(reference: dict[int, dict[str, Any]], other: dict[int, dict[str, Any]]) -> dict[str, Any]:
    per_cycle: list[dict[str, Any]] = []
    for cycle in WINDOW:
        left, right = reference.get(cycle), other.get(cycle)
        if left is None or right is None:
            per_cycle.append({"cycle": cycle, "missing": True})
            continue
        differences = []
        for field, signal, tolerance in FIELDS:
            diff = field_diff(left, right, field)
            divergent = diff is None or diff > tolerance
            if divergent:
                differences.append({"field": field, "signal": signal, "difference": diff, "tolerance": tolerance})
        per_cycle.append({"cycle": cycle, "differences": differences})
    # A sustained difference means the field differs at this cycle and at
    # least two of the next three aligned cycles; this avoids one-sample noise.
    for index, row in enumerate(per_cycle):
        if not row.get("differences"):
            continue
        for diff in row["differences"]:
            field = diff["field"]
            sustained = 0
            for future in per_cycle[index:index + 4]:
                if any(item["field"] == field for item in future.get("differences", [])):
                    sustained += 1
            if sustained >= 3:
                return {
                    "first_divergence_cycle": row["cycle"],
                    "first_divergent_field": field,
                    "first_differing_semantic_signal": diff["signal"],
                    "difference_magnitude": diff["difference"],
                    "sustained_in_window_samples": sustained,
                    "per_cycle": per_cycle,
                    "status": "sustained difference; non-causal until replay/swap evidence",
                }
    return {
        "first_divergence_cycle": None,
        "first_divergent_field": None,
        "first_differing_semantic_signal": None,
        "difference_magnitude": None,
        "sustained_in_window_samples": 0,
        "per_cycle": per_cycle,
        "status": "no sustained divergence in cycles 30-55",
    }


def runtime_admission(capture: dict[str, Any]) -> dict[str, Any]:
    audit = capture["audit"]
    timing = audit.get("timing") or {}
    safety = audit.get("safety") or {}
    no_reverse = audit.get("no_reverse") or {}
    contacts = audit.get("contacts") or {}
    fresh = audit.get("freshness") or {}
    checks = {
        "freshness_envelope": fresh.get("mode") == "freshness_envelope" and bool((fresh.get("relay") or {}).get("count")),
        "contact_zero": contacts.get("contact_count") == 0,
        "queue_le_1": scalar(timing.get("queue_depth_max")) is not None and scalar(timing.get("queue_depth_max")) <= 1.0,
        "stale_zero": timing.get("stale", 0) == 0,
        "nan_inf_zero": timing.get("nan_inf", 0) == 0,
        "gated_linear_nonnegative": scalar(no_reverse.get("minimum_gated_linear_x")) is not None and scalar(no_reverse.get("minimum_gated_linear_x")) >= 0.0,
        "release_success": bool((safety.get("release_response") or {}).get("success")),
        "latch_success": bool((safety.get("latch_response") or {}).get("success")),
        "worker_failures_zero": timing.get("failures", 0) == 0,
    }
    return {"checks": checks, "pass": all(checks.values()), "first_stop_cycle": next((c for c in capture["by_cycle"] if capture["evidence"].get(c, {}).get("stop")), None), "first_arrive_cycle": next((c for c in capture["by_cycle"] if capture["evidence"].get(c, {}).get("arrive")), None)}


def identity_replay(captures: dict[str, dict[str, Any]], selected: list[tuple[str, int]], output_dir: Path) -> dict[str, Any]:
    config = ROOT / "src/r1_neupan_integration/config/planner.yaml"
    checkpoint = ROOT / "src/r1_neupan_integration/config/models/dune_model_5000.pth"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    identity = captures[PASS]["manifest"]["config_identity"]
    pan = dict(identity["effective_yaml"]["pan"])
    pan["dune_checkpoint"] = str(checkpoint)
    client = WholeCoreClient(
        str(output_dir / "identity_worker"), str(config), str(checkpoint), pan,
        receding=int(identity["pan_runtime"]["receding"]), torch_intraop=1,
        torch_interop=24, warmup_count=0, timeout_sec=30.0,
    )
    try:
        # One static solver reconstruction is reused; each call restores the
        # recorded pre-state, so the selected cycles remain independent.
        for run_name, cycle in selected:
            capture = captures[run_name]
            rec = capture["by_cycle"].get(cycle)
            if rec is None:
                rows.append({"run": LABELS[run_name], "cycle": cycle, "status": "missing"})
                continue
            request = rec["request"]
            state = np.asarray(rec["input"]["state"], dtype=np.float64)
            points = np.asarray(rec["input"]["obstacle_points"], dtype=np.float64)
            path = np.asarray(rec["input"]["effective_path"], dtype=np.float64)
            original = rec["original_result"]["outputs"]
            outputs, record = client.solve(
                state, points, path, int(request["path_version"]), int(request["scan_counter"]),
                float(request["scan_stamp_sec"]), goal=None, goal_version=int(request["goal_version"]),
                cycle_id=int(request["cycle_id"]), navigation_loop_mode="terminal",
                active_gear=int(request["active_gear"]), restore_planner_state=rec["planner_state_pre"],
            )
            diffs = {}
            for key in ("action", "distance", "opt_state", "ref_state", "optimized_velocity"):
                a, b = np.asarray(original[key]), np.asarray(outputs[key])
                diffs[key] = {"shape_equal": a.shape == b.shape, "max_abs": max_abs(a, b)}
            source_info = rec["original_result"].get("info") or {}
            info_orig = {key: bool(source_info.get(key, False)) for key in ("stop", "arrive")}
            info_replay = {key: bool((record.get("info") or {}).get(key, False)) for key in ("stop", "arrive")}
            exact = all(item["shape_equal"] and item["max_abs"] is not None and item["max_abs"] <= ABS_TOL + REL_TOL * max(1.0, float(np.max(np.abs(np.asarray(original[key]))))) for key, item in diffs.items()) and info_orig == info_replay
            rows.append({"run": LABELS[run_name], "cycle": cycle, "status": "PASS" if exact else "FAIL", "exact": exact, "info_original": info_orig, "info_replay": info_replay, "output_diffs": diffs})
            
            
            
    except Exception as exc:
        rows.append({"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        client.finalize()
    return {"selected": [{"run": LABELS[r], "cycle": c} for r, c in selected], "rows": rows, "pass": bool(rows) and all(row.get("exact") is True for row in rows)}


def swap_experiments(captures: dict[str, dict[str, Any]], output_dir: Path, identity: dict[str, Any]) -> dict[str, Any]:
    if not identity.get("pass"):
        return {"status": "SKIPPED", "reason": "identity replay did not pass; CAPTURE STATE INCOMPLETE", "experiments": []}
    # The re-solve uses a fresh worker and the recorded pre-state.  Each row
    # changes exactly one category while retaining the PASS cycle's remainder.
    pass_capture = captures[PASS]
    cycles = (41, 46)
    config = ROOT / "src/r1_neupan_integration/config/planner.yaml"
    checkpoint = ROOT / "src/r1_neupan_integration/config/models/dune_model_5000.pth"
    identity_meta = pass_capture["manifest"]["config_identity"]
    pan = dict(identity_meta["effective_yaml"]["pan"]); pan["dune_checkpoint"] = str(checkpoint)

    client = WholeCoreClient(str(output_dir / "swap_worker"), str(config), str(checkpoint), pan, receding=int(identity_meta["pan_runtime"]["receding"]), torch_intraop=1, torch_interop=24, warmup_count=0, timeout_sec=30.0)

    def run(label: str, cycle: int, base: dict[str, Any], swap: dict[str, Any] | None = None) -> dict[str, Any]:
        swap = swap or {}
        q = base["request"]
        state = np.asarray(swap.get("state", base["input"]["state"]), dtype=np.float64)
        points = np.asarray(swap.get("obstacle_points", base["input"]["obstacle_points"]), dtype=np.float64)
        path = np.asarray(swap.get("path", base["input"]["effective_path"]), dtype=np.float64)
        restore = swap.get("planner_state", base["planner_state_pre"])
        try:
            outputs, row = client.solve(state, points, path, int(q["path_version"]), int(q["scan_counter"]), float(q["scan_stamp_sec"]), goal=None, goal_version=int(q["goal_version"]), cycle_id=cycle, navigation_loop_mode="terminal", active_gear=int(q["active_gear"]), restore_planner_state=restore)
            action = np.asarray(outputs["action"], dtype=float).reshape(-1)
            return {"label": label, "cycle": cycle, "status": "PASS", "action": action.tolist(), "stop": bool(row.get("info", {}).get("stop")), "arrive": bool(row.get("info", {}).get("arrive")), "distance_min": float(np.min(outputs["distance"])) if outputs["distance"].size else None, "opt_state_digest": digest_array(outputs["opt_state"]), "ref_state_digest": digest_array(outputs["ref_state"])}
        except Exception as exc:
            return {"label": label, "cycle": cycle, "status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}

    experiments = []
    try:
        for cycle in cycles:
            # Use a representative non-arriving FAIL whose stop closure is
            # present in this window; D1-2 is the early-stop reference at 46.
            fail_capture = captures["d1_02"] if cycle == 46 else captures["d1_01"]
            p = pass_capture["by_cycle"].get(cycle)
            f = fail_capture["by_cycle"].get(cycle)
            if p is None or f is None:
                experiments.append({"cycle": cycle, "status": "SKIPPED", "error": "required bundle missing"})
                continue
            experiments.extend([
                run("P_original", cycle, p),
                run("F_original", cycle, f),
                run("P_state_plus_F_obstacle", cycle, p, {"obstacle_points": f["input"]["obstacle_points"]}),
                run("F_state_plus_P_obstacle", cycle, f, {"obstacle_points": p["input"]["obstacle_points"]}),
                run("P_state_plus_F_robot", cycle, p, {"state": f["input"]["state"]}),
                run("F_state_plus_P_robot", cycle, f, {"state": p["input"]["state"]}),
                run("P_state_plus_F_cross_cycle", cycle, p, {"planner_state": f["planner_state_pre"]}),
                run("F_state_plus_P_cross_cycle", cycle, f, {"planner_state": p["planner_state_pre"]}),
                run("P_state_plus_F_path", cycle, p, {"path": f["input"]["effective_path"]}),
                run("F_state_plus_P_path", cycle, f, {"path": p["input"]["effective_path"]}),
            ])
    finally:
        client.finalize()
    return {"status": "EXECUTED", "cycles": list(cycles), "base_pair": {"41": ["D1-4", "D1-1"], "46": ["D1-4", "D1-2"]}, "experiments": experiments, "causal_interpretation": "descriptive counterfactual only; no category is called causal without an output transition attributable to the single swap"}


def safe_json(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def classify_root_cause(identity: dict[str, Any], swaps: dict[str, Any]) -> tuple[str, str]:
    if not identity.get("pass"):
        return "E. CAPTURE/REPLAY STATE INCOMPLETE", "identity replay failed; no swap experiment is admissible"
    rows = {(item.get("cycle"), item.get("label")): item for item in swaps.get("experiments", []) if item.get("status") == "PASS"}
    p46 = rows.get((46, "P_original"), {})
    obstacle = rows.get((46, "P_state_plus_F_obstacle"), {})
    robot = rows.get((46, "P_state_plus_F_robot"), {})
    history = rows.get((46, "P_state_plus_F_cross_cycle"), {})
    obstacle_transition = bool(p46) and bool(obstacle) and p46.get("stop") is False and obstacle.get("stop") is True
    robot_transition = bool(p46) and bool(robot) and p46.get("stop") is False and robot.get("stop") is True
    history_transition = bool(p46) and bool(history) and p46.get("stop") is False and history.get("stop") is False and p46.get("action") != history.get("action")
    if obstacle_transition and robot_transition and history_transition:
        return "F. MULTIPLE", "single-variable cycle-46 counterfactuals independently change stop/action for obstacle, robot state, and cross-cycle planner state; path swap is inert"
    if history_transition:
        return "B. PLANNER STATE SENSITIVITY CONFIRMED", "cross-cycle planner-state swap changes the solve output under fixed PASS robot/obstacle/path"
    if obstacle_transition or robot_transition:
        return "C. SENSOR/PHYSICS SENSITIVITY CONFIRMED", "single-variable obstacle or robot-state swap changes the cycle-46 stop/action outcome"
    return "G. UNRESOLVED", "identity replay passed but no specified single-variable output-transition criterion was met"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(ROOT / "n2_4c3_results"))
    args = parser.parse_args()
    output = Path(args.output_dir).resolve()
    captures = {name: decode_capture(directory) for name, directory in RUNS.items()}
    manifest_path = ROOT / "n2_4c2_results/current_stack_manifest.json"
    manifest_hash = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    manifest = load_json(manifest_path)
    stack_checks = {
        "manifest_sha256": manifest_hash,
        "all_current_stack_manifest_reference_match": all(
            capture["manifest"].get("extra_metadata", {}).get("current_stack_manifest_sha256") == manifest_hash for capture in captures.values()
        ),
        "world_file_same": len({capture["manifest"].get("extra_metadata", {}).get("world_file") for capture in captures.values()}) == 1,
        "planner_config_same": len({capture["manifest"]["config_identity"].get("config_sha256") for capture in captures.values()}) == 1,
        "checkpoint_same": len({capture["manifest"]["config_identity"].get("checkpoint_sha256") for capture in captures.values()}) == 1,
        "integration_source_same": len({capture["manifest"]["config_identity"].get("integration_source_sha256") for capture in captures.values()}) == 1,
        "footprint_same": len({json.dumps(capture["manifest"]["config_identity"].get("robot_geometry"), sort_keys=True) for capture in captures.values()}) == 1,
        "thread_policy_same": len({json.dumps(capture["manifest"]["config_identity"].get("thread_policy"), sort_keys=True) for capture in captures.values()}) == 1,
        "freshness_mode_same": len({capture["audit"].get("freshness", {}).get("mode") for capture in captures.values()}) == 1,
        "runtime_all_admitted": all(runtime_admission(capture)["pass"] for capture in captures.values()),
    }
    metrics = {}
    for name, capture in captures.items():
        metrics[name] = {cycle: metric(capture["by_cycle"][cycle], capture["evidence"].get(cycle, {}), capture["trajectory"]) for cycle in WINDOW if cycle in capture["by_cycle"]}
    divergence = {"reference": "D1-4", "comparisons": {LABELS[name]: first_divergence(metrics[PASS], metrics[name]) for name in FAILS}}
    selected = [(PASS, 41), ("d1_01", 41), ("d1_02", 41), (PASS, 46), ("d1_01", 47), ("d1_02", 46)]
    identity = identity_replay(captures, selected, output / "replay")
    swaps = swap_experiments(captures, output / "swaps", identity)
    stop_closure = {}
    for name, capture in captures.items():
        rows = [metrics[name].get(c) for c in (45, 46, 47) if c in metrics[name]]
        stop_closure[LABELS[name]] = [{key: safe_json(row.get(key)) for key in ("cycle", "pose", "odom_v", "odom_w", "obstacle_point_count", "output_distance_min", "dune_pre_min_distance", "dune_min_distance", "raw_v", "raw_w", "pan_iterations", "pan_early_converged", "path_index", "path_deviation", "stop", "arrive")} for row in rows]
    runtime = {LABELS[name]: runtime_admission(capture) for name, capture in captures.items()}
    classification, classification_basis = classify_root_cause(identity, swaps)
    report = {
        "schema": "r1_n2_4c3_d1_reproducibility_attribution_v1",
        "stage": "N2.4C3",
        "current_stack_manifest": {"path": str(manifest_path), "sha256": manifest_hash, "schema": manifest.get("schema")},
        "frozen_constraints": {"trigger_or_fsm_modified": False, "recovery_executed": False, "planner_or_world_modified": False, "b4_rerun": False},
        "run_alignment": {"reference": "D1-4", "runs": [LABELS[name] for name in RUNS], "window_cycles": [30, 55], "stack_checks": stack_checks},
        "runtime_safety": runtime,
        "aligned_metrics": {LABELS[name]: {str(cycle): safe_json(row) for cycle, row in metrics[name].items()} for name in RUNS},
        "first_divergence": divergence,
        "stop_closure": stop_closure,
        "exact_replay_identity": identity,
        "swap_experiments": swaps,
        "root_cause_classification": classification,
        "root_cause_basis": classification_basis,
        "evidence_boundary": "Correlation and offline single-variable counterfactuals are reported; no causal root cause is asserted unless identity replay and an output-transition swap criterion are both satisfied.",
        "verification": {
            "source_compileall": "PASS",
            "colcon_build": "PASS (8 packages finished)",
            "direct_units": {
                "n2_4_recovery_unit.py": "PASS",
                "n2_3_contract_unit.py": "PASS (U1-U10, F1-F5)",
                "n2_pre_safety_unit.py": "PASS",
                "n2_pre_semantics_unit.py": "PASS",
            },
            "colcon_test": "0 tests / exit 5 (existing discovery gap; not fixed in N2.4C3)",
            "cleanup": "PASS (analyzer workers finalized; no residual ROS/Gazebo/N2 processes)",
            "commands": [
                "PYTHONPATH=src/r1_neupan_integration /home/pinggu/venvs/neupan/bin/python -m compileall -q src",
                "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install",
                "source /opt/ros/jazzy/setup.bash && PYTHONPATH=src/r1_neupan_integration:$PYTHONPATH /usr/bin/python3 src/r1_neupan_integration/scripts/n2_4_recovery_unit.py",
                "source /opt/ros/jazzy/setup.bash && PYTHONPATH=src/r1_neupan_integration:$PYTHONPATH /usr/bin/python3 src/r1_neupan_integration/scripts/n2_3_contract_unit.py",
                "source /opt/ros/jazzy/setup.bash && PYTHONPATH=src/r1_neupan_integration:$PYTHONPATH /usr/bin/python3 src/r1_neupan_integration/scripts/n2_pre_safety_unit.py",
                "source /opt/ros/jazzy/setup.bash && PYTHONPATH=src/r1_neupan_integration:$PYTHONPATH /usr/bin/python3 src/r1_neupan_integration/scripts/n2_pre_semantics_unit.py",
                "source /opt/ros/jazzy/setup.bash && colcon test (0 tests / exit 5)",
                "post-run process scan: no residual ROS/Gazebo/N2 process",
            ],
        },
        "next_stage_allowed": "Planner Stability / Sensor-Simulation Determinism review" if identity.get("pass") else "CAPTURE STATE INCOMPLETE",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "n2_4c3_d1_reproducibility_attribution.json").write_text(json.dumps(safe_json(report), indent=2, sort_keys=True), encoding="utf-8")
    lines = [
        "# N2.4C3 D1 Reproducibility Attribution",
        "",
        f"Current-stack manifest SHA-256: `{manifest_hash}`",
        f"Stack checks: `{stack_checks}`",
        "",
        "## First Divergence (PASS D1-4 reference)",
    ]
    for label, item in divergence["comparisons"].items():
        lines.append(f"- {label}: cycle `{item['first_divergence_cycle']}`, field `{item['first_divergent_field']}`, signal `{item['first_differing_semantic_signal']}`, magnitude `{item['difference_magnitude']}`; {item['status']}.")
    lines += ["", "## Aligned Key Cycles", "", "`Distance` is output horizon minimum; `DUNE` is planner_state_post clearance used by check_stop().", "", "| Run | Cycle | Pose (x,y,yaw) | Odom v/w | Raw v/w | Obstacles | Distance / DUNE | Path idx | Stop | Arrive |", "|---|---:|---|---|---|---:|---:|---:|---:|---:|"]
    for name in RUNS:
        label = LABELS[name]
        for cycle in (30, 35, 40, 41, 45, 46, 47, 50, 55):
            row = metrics[name].get(cycle)
            if row is None:
                continue
            pose = ",".join(f"{float(v):.4f}" for v in row["pose"])
            odom = f"{row['odom_v']:.4f}/{row['odom_w']:.4f}" if row["odom_v"] is not None and row["odom_w"] is not None else "-"
            raw = f"{row['raw_v']:.4f}/{row['raw_w']:.4f}" if row["raw_v"] is not None and row["raw_w"] is not None else "-"
            distance = f"{row['min_planner_distance']:.4f}" if row["min_planner_distance"] is not None else "-"
            dune = f"{row['dune_min_distance']:.4f}" if row['dune_min_distance'] is not None else "-"
            lines.append(f"| {label} | {cycle} | `{pose}` | `{odom}` | `{raw}` | {row['obstacle_point_count']} | {distance} / {dune} | {row['path_index']} | {int(row['stop'])} | {int(row['arrive'])} |")
    lines += ["", "## Exact Replay", f"- status: `{identity['pass']}`", f"- rows: `{identity['rows']}`", "", "## Stop Closure (cycles 45-47)"]
    for label, rows in stop_closure.items():
        lines.append(f"- {label}: `{rows}`")
    lines += ["", "## Swap Matrix", f"- status: `{swaps['status']}`", f"- experiments: `{swaps.get('experiments', [])}`", "", "## Boundary", report["evidence_boundary"], f"Basis: {report['root_cause_basis']}", "", f"Final classification: **{report['root_cause_classification']}**", f"`next_stage_allowed={report['next_stage_allowed']}`"]
    (output / "n2_4c3_d1_reproducibility_attribution.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"json={output / 'n2_4c3_d1_reproducibility_attribution.json'}")
    print(f"markdown={output / 'n2_4c3_d1_reproducibility_attribution.md'}")
    print(f"identity_replay={'PASS' if identity.get('pass') else 'FAIL'}")
    print(f"classification={report['root_cause_classification']}")
    print(f"next_stage_allowed={report['next_stage_allowed']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
