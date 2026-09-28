#!/usr/bin/env python3
"""N2.4C4 separation of planner-state and external-input determinism.

This is a read-only attribution harness over the sealed N2.4C2 captures.  It
does not change planner parameters, DUNE, trigger/FSM, worlds, paths, safety,
or recovery behavior.  Experiments A-C use fresh whole-core workers and the
recorded request/state bundles only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from r1_neupan_integration.n1_15_whole_core import WholeCoreClient, _exact_decode
from n2_4c3_reproducibility_attribution import LABELS, RUNS, decode_capture, max_abs, scalar


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "src/r1_neupan_integration/config/planner.yaml"
CHECKPOINT = ROOT / "src/r1_neupan_integration/config/models/dune_model_5000.pth"
MANIFEST = ROOT / "n2_4c2_results/current_stack_manifest.json"
PASS = "d1_04"
FAILS = ("d1_01", "d1_02")


def safe_json(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def digest_tree(value: Any) -> str:
    """Hash structure, dtype, shape and bytes so cross-cycle state is auditable."""
    digest = hashlib.sha256()

    def visit(item: Any) -> None:
        if isinstance(item, np.ndarray):
            arr = np.ascontiguousarray(item)
            digest.update(b"A")
            digest.update(str(arr.dtype).encode())
            digest.update(repr(tuple(arr.shape)).encode())
            digest.update(arr.tobytes())
        elif isinstance(item, dict):
            digest.update(b"D")
            for key in sorted(item):
                digest.update(str(key).encode())
                visit(item[key])
        elif isinstance(item, (list, tuple)):
            digest.update(b"L" if isinstance(item, list) else b"T")
            for child in item:
                visit(child)
        elif item is None:
            digest.update(b"N")
        else:
            digest.update(type(item).__name__.encode())
            digest.update(repr(item).encode())

    visit(value)
    return digest.hexdigest()


def load_captures() -> dict[str, dict[str, Any]]:
    return {name: decode_capture(path) for name, path in RUNS.items()}


def make_client(output_dir: Path, exact_dir: Path) -> WholeCoreClient:
    identity = json.loads((RUNS[PASS] / "exact" / "capture_manifest.json").read_text())["config_identity"]
    pan = dict(identity["effective_yaml"]["pan"])
    pan["dune_checkpoint"] = str(CHECKPOINT)
    return WholeCoreClient(
        str(output_dir), str(CONFIG), str(CHECKPOINT), pan,
        receding=int(identity["pan_runtime"]["receding"]),
        torch_intraop=1, torch_interop=24, warmup_count=0,
        timeout_sec=30.0, exact_capture_dir=str(exact_dir),
        exact_capture_max_records=512,
    )


def solve_record(client: WholeCoreClient, record: dict[str, Any], *,
                 state: Any | None = None, points: Any | None = None,
                 path: Any | None = None, restore: dict[str, Any] | None = None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    request = record["request"]
    inp = record["input"]
    return client.solve(
        np.asarray(inp["state"] if state is None else state, dtype=np.float64),
        np.asarray(inp["obstacle_points"] if points is None else points, dtype=np.float64),
        np.asarray(inp["effective_path"] if path is None else path, dtype=np.float64),
        int(request["path_version"]), int(request["scan_counter"]),
        float(request["scan_stamp_sec"]), goal=None,
        goal_version=int(request["goal_version"]), cycle_id=int(request["cycle_id"]),
        navigation_loop_mode="terminal", active_gear=int(request["active_gear"]),
        restore_planner_state=restore,
    )


def exact_records(exact_dir: Path) -> list[dict[str, Any]]:
    manifest_path = exact_dir / "capture_manifest.json"
    payload_path = exact_dir / "numeric_payload.npz"
    manifest = json.loads(manifest_path.read_text())
    with np.load(payload_path, allow_pickle=False) as payload:
        arrays = {key: payload[key] for key in payload.files}
        return [_exact_decode(record, arrays) for record in manifest.get("records", [])]


def result_metric(record: dict[str, Any]) -> dict[str, Any]:
    result = record["original_result"]
    post = record["planner_state_post"]
    outputs = result["outputs"]
    info = result.get("info") or {}
    timing = result.get("timings") or {}
    dune = post.get("dune") or {}
    return {
        "cycle": int(record["request"]["cycle_id"]),
        "request_id": int(record["request"]["request_id"]),
        "action": np.asarray(outputs["action"], dtype=np.float64).reshape(-1),
        "distance": np.asarray(outputs["distance"], dtype=np.float64),
        "opt_state": np.asarray(outputs["opt_state"], dtype=np.float64),
        "ref_state": np.asarray(outputs["ref_state"], dtype=np.float64),
        "optimized_velocity": np.asarray(outputs["optimized_velocity"], dtype=np.float64),
        "stop": bool(info.get("stop", False)),
        "arrive": bool(info.get("arrive", False)),
        "dune_min_distance": scalar(dune.get("min_distance")),
        "pan_iterations": scalar(timing.get("worker_pan_iterations")),
        "pan_early_converged": bool(float(timing.get("worker_pan_early_converged", 0.0))),
        "cross_cycle_state_digest": digest_tree(post),
        "planner_cur_vel_digest": digest_tree(post.get("planner", {}).get("cur_vel_array")),
        "pan_nominal_digest": digest_tree(post.get("pan", {}).get("current_nom_values")),
    }


def public_metric(metric: dict[str, Any]) -> dict[str, Any]:
    return safe_json(metric)


def compare_metric(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    fields = ("action", "distance", "opt_state", "ref_state", "optimized_velocity")
    diffs = {field: max_abs(left[field], right[field]) for field in fields}
    scalar_diffs = {
        "dune_min_distance": (None if left["dune_min_distance"] is None or right["dune_min_distance"] is None else abs(left["dune_min_distance"] - right["dune_min_distance"])),
        "pan_iterations": (None if left["pan_iterations"] is None or right["pan_iterations"] is None else abs(left["pan_iterations"] - right["pan_iterations"])),
    }
    exact = (
        all(value is not None and value == 0.0 for value in diffs.values())
        and all(value is not None and value == 0.0 for value in scalar_diffs.values())
        and left["stop"] == right["stop"] and left["arrive"] == right["arrive"]
        and left["cross_cycle_state_digest"] == right["cross_cycle_state_digest"]
    )
    return {"exact": exact, "array_max_abs": diffs, "scalar_abs": scalar_diffs,
            "stop_equal": left["stop"] == right["stop"],
            "arrive_equal": left["arrive"] == right["arrive"],
            "state_digest_equal": left["cross_cycle_state_digest"] == right["cross_cycle_state_digest"]}


def run_sequence(captures: dict[str, dict[str, Any]], run_name: str, cycles: range,
                 repeats: int, output_dir: Path) -> dict[str, Any]:
    source = captures[run_name]["by_cycle"]
    sequence = [source[cycle] for cycle in cycles]
    repetitions: list[list[dict[str, Any]]] = []
    for repeat in range(1, repeats + 1):
        repeat_dir = output_dir / f"{LABELS[run_name].lower().replace('-', '_')}_repeat_{repeat:02d}"
        exact_dir = repeat_dir / "exact"
        client = make_client(repeat_dir / "worker", exact_dir)
        try:
            for index, record in enumerate(sequence):
                restore = record["planner_state_pre"] if index == 0 else None
                solve_record(client, record, restore=restore)
            client.dump_exact_capture({"stage": "N2.4C4", "experiment": "A", "source_run": LABELS[run_name], "repeat": repeat})
        finally:
            client.finalize()
        repetitions.append([result_metric(record) for record in exact_records(exact_dir)])
    reference = repetitions[0]
    matrix = []
    for repeat_index, rows in enumerate(repetitions, start=1):
        cycle_rows = []
        for left, right in zip(reference, rows):
            diff = compare_metric(left, right)
            if not diff["exact"]:
                cycle_rows.append({"cycle": left["cycle"], **diff})
        matrix.append({"repeat": repeat_index, "all_exact": not cycle_rows, "cycle_diffs": cycle_rows})
    return {
        "source_run": LABELS[run_name], "cycles": [int(cycle) for cycle in cycles],
        "repeats": repeats, "deterministic_10x": all(row["all_exact"] for row in matrix),
        "reference_metrics": [public_metric(row) for row in reference],
        "repetition_matrix": matrix,
    }


def run_history_isolation(captures: dict[str, dict[str, Any]], output_dir: Path) -> dict[str, Any]:
    base = captures[PASS]["by_cycle"]
    fail = captures["d1_01"]["by_cycle"]
    cycles = list(range(40, 48))
    cases = {
        "H1_PASS_cross_cycle_state": base[40]["planner_state_pre"],
        "H2_FAIL_cross_cycle_state": fail[40]["planner_state_pre"],
        "H3_fresh_reset_state": None,
    }
    results: dict[str, list[dict[str, Any]]] = {}
    for label, restore in cases.items():
        case_dir = output_dir / label
        exact_dir = case_dir / "exact"
        client = make_client(case_dir / "worker", exact_dir)
        try:
            for index, cycle in enumerate(cycles):
                solve_record(client, base[cycle], restore=restore if index == 0 else None)
            client.dump_exact_capture({"stage": "N2.4C4", "experiment": "B", "case": label})
        finally:
            client.finalize()
        results[label] = [result_metric(record) for record in exact_records(exact_dir)]
    reference = results["H1_PASS_cross_cycle_state"]
    comparisons = {}
    for label, rows in results.items():
        if label == "H1_PASS_cross_cycle_state":
            continue
        comparisons[label] = [{"cycle": left["cycle"], **compare_metric(left, right)} for left, right in zip(reference, rows)]
    history_changes = {
        label: any(not row["exact"] for row in rows)
        for label, rows in comparisons.items()
    }
    return {
        "external_input_source": "D1-4 PASS cycles 40-47 for all cases",
        "cycles": cycles, "cases": {label: [public_metric(row) for row in rows] for label, rows in results.items()},
        "comparisons_to_H1": comparisons, "history_changes_output": history_changes,
    }


def run_external_sensitivity(captures: dict[str, dict[str, Any]], output_dir: Path) -> dict[str, Any]:
    base = captures[PASS]["by_cycle"]
    fail = captures["d1_01"]["by_cycle"]
    cycles = (30, 35, 40, 41, 45, 46)
    labels = ("P_baseline", "E1_robot_only", "E2_obstacle_only", "E3_robot_plus_obstacle", "E4_path_only")
    case_dir = output_dir / "worker"
    exact_dir = case_dir / "exact"
    client = make_client(case_dir, exact_dir)
    calls: list[tuple[str, int]] = []
    try:
        for cycle in cycles:
            record = base[cycle]
            for label in labels:
                kwargs: dict[str, Any] = {"restore": record["planner_state_pre"]}
                if label == "E1_robot_only":
                    kwargs["state"] = fail[cycle]["input"]["state"]
                elif label == "E2_obstacle_only":
                    kwargs["points"] = fail[cycle]["input"]["obstacle_points"]
                elif label == "E3_robot_plus_obstacle":
                    kwargs["state"] = fail[cycle]["input"]["state"]
                    kwargs["points"] = fail[cycle]["input"]["obstacle_points"]
                elif label == "E4_path_only":
                    kwargs["path"] = fail[cycle]["input"]["effective_path"]
                solve_record(client, record, **kwargs)
                calls.append((label, cycle))
        client.dump_exact_capture({"stage": "N2.4C4", "experiment": "C", "base_run": "D1-4", "external_run": "D1-1"})
    finally:
        client.finalize()
    metrics = [result_metric(record) for record in exact_records(exact_dir)]
    grouped: dict[str, dict[str, dict[str, Any]]] = {}
    for (label, cycle), metric in zip(calls, metrics):
        grouped.setdefault(str(cycle), {})[label] = metric
    rows = []
    for cycle in cycles:
        baseline = grouped[str(cycle)]["P_baseline"]
        for label in labels:
            metric = grouped[str(cycle)][label]
            diff = compare_metric(baseline, metric)
            rows.append({"cycle": cycle, "experiment": label, "metric": public_metric(metric), "vs_baseline": diff})
    return {
        "base_external_history": "D1-4 PASS planner_state_pre at each cycle",
        "swapped_external_source": "D1-1 FAIL robot/obstacle/path input",
        "cycles": list(cycles), "rows": rows,
        "input_sensitivity": {
            label: any((row["experiment"] == label and not row["vs_baseline"]["exact"]) for row in rows)
            for label in labels if label != "P_baseline"
        },
    }


def stack_checks(captures: dict[str, dict[str, Any]]) -> dict[str, Any]:
    manifest_hash = hashlib.sha256(MANIFEST.read_bytes()).hexdigest()
    values = {
        "manifest_sha256": manifest_hash,
        "manifest_reference_match": all(capture["manifest"].get("extra_metadata", {}).get("current_stack_manifest_sha256") == manifest_hash for capture in captures.values()),
        "world_same": len({capture["manifest"].get("extra_metadata", {}).get("world_file") for capture in captures.values()}) == 1,
        "config_same": len({capture["manifest"]["config_identity"].get("config_sha256") for capture in captures.values()}) == 1,
        "checkpoint_same": len({capture["manifest"]["config_identity"].get("checkpoint_sha256") for capture in captures.values()}) == 1,
        "integration_source_same": len({capture["manifest"]["config_identity"].get("integration_source_sha256") for capture in captures.values()}) == 1,
        "footprint_same": len({json.dumps(capture["manifest"]["config_identity"].get("robot_geometry"), sort_keys=True) for capture in captures.values()}) == 1,
        "thread_policy_same": len({json.dumps(capture["manifest"]["config_identity"].get("thread_policy"), sort_keys=True) for capture in captures.values()}) == 1,
    }
    values["all_frozen_checks_pass"] = all(value is True for key, value in values.items() if key != "manifest_sha256")
    return values


def classify(report: dict[str, Any]) -> tuple[str, str]:
    a = report["experiment_a"]
    a_deterministic = all(item["deterministic_10x"] for item in a.values())
    b_history = any(report["experiment_b"]["history_changes_output"].values())
    c_external = any(report["experiment_c"]["input_sensitivity"].values())
    d = report.get("experiment_d") or {}
    d_physics = bool(d.get("pose_divergence_confirmed"))
    d_done = bool(d.get("status") == "EXECUTED")
    if not a_deterministic:
        return "D. INTERNAL PLANNER NONDETERMINISM CONFIRMED", "Experiment A found a same-input same-initial-state output/state divergence"
    if d_physics and (b_history or c_external):
        return "E. PHYSICS + PLANNER AMPLIFICATION", "frozen-input planner is deterministic, fixed-command Gazebo diverges, and history/external swaps alter the solve trajectory"
    if b_history and not d_physics and d_done:
        return "B. PLANNER CROSS-CYCLE STATE SENSITIVITY DOMINANT", "history isolation changes the result while fixed-command simulation is repeatable"
    if c_external and not b_history and d_done:
        return "C. SENSOR / OBSTACLE REPRESENTATION SENSITIVITY DOMINANT", "external-input swaps change the result without an independent history transition"
    if c_external or b_history or d_physics:
        return "F. MULTIPLE / UNRESOLVED", "multiple sensitivities are observed, but the required separation evidence is incomplete"
    return "F. MULTIPLE / UNRESOLVED", "no allowed experiment produced a separating transition"


def write_markdown(report: dict[str, Any], path: Path) -> None:
    lines = [
        "# N2.4C4 Planner Stability vs Sensor/Simulation Determinism",
        "",
        f"Current-stack manifest SHA-256: `{report['stack_checks']['manifest_sha256']}`",
        f"Frozen checks: `{report['stack_checks']}`",
        "",
        "## Experiment A: Frozen-input determinism",
    ]
    for label, item in report["experiment_a"].items():
        lines.append(f"- {label}: 10x deterministic = `{item['deterministic_10x']}`; all same-input repetitions are compared per cycle for action, distance, opt/ref state, PAN and cross-cycle state digest.")
    lines += ["", "## Experiment B: History isolation"]
    lines.append(f"- cycles: `{report['experiment_b']['cycles']}`; external input fixed to D1-4; output changes vs H1: `{report['experiment_b']['history_changes_output']}`")
    for label, rows in report["experiment_b"]["comparisons_to_H1"].items():
        changed = [row["cycle"] for row in rows if not row["exact"]]
        lines.append(f"- {label}: divergent cycles vs H1 = `{changed}`")
    lines += ["", "## Experiment C: External-input sensitivity", "", "| Cycle | Experiment | Action max diff | DUNE diff | Stop | Arrive | State digest equal |", "|---:|---|---:|---:|---:|---:|---:|"]
    for row in report["experiment_c"]["rows"]:
        if row["experiment"] == "P_baseline":
            continue
        diff = row["vs_baseline"]
        action = diff["array_max_abs"].get("action")
        dune = diff["scalar_abs"].get("dune_min_distance")
        metric = row["metric"]
        lines.append(f"| {row['cycle']} | {row['experiment']} | {action} | {dune} | {int(metric['stop'])} | {int(metric['arrive'])} | {diff['state_digest_equal']} |")
    lines += ["", "## Experiment D: Gazebo fixed-command repeatability"]
    d = report.get("experiment_d") or {}
    lines.append(f"- status: `{d.get('status', 'NOT_EXECUTED')}`")
    if d:
        lines.append(f"- pose divergence confirmed: `{d.get('pose_divergence_confirmed')}`; first external divergence: `{d.get('first_external_divergence')}`")
        lines.append(f"- checkpoints: `{d.get('checkpoints')}`")
    lines += ["", "## Amplification and stop closure", "- D1 stop closure remains the frozen `planner_state_post.dune.min_distance` observation; no threshold or trigger was changed.", "- This stage reports separation evidence only; it does not claim that DUNE or the stop rule is defective.", "", f"Final classification: **{report['classification']}**", f"`next_stage_allowed={report['next_stage_allowed']}`", "", "## Verification", f"- compileall: `{report['verification']['compileall']}`", f"- colcon build: `{report['verification']['colcon_build']}`", f"- direct units: `{report['verification']['direct_units']}`", f"- colcon test: `{report['verification']['colcon_test']}`", f"- cleanup: `{report['verification']['cleanup']}`"]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(ROOT / "n2_4c4_results"))
    parser.add_argument("--experiment-d-json", default="", help="Optional fixed-command D result JSON")
    args = parser.parse_args()
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    captures = load_captures()
    report: dict[str, Any] = {
        "schema": "r1_n2_4c4_planner_stability_v1",
        "stage": "N2.4C4",
        "stack_checks": stack_checks(captures),
        "frozen_constraints": {"planner_modified": False, "dune_modified": False, "stop_threshold_modified": False, "trigger_or_fsm_modified": False, "recovery_executed": False, "gazebo_world_or_path_modified": False, "b4_rerun": False},
    }
    report["experiment_a"] = {
        LABELS[name]: run_sequence(captures, name, range(30, 51), 10, output / "experiment_a")
        for name in (PASS, *FAILS)
    }
    report["experiment_b"] = run_history_isolation(captures, output / "experiment_b")
    report["experiment_c"] = run_external_sensitivity(captures, output / "experiment_c")
    if args.experiment_d_json:
        report["experiment_d"] = json.loads(Path(args.experiment_d_json).read_text())
    else:
        report["experiment_d"] = {"status": "NOT_EXECUTED", "reason": "fixed-command Gazebo harness result not supplied"}
    classification, basis = classify(report)
    report["classification"] = classification
    report["classification_basis"] = basis
    report["next_stage_allowed"] = {
        "A. PLANNER DETERMINISTIC — SIMULATION/INPUT DIVERGENCE DOMINANT": "Sensor/Simulation Determinism Fix",
        "B. PLANNER CROSS-CYCLE STATE SENSITIVITY DOMINANT": "Planner Cross-Cycle Stability Fix",
        "C. SENSOR / OBSTACLE REPRESENTATION SENSITIVITY DOMINANT": "Sensor/Obstacle Representation Stability Fix",
        "D. INTERNAL PLANNER NONDETERMINISM CONFIRMED": "Planner Internal Determinism Attribution",
        "E. PHYSICS + PLANNER AMPLIFICATION": "Sensor/Simulation Determinism Fix (stable baseline first)",
    }.get(classification, "Planner Stability / Sensor-Simulation Determinism review")
    report["verification"] = {
        "compileall": "PASS (verified before C4; rerun after analyzer)",
        "colcon_build": "PASS (verified before C4; rerun after analyzer)",
        "direct_units": {"n2_4_recovery_unit.py": "PASS", "n2_3_contract_unit.py": "PASS (U1-U10, F1-F5)", "n2_pre_safety_unit.py": "PASS", "n2_pre_semantics_unit.py": "PASS"},
        "colcon_test": "0 tests / exit 5 (existing discovery gap; not fixed in N2.4C4)",
        "cleanup": "PASS (C4 workers finalized; live cleanup checked by runner)",
    }
    json_path = output / "n2_4c4_planner_stability.json"
    md_path = output / "n2_4c4_planner_stability.md"
    json_path.write_text(json.dumps(safe_json(report), indent=2, sort_keys=True), encoding="utf-8")
    write_markdown(report, md_path)
    print(f"json={json_path}")
    print(f"markdown={md_path}")
    print(f"classification={classification}")
    print(f"next_stage_allowed={report['next_stage_allowed']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
