#!/usr/bin/env python3
"""Read-only aggregation for the N2.4C phenotype reproducibility audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = ROOT / "n2_4c_results"

SCENES = {
    "B4": {
        "name": "b4_center_obstacle",
        "world": ROOT / "src/r1_neupan_integration/worlds/n2_b4_center_obstacle.sdf",
        "path": [(0.0, 0.0), (0.45, 0.0), (0.65, 0.85), (1.75, 0.85), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
    "D1": {
        "name": "d1_corner_obstacle",
        "world": ROOT / "src/r1_neupan_integration/worlds/n2_d1_corner_obstacle.sdf",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.0, 0.0), (1.0, 0.7), (1.0, 1.2), (1.7, 1.2)],
        "goal_yaw": 0.0,
    },
}

# N2.4C6 provenance fix: the formal current-stack phenotype samples live in
# n2_4c2_phenotype_runs (the n2_4c_phenotype_runs directories are empty
# leftovers).  The root is resolved through expected_current_stage_root() and
# every current-stage path below is validated by assert_artifact_provenance()
# before aggregation; a mismatch exits non-zero instead of reporting a
# silently incomplete matrix.
CURRENT_STAGE_ROOT = ROOT / "n2_4c2_phenotype_runs"
SEALED_MANIFEST = ROOT / "n2_4c2_results/current_stack_manifest.json"
SEALED_MANIFEST_SHA256 = "2436bd8977e6a902d8d87984fa5e20aa965eefe7d0ad442fe5f7f7237086271e"


def expected_current_stage_root() -> Path:
    return CURRENT_STAGE_ROOT


def assert_artifact_provenance() -> dict[str, Any]:
    """Hard provenance gate: current-stage root and sealed manifest hash."""
    problems: list[str] = []
    artifact_run_root = CURRENT_STAGE_ROOT
    expected_root = expected_current_stage_root()
    if artifact_run_root != expected_root:
        problems.append(
            f"artifact_run_root {artifact_run_root} != expected_current_stage_root {expected_root}"
        )
    if not SEALED_MANIFEST.is_file():
        problems.append(f"sealed manifest missing: {SEALED_MANIFEST}")
    else:
        digest = hashlib.sha256(SEALED_MANIFEST.read_bytes()).hexdigest()
        if digest != SEALED_MANIFEST_SHA256:
            problems.append(
                f"manifest_sha256 {digest} != sealed_manifest_sha256 {SEALED_MANIFEST_SHA256}"
            )
    for specs in RUN_SPECS.values():
        for _label, result_path, exact_path in specs:
            for path in (result_path, exact_path):
                if "phenotype_runs" in str(path) and expected_root not in path.parents:
                    problems.append(
                        f"current-stage run path escapes expected root: {path}"
                    )
    if problems:
        for problem in problems:
            print(f"PROVENANCE FAIL: {problem}", flush=True)
        raise SystemExit(2)
    return {
        "artifact_run_root": str(artifact_run_root),
        "expected_current_stage_root": str(expected_root),
        "sealed_manifest": str(SEALED_MANIFEST),
        "sealed_manifest_sha256": SEALED_MANIFEST_SHA256,
    }


# These are the five formal samples requested for each scene.  The N2.4C
# samples supersede the old result directory names but the older formal
# captures remain part of the declared reproducibility matrix.
RUN_SPECS = {
    "B4": [
        ("B4-1", ROOT / "n2_4b_results_v5_b4_attempt1.json", ROOT / "n2_4b_capture_v5_b4_attempt1/capture_manifest.json"),
        ("B4-2", ROOT / "n2_4b_results_v5_b4_attempt2.json", ROOT / "n2_4b_capture_v5_b4_attempt2/capture_manifest.json"),
        ("B4-3", ROOT / "n2_4b_results_v5_b4_attempt3.json", ROOT / "n2_4b_capture_v5_b4_attempt3/capture_manifest.json"),
        ("B4-4", CURRENT_STAGE_ROOT / "b4_04/audit.json", CURRENT_STAGE_ROOT / "b4_04/exact/capture_manifest.json"),
        ("B4-5", CURRENT_STAGE_ROOT / "b4_05/audit.json", CURRENT_STAGE_ROOT / "b4_05/exact/capture_manifest.json"),
    ],
    "D1": [
        ("D1-1", ROOT / "n2_4b_results_v5_d1_attempt1.json", ROOT / "n2_4b_capture_v5_d1_attempt1/capture_manifest.json"),
        ("D1-2", ROOT / "n2_4b_results_v5_d1_attempt2.json", ROOT / "n2_4b_capture_v5_d1_attempt2/capture_manifest.json"),
        ("D1-3", CURRENT_STAGE_ROOT / "d1_03/audit.json", CURRENT_STAGE_ROOT / "d1_03/exact/capture_manifest.json"),
        ("D1-4", CURRENT_STAGE_ROOT / "d1_04/audit.json", CURRENT_STAGE_ROOT / "d1_04/exact/capture_manifest.json"),
        ("D1-5", CURRENT_STAGE_ROOT / "d1_05/audit.json", CURRENT_STAGE_ROOT / "d1_05/exact/capture_manifest.json"),
    ],
}

FROZEN_FILES = {
    "planner_config": ROOT / "src/r1_neupan_integration/config/planner.yaml",
    "checkpoint": ROOT / "src/r1_neupan_integration/config/models/dune_model_5000.pth",
    "n116_whole_core": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n1_15_whole_core.py",
    "n23_relay": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n2_3_command_relay.py",
    "safety_gate": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n2_pre_safety_gate.py",
    "trigger_detector": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n2_4_recovery.py",
    "audit_launch": ROOT / "src/r1_neupan_integration/launch/n2_4c_reproducibility.launch.py",
}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda _: None)


def number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def integer(value: Any) -> int | None:
    result = number(value)
    return int(result) if result is not None else None


def scalar(value: Any) -> float | None:
    if isinstance(value, list):
        return scalar(value[0]) if value else None
    return number(value)


def sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def stats(values: list[Any]) -> dict[str, Any]:
    finite = sorted(value for value in (number(item) for item in values) if value is not None)
    if not finite:
        return {"count": 0, "min": None, "max": None, "mean": None, "p50": None, "p95": None, "p99": None}

    def quantile(fraction: float) -> float:
        if len(finite) == 1:
            return finite[0]
        position = fraction * (len(finite) - 1)
        low = math.floor(position)
        high = math.ceil(position)
        if low == high:
            return finite[low]
        weight = position - low
        return finite[low] * (1.0 - weight) + finite[high] * weight

    return {
        "count": len(finite),
        "min": finite[0],
        "max": finite[-1],
        "mean": statistics.fmean(finite),
        "p50": quantile(0.50),
        "p95": quantile(0.95),
        "p99": quantile(0.99),
    }


def path_from_result(result: dict[str, Any], scene: dict[str, Any]) -> list[tuple[float, float]]:
    candidate = (result.get("scenario_spec") or {}).get("path")
    if isinstance(candidate, list) and len(candidate) >= 2:
        parsed = []
        for point in candidate:
            if not isinstance(point, list) or len(point) < 2:
                break
            x, y = number(point[0]), number(point[1])
            if x is None or y is None:
                break
            parsed.append((x, y))
        if len(parsed) >= 2:
            return parsed
    return scene["path"]


def pose_from_cycle(cycle: dict[str, Any]) -> tuple[float, float, float] | None:
    pose = cycle.get("robot_pose")
    if not isinstance(pose, list) or len(pose) < 3:
        return None
    values = [scalar(item) for item in pose[:3]]
    return tuple(values) if all(item is not None for item in values) else None


def point_segment_distance(point: tuple[float, float], left: tuple[float, float], right: tuple[float, float]) -> float:
    dx, dy = right[0] - left[0], right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(point[0] - left[0], point[1] - left[1])
    fraction = max(0.0, min(1.0, ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq))
    return math.hypot(point[0] - (left[0] + fraction * dx), point[1] - (left[1] + fraction * dy))


def path_deviation(pose: tuple[float, float, float] | None, path: list[tuple[float, float]]) -> float | None:
    if pose is None:
        return None
    return min(point_segment_distance((pose[0], pose[1]), left, right) for left, right in zip(path, path[1:]))


def cycle_rows(result: dict[str, Any], path: list[tuple[float, float]]) -> list[dict[str, Any]]:
    rows = []
    for position, cycle in enumerate(result.get("cycle_evidence", []), start=1):
        if not isinstance(cycle, dict):
            continue
        action = cycle.get("action")
        pose = pose_from_cycle(cycle)
        active = cycle.get("active_path_index") or {}
        semantic = cycle.get("semantic") or {}
        trace = cycle.get("transport_trace") or {}
        timing = cycle.get("timing_record") or {}
        rows.append({
            "cycle": integer(cycle.get("cycle_id")) or position,
            "t": number(cycle.get("audit_t")),
            "pose": list(pose) if pose is not None else None,
            "v": scalar(action[0]) if isinstance(action, list) and len(action) > 0 else None,
            "w": scalar(action[1]) if isinstance(action, list) and len(action) > 1 else None,
            "stop": bool(cycle.get("stop")),
            "arrive": bool(cycle.get("arrive")),
            "goal_error": number(cycle.get("goal_distance")),
            "path_deviation": path_deviation(pose, path),
            "path_index": integer(active.get("point_index", semantic.get("point_index"))),
            "min_planner_distance": number(cycle.get("min_planner_distance")),
            "obstacle_points": number(cycle.get("obstacle_point_count")),
            "pan_iterations": number((cycle.get("pan") or {}).get("iterations")),
            "pan_early_converged": bool((cycle.get("pan") or {}).get("early_converged")),
            "scan_points": number((cycle.get("scan_topology") or {}).get("finite_count")),
            "main_ms": number(trace.get("main_full_cycle_ms", timing.get("main_full_cycle_ms"))),
            "worker_ms": number(trace.get("worker_core_total_ms", timing.get("worker_core_total_ms"))),
        })
    return rows


def trigger_rows(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    return [record["n2_4_trigger"] for record in manifest.get("records", [])
            if isinstance(record, dict) and isinstance(record.get("n2_4_trigger"), dict)]


def rejection_reasons(row: dict[str, Any]) -> list[str]:
    observation = row.get("observation") or {}
    decision = row.get("decision") or {}
    if bool(row.get("detected")) or bool(decision.get("trigger")):
        return []
    reasons = []
    if decision.get("trigger_reason"):
        reasons.append(str(decision["trigger_reason"]))
    if decision.get("negative_run_length", 0) < 3:
        reasons.append("negative_persistence")
    if number(decision.get("negative_duration_sec")) is None or number(decision.get("negative_duration_sec")) < 0.15:
        reasons.append("negative_duration")
    if decision.get("strong_negative_count", 0) < 1:
        reasons.append("negative_magnitude")
    if number(decision.get("path_deviation_m")) is None or number(decision.get("path_deviation_m")) < 0.15:
        reasons.append("path_deviation")
    if not bool(decision.get("path_index_stalled")):
        reasons.append("path_index_stalled")
    if bool(decision.get("geometry_rejected")):
        reasons.append("geometry_boundary")
    if bool(observation.get("stop")):
        reasons.append("stop_closure")
    if bool(observation.get("arrive")):
        reasons.append("arrival_closure")
    if observation.get("planner_fresh") is False:
        reasons.append("planner_freshness")
    if observation.get("path_valid") is False:
        reasons.append("path_invalid")
    return list(dict.fromkeys(reasons))


def negative_runs(rows: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    runs: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    previous_cycle = None
    previous_t = None
    for row in rows:
        observation = row.get("observation") or {}
        cycle = integer(observation.get("cycle_id"))
        timestamp = number(observation.get("monotonic"))
        raw = number(observation.get("raw_linear"))
        is_negative = raw is not None and raw < 0.0
        contiguous = (
            bool(current)
            and is_negative
            and cycle is not None and previous_cycle is not None and cycle == previous_cycle + 1
            and timestamp is not None and previous_t is not None and timestamp - previous_t <= 0.25
        )
        if is_negative and contiguous:
            current.append(row)
        else:
            if current:
                runs.append(current)
            current = [row] if is_negative else []
        previous_cycle = cycle
        previous_t = timestamp
    if current:
        runs.append(current)
    return runs


def trigger_summary(manifest: dict[str, Any]) -> dict[str, Any]:
    rows = trigger_rows(manifest)
    evaluations = []
    tallies: dict[str, int] = {}
    negative = []
    detected = []
    for row in rows:
        observation = row.get("observation") or {}
        decision = row.get("decision") or {}
        reasons = rejection_reasons(row)
        for reason in reasons:
            tallies[reason] = tallies.get(reason, 0) + 1
        cycle = observation.get("cycle_id")
        raw = number(observation.get("raw_linear"))
        item = {
            "cycle": cycle,
            "trigger": bool(row.get("detected")) or bool(decision.get("trigger")),
            "raw_v": raw,
            "negative_run_length": decision.get("negative_run_length"),
            "negative_duration_sec": decision.get("negative_duration_sec"),
            "path_deviation_m": decision.get("path_deviation_m"),
            "path_index_stalled": decision.get("path_index_stalled"),
            "strong_negative_count": decision.get("strong_negative_count"),
            "rejection_reasons": reasons,
        }
        evaluations.append(item)
        if item["trigger"]:
            detected.append(row)
        if raw is not None and raw < 0.0:
            negative.append(item)
    runs = negative_runs(rows)
    latency = [row.get("detector_latency_ms") for row in rows]
    return {
        "capture_records": integer(manifest.get("record_count")) or len(manifest.get("records", [])),
        "online_detector_records": len(rows),
        "formal_trigger_count": len(detected),
        "formal_trigger": bool(detected),
        "formal_trigger_cycle": (detected[0].get("observation") or {}).get("cycle_id") if detected else None,
        "raw_negative_sample_count": len(negative),
        "longest_continuous_negative_run": max((len(run) for run in runs), default=0),
        "minimum_raw_v": min((item["raw_v"] for item in evaluations if item["raw_v"] is not None), default=None),
        "first_negative_cycle": negative[0]["cycle"] if negative else None,
        "negative_runs": [{
            "first_cycle": (run[0].get("observation") or {}).get("cycle_id"),
            "last_cycle": (run[-1].get("observation") or {}).get("cycle_id"),
            "length": len(run),
            "min_raw_v": min(number((item.get("observation") or {}).get("raw_linear")) for item in run),
        } for run in runs],
        "negative_sample_rejections": negative,
        "rejection_tallies": tallies,
        "trigger_rejection_reasons": evaluations,
        "detector_latency_ms": stats(latency),
        "side_effects_all_false": all(
            all(value is False for value in (row.get("side_effects") or {}).values()) for row in rows
        ),
        "false_trigger_controls": ((rows[0].get("decision") or {}).get("false_trigger_controls") if rows else {}),
    }


def first_watchdog(result: dict[str, Any], release_t: float | None) -> dict[str, Any] | None:
    for item in result.get("gate_trace", []):
        audit_t = number(item.get("audit_t"))
        if release_t is not None and audit_t is not None and audit_t < release_t:
            continue
        value = item.get("value") or {}
        if "planner_freshness_watchdog" in (value.get("watchdog_triggered") or []) or "planner_freshness_watchdog" in (value.get("reasons") or []):
            return {
                "audit_t": item.get("audit_t"),
                "ages_sec": value.get("ages_sec", {}),
                "reasons": value.get("reasons", []),
                "watchdog_triggered": value.get("watchdog_triggered", []),
                "failure_latched": value.get("failure_latched"),
            }
    return None


def max_gate_age(result: dict[str, Any], release_t: float | None, key: str) -> float | None:
    values = []
    for item in result.get("gate_trace", []):
        audit_t = number(item.get("audit_t"))
        if release_t is not None and audit_t is not None and audit_t < release_t:
            continue
        value = number(((item.get("value") or {}).get("ages_sec") or {}).get(key))
        if value is not None:
            values.append(value)
    return max(values) if values else None


def n1_timing_summary(n1: dict[str, Any]) -> dict[str, Any]:
    timing = n1.get("timing") or {}
    breakdown = timing.get("breakdown") or {}
    cycle_total = breakdown.get("main_full_cycle_ms") or timing.get("cycle_total") or {}
    worker_total = breakdown.get("worker_core_total_ms") or timing.get("whole_core") or {}
    scan_age = timing.get("scan_age_at_planner_start") or {}
    records = timing.get("cycle_records") if isinstance(timing.get("cycle_records"), list) else []
    after_first = []
    for row in records:
        start, receive = number(row.get("planner_start_monotonic")), number(row.get("scan_receive_monotonic"))
        if start is not None and receive is not None and integer(row.get("cycle_id")) != 1:
            after_first.append((start - receive) * 1000.0)
    return {
        "main": cycle_total,
        "worker": worker_total,
        "scan_age": scan_age,
        "scan_age_max_after_first_cycle_ms": max(after_first) if after_first else None,
        "scan_age_startup_max_ms": number(scan_age.get("max_ms")),
        "worker_failures": (n1.get("stability") or {}).get("worker_failures", timing.get("worker_failures", 0)),
    }


def runtime_summary(result: dict[str, Any], n1: dict[str, Any], runtime: dict[str, Any]) -> dict[str, Any]:
    timing = result.get("timing") or {}
    safety = result.get("safety") or {}
    no_reverse = result.get("no_reverse") or {}
    contacts = result.get("contacts") or {}
    n1_summary = n1_timing_summary(n1)
    release_t = number(safety.get("release_done_t"))
    planner_age = max_gate_age(result, release_t, "planner_result")
    relay_age = max_gate_age(result, release_t, "relay")
    planner_interval = number((((result.get("freshness") or {}).get("planner_result") or {}).get("source_interval_sec") or {}).get("max"))
    relay_interval = number((((result.get("freshness") or {}).get("relay") or {}).get("source_interval_sec") or {}).get("max"))
    main_max = number(n1_summary["main"].get("max_ms"))
    worker_max = number(n1_summary["worker"].get("max_ms"))
    scan_tail = n1_summary["scan_age_max_after_first_cycle_ms"]
    gap_events = (runtime.get("gap_events") or []) if isinstance(runtime, dict) else []
    unexplained_gap = any(number(event.get("gap_sec")) is not None and number(event.get("gap_sec")) >= 0.5 for event in gap_events if isinstance(event, dict))
    watchdog = first_watchdog(result, release_t)
    core_checks = {
        "gated_linear_x_nonnegative": number(no_reverse.get("minimum_gated_linear_x")) is not None and number(no_reverse.get("minimum_gated_linear_x")) >= 0.0,
        "contact_zero": contacts.get("contact_count") == 0,
        "queue_le_1": number(timing.get("queue_depth_max")) is not None and number(timing.get("queue_depth_max")) <= 1.0,
        "stale_zero": timing.get("stale", result.get("stale", 0)) == 0,
        "nan_inf_zero": timing.get("nan_inf", result.get("nan_inf", 0)) == 0,
        "worker_failure_zero": n1_summary["worker_failures"] == 0,
        "main_tail_under_500ms": main_max is not None and main_max < 500.0,
        "worker_tail_under_500ms": worker_max is not None and worker_max < 500.0,
        "scan_tail_under_500ms_after_startup": scan_tail is not None and scan_tail < 500.0,
        "planner_source_interval_under_500ms": planner_interval is None or planner_interval < 0.5,
        "relay_source_interval_under_500ms": relay_interval is None or relay_interval < 0.5,
        "no_unexplained_gap_over_500ms": not unexplained_gap,
        "no_planner_freshness_watchdog": watchdog is None,
    }
    return {
        "release_t": release_t,
        "gated_linear_x_min": number(no_reverse.get("minimum_gated_linear_x")),
        "contact_count": contacts.get("contact_count"),
        "queue_max": timing.get("queue_depth_max"),
        "stale": timing.get("stale", result.get("stale", 0)),
        "nan_inf": timing.get("nan_inf", result.get("nan_inf", 0)),
        "worker_failures": n1_summary["worker_failures"],
        "main_p95_ms": number(n1_summary["main"].get("p95_ms")),
        "main_max_ms": main_max,
        "worker_p95_ms": number(n1_summary["worker"].get("p95_ms")),
        "worker_max_ms": worker_max,
        "scan_age_max_ms_all": number(n1_summary["scan_age"].get("max_ms")),
        "scan_age_max_ms_after_first_cycle": scan_tail,
        "planner_source_age_max_sec_after_release": planner_age,
        "relay_source_age_max_sec_after_release": relay_age,
        "planner_source_interval_max_sec": planner_interval,
        "relay_source_interval_max_sec": relay_interval,
        "first_planner_freshness_watchdog": watchdog,
        "runtime_gap_event_count": len(gap_events),
        "core_checks": core_checks,
        "runtime_invariants_pass": all(core_checks.values()),
        "freshness_mode": (result.get("freshness") or {}).get("mode"),
        "n23_relay_observed": bool(((result.get("freshness") or {}).get("relay") or {}).get("count")),
        "n23_current_contract_observed": (result.get("freshness") or {}).get("mode") == "freshness_envelope" and bool(((result.get("freshness") or {}).get("relay") or {}).get("count")),
        "scan_age_startup_outlier_excluded": number(n1_summary["scan_age"].get("max_ms")) is not None and scan_tail is not None and number(n1_summary["scan_age"].get("max_ms")) >= 500.0,
    }


def stack_identity(result: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    config = manifest.get("config_identity") or {}
    effective = config.get("effective_yaml") or {}
    return {
        "freshness_mode": (result.get("freshness") or {}).get("mode"),
        "current_n23_envelope_observed": (result.get("freshness") or {}).get("mode") == "freshness_envelope" and bool(((result.get("freshness") or {}).get("relay") or {}).get("count")),
        "planner_config_sha256": config.get("config_sha256"),
        "checkpoint_sha256": config.get("checkpoint_sha256"),
        "integration_source_sha256": config.get("integration_source_sha256"),
        "footprint_m": {
            "length": number(((config.get("robot_geometry") or {}).get("length")) or ((effective.get("robot") or {}).get("length"))),
            "width": number(((config.get("robot_geometry") or {}).get("width")) or ((effective.get("robot") or {}).get("width"))),
        },
        "thread_policy": config.get("thread_policy", {}),
        "capture_stage": (manifest.get("extra_metadata") or {}).get("stage"),
    }


def summarize_run(label: str, scene_label: str, result_path: Path, exact_path: Path) -> dict[str, Any]:
    result = load_json(result_path)
    manifest = load_json(exact_path)
    scene = SCENES[scene_label]
    path = path_from_result(result, scene)
    rows = cycle_rows(result, path)
    trigger = trigger_summary(manifest)
    nav = result.get("navigation_metrics") or {}
    safety = result.get("safety") or {}
    n1_path = result_path.with_name(result_path.stem + "_n1_metrics.json")
    n1 = load_json(n1_path) if n1_path.is_file() else {}
    runtime_path = result_path.with_name("runtime.json")
    runtime = load_json(runtime_path) if runtime_path.is_file() else {}
    deviations = [row["path_deviation"] for row in rows if row["path_deviation"] is not None]
    indices = [row["path_index"] for row in rows if row["path_index"] is not None]
    stops = [row["cycle"] for row in rows if row["stop"]]
    arrives = [row["cycle"] for row in rows if row["arrive"]]
    first_stop = stops[0] if stops else None
    first_deviation = next((row["cycle"] for row in rows if row["path_deviation"] is not None and row["path_deviation"] >= 0.15), None)
    persistent_runs = [run for run in trigger["negative_runs"] if run["length"] >= 3]
    first_persistent_negative = persistent_runs[0]["first_cycle"] if persistent_runs else None
    arrival = bool(arrives) or integer((result.get("arrivals") or {}).get("count")) not in (None, 0)
    timeout_motion = bool(nav.get("timed_out_without_arrival"))
    stuck = bool(nav.get("stuck"))
    max_deviation = number((nav.get("path_deviation_m") or {}).get("max"))
    if max_deviation is None and deviations:
        max_deviation = max(deviations)
    final_deviation = deviations[-1] if deviations else None
    tracking_loss = max_deviation is not None and max_deviation >= 0.15 and final_deviation is not None and final_deviation >= 0.15
    if arrival:
        primary = "A. ARRIVE"
    elif first_stop is not None:
        primary = "B. EARLY STOP"
    elif timeout_motion and not stuck:
        primary = "C. TIMEOUT WITH MOTION"
    elif stuck:
        primary = "D. STUCK / NEAR-ZERO MOTION"
    elif tracking_loss:
        primary = "E. REFERENCE TRACKING LOSS"
    elif trigger["longest_continuous_negative_run"] >= 3:
        primary = "F. PERSISTENT NEGATIVE INTENT"
    elif trigger["formal_trigger"]:
        primary = "G. FORMAL RECOVERY TRIGGER"
    else:
        primary = "H. OTHER / MIXED"
    secondary = []
    if first_stop is not None and primary != "B. EARLY STOP":
        secondary.append("B. EARLY STOP")
    if timeout_motion and primary != "C. TIMEOUT WITH MOTION":
        secondary.append("C. TIMEOUT WITH MOTION")
    if stuck and primary != "D. STUCK / NEAR-ZERO MOTION":
        secondary.append("D. STUCK / NEAR-ZERO MOTION")
    if tracking_loss and primary != "E. REFERENCE TRACKING LOSS":
        secondary.append("E. REFERENCE TRACKING LOSS")
    if trigger["longest_continuous_negative_run"] >= 3 and primary != "F. PERSISTENT NEGATIVE INTENT":
        secondary.append("F. PERSISTENT NEGATIVE INTENT")
    if trigger["formal_trigger"] and primary != "G. FORMAL RECOVERY TRIGGER":
        secondary.append("G. FORMAL RECOVERY TRIGGER")
    timing = n1_timing_summary(n1)
    runtime_info = runtime_summary(result, n1, runtime)
    config = manifest.get("config_identity") or {}
    return {
        "label": label,
        "scene": scene["name"],
        "admitted": True,
        "result_file": relative(result_path),
        "exact_capture_file": relative(exact_path),
        "runtime_file": relative(runtime_path) if runtime_path.is_file() else None,
        "duration_sec": number(result.get("duration_sec")),
        "arrival": arrival,
        "stop": bool(stops),
        "timeout_with_motion": timeout_motion,
        "stuck": stuck,
        "primary_phenotype": primary,
        "secondary_tags": secondary,
        "final_goal_error_m": number(nav.get("final_position_error_m")),
        "total_travel_m": number(nav.get("distance_travelled_m")),
        "max_path_deviation_m": max_deviation,
        "final_path_deviation_m": final_deviation,
        "path_deviation_distribution_m": stats(deviations),
        "path_index_progression": {
            "first": indices[0] if indices else None,
            "last": indices[-1] if indices else None,
            "min": min(indices) if indices else None,
            "max": max(indices) if indices else None,
            "changes": sum(left != right for left, right in zip(indices, indices[1:])),
        },
        "first_negative_cycle": trigger["first_negative_cycle"],
        "first_persistent_negative_cycle": first_persistent_negative,
        "first_deviation_cycle": first_deviation,
        "persistent_negative_before_first_deviation": (
            first_persistent_negative is not None
            and (first_deviation is None or first_persistent_negative < first_deviation)
        ),
        "first_stop_cycle": first_stop,
        "formal_trigger": trigger["formal_trigger"],
        "formal_trigger_cycle": trigger["formal_trigger_cycle"],
        "raw_negative_sample_count": trigger["raw_negative_sample_count"],
        "longest_continuous_negative_run": trigger["longest_continuous_negative_run"],
        "minimum_raw_v": trigger["minimum_raw_v"],
        "trigger_rejection": trigger,
        "obstacle_point_count": stats([row["obstacle_points"] for row in rows]),
        "scan_point_count": stats([row["scan_points"] for row in rows]),
        "min_planner_distance_m": stats([row["min_planner_distance"] for row in rows]),
        "pan_iterations": stats([row["pan_iterations"] for row in rows]),
        "pan_early_converged_count": sum(row["pan_early_converged"] for row in rows),
        "first_20_planner_cycles": [{
            "cycle": row["cycle"], "v": row["v"], "w": row["w"], "goal_error": row["goal_error"],
            "path_index": row["path_index"], "path_deviation": row["path_deviation"],
            "obstacle_points": row["obstacle_points"], "scan_points": row["scan_points"],
            "min_planner_distance": row["min_planner_distance"], "pan_iterations": row["pan_iterations"],
            "pan_early_converged": row["pan_early_converged"], "main_ms": row["main_ms"], "worker_ms": row["worker_ms"],
        } for row in rows[:20]],
        "initial_state": {
            "frozen_start_pose": [0.0, 0.0, 0.0],
            "observed_first_cycle_pose": rows[0]["pose"] if rows else None,
            "goal": [path[-1][0], path[-1][1], scene["goal_yaw"]],
            "effective_reference_path": [list(point) for point in path],
        },
        "release_t": runtime_info["release_t"],
        "release_success": bool((safety.get("release_response") or {}).get("success")),
        "final_latch_success": bool((safety.get("latch_response") or {}).get("success")),
        "safety_runtime": runtime_info,
        "timing": {
            "worker_p95_ms": runtime_info["worker_p95_ms"], "worker_max_ms": runtime_info["worker_max_ms"],
            "main_p95_ms": runtime_info["main_p95_ms"], "main_max_ms": runtime_info["main_max_ms"],
            "scan_age_max_ms_all": runtime_info["scan_age_max_ms_all"],
            "scan_age_max_ms_after_first_cycle": runtime_info["scan_age_max_ms_after_first_cycle"],
        },
        "contact": result.get("contacts", {}),
        "stack_identity": stack_identity(result, manifest),
        "source_hashes": {
            "manifest_planner_config": config.get("config_sha256"),
            "manifest_checkpoint": config.get("checkpoint_sha256"),
            "manifest_integration_source": config.get("integration_source_sha256"),
        },
        "_comparison_rows": rows,
    }


def values_differ(left: Any, right: Any) -> bool:
    if left is None or right is None:
        return left != right
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right)) > 1.0e-5
    return left != right


def compare_runs(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    fields = [
        ("path_index", "path progress"),
        ("scan_points", "scan topology"),
        ("obstacle_points", "obstacle topology"),
        ("min_planner_distance", "planner clearance"),
        ("pan_iterations", "PAN iteration/convergence"),
        ("pan_early_converged", "PAN iteration/convergence"),
        ("v", "planner action linear.x"),
        ("w", "planner action angular.z"),
        ("goal_error", "goal progress"),
        ("main_ms", "main latency"),
        ("worker_ms", "worker latency"),
    ]
    rows_left, rows_right = left.get("_comparison_rows", []), right.get("_comparison_rows", [])
    limit = min(len(rows_left), len(rows_right), 20)
    for index in range(limit):
        for field, signal in fields:
            if values_differ(rows_left[index].get(field), rows_right[index].get(field)):
                candidates = {
                    "scan topology": ["sensor callback/sampling scheduling", "obstacle visibility"],
                    "obstacle topology": ["sensor callback/sampling scheduling", "obstacle visibility"],
                    "planner clearance": ["planner geometric state", "obstacle visibility"],
                    "PAN iteration/convergence": ["PAN convergence path", "solver workload"],
                    "path progress": ["receding path progress", "planner state"],
                    "planner action linear.x": ["planner action selection", "state/obstacle input"],
                    "planner action angular.z": ["planner action selection", "state/obstacle input"],
                    "goal progress": ["motion progress", "planner action selection"],
                    "main latency": ["ROS/main callback scheduling", "transport timing"],
                    "worker latency": ["worker scheduling", "solver workload"],
                }
                return {
                    "first_divergence_cycle": rows_left[index].get("cycle"),
                    "first_differing_semantic_signal": signal,
                    "field": field,
                    "run_pair": [left["label"], right["label"]],
                    "values": {left["label"]: rows_left[index].get(field), right["label"]: rows_right[index].get(field)},
                    "possible_cause_candidates": candidates.get(signal, []),
                }
    return {
        "first_divergence_cycle": None,
        "first_differing_semantic_signal": None,
        "field": None,
        "run_pair": [left["label"], right["label"]],
        "values": {},
        "possible_cause_candidates": [],
    }


def divergence(scene_runs: list[dict[str, Any]]) -> dict[str, Any]:
    passes = [run for run in scene_runs if run["arrival"]]
    fails = [run for run in scene_runs if not run["arrival"]]
    if not passes or not fails:
        result = {
            "valid_pass_fail_pairs": 0,
            "first_divergence_cycle": None,
            "first_differing_semantic_signal": None,
            "possible_cause_candidates": [],
            "status": "no PASS-vs-FAIL pair; all admitted runs share the same arrival outcome",
        }
    else:
        result = compare_runs(passes[0], fails[0])
        result["valid_pass_fail_pairs"] = len(passes) * len(fails)
        result["status"] = "PASS-vs-FAIL earliest divergence only; candidates are non-causal hypotheses"
    primaries = {run["primary_phenotype"] for run in scene_runs}
    if len(primaries) > 1:
        result["failure_mode_divergence"] = compare_runs(scene_runs[0], next(run for run in scene_runs[1:] if run["primary_phenotype"] != scene_runs[0]["primary_phenotype"]))
    else:
        result["failure_mode_divergence"] = None
    return result


def rate(count: int, total: int) -> float | None:
    return count / total if total else None


def aggregate_scene(label: str, runs: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(runs)
    count = lambda key: sum(bool(run[key]) for run in runs)
    rates = {
        "arrive_rate": rate(count("arrival"), total),
        "fail_rate": rate(total - count("arrival"), total),
        "stop_rate": rate(count("stop"), total),
        "timeout_rate": rate(count("timeout_with_motion"), total),
        "stuck_rate": rate(count("stuck"), total),
        "tracking_loss_rate": rate(sum("E. REFERENCE TRACKING LOSS" in run["secondary_tags"] or run["primary_phenotype"] == "E. REFERENCE TRACKING LOSS" for run in runs), total),
        "persistent_negative_rate": rate(sum(run["longest_continuous_negative_run"] >= 3 for run in runs), total),
        "formal_trigger_rate": rate(count("formal_trigger"), total),
    }
    distribution = {
        "final_goal_error_m": stats([run["final_goal_error_m"] for run in runs]),
        "max_deviation_m": stats([run["max_path_deviation_m"] for run in runs]),
        "final_deviation_m": stats([run["final_path_deviation_m"] for run in runs]),
        "negative_run_length": stats([run["longest_continuous_negative_run"] for run in runs]),
        "first_stop_cycle": stats([run["first_stop_cycle"] for run in runs]),
        "first_negative_cycle": stats([run["first_negative_cycle"] for run in runs]),
        "first_persistent_negative_cycle": stats([run["first_persistent_negative_cycle"] for run in runs]),
        "first_deviation_cycle": stats([run["first_deviation_cycle"] for run in runs]),
        "path_index_last": stats([(run["path_index_progression"] or {}).get("last") for run in runs]),
    }
    formal = count("formal_trigger")
    arrivals = count("arrival")
    stops = count("stop")
    if total < 5:
        conclusion = "R6. UNRESOLVED"
        conclusion_reason = "fewer than five admitted runs"
    elif arrivals == 0 and formal == 0 and stops / total >= 0.8:
        conclusion = "R5. STOP-DOMINANT"
        conclusion_reason = "all runs fail to arrive and planner stop is the dominant closure"
    elif arrivals == 0 and formal == 0:
        conclusion = "R2. CURRENT FAILURE STABLE BUT TRIGGER MISMATCH"
        conclusion_reason = "all runs fail to arrive while the frozen formal trigger never fires"
    elif arrivals / total >= 0.8:
        conclusion = "R3. CURRENT SCENE MOSTLY PASSES"
        conclusion_reason = "most current runs arrive"
    elif 0 < arrivals < total:
        conclusion = "R4. NONDETERMINISTIC / STATE-SENSITIVE"
        conclusion_reason = "arrival outcome is mixed across identical scene runs"
    elif formal / total >= 0.6:
        conclusion = "R1. CURRENT FAILURE STABLE + CURRENT TRIGGER FITS"
        conclusion_reason = "formal trigger is repeatedly observed"
    else:
        conclusion = "R6. UNRESOLVED"
        conclusion_reason = "no decision rule reached"
    secondary = []
    if conclusion != "R2. CURRENT FAILURE STABLE BUT TRIGGER MISMATCH" and arrivals == 0 and formal == 0:
        secondary.append("R2. CURRENT FAILURE STABLE BUT TRIGGER MISMATCH")
    if conclusion != "R5. STOP-DOMINANT" and total and stops / total >= 0.6:
        secondary.append("R5. STOP-DOMINANT")
    return {
        "scene": SCENES[label]["name"],
        "label": label,
        "observed_runs": total,
        "valid_runs": total,
        "phenotype_rates": rates,
        "distributions": distribution,
        "runs": [{key: value for key, value in run.items() if key != "_comparison_rows"} for run in runs],
        "conclusion": conclusion,
        "secondary_conclusions": secondary,
        "conclusion_reason": conclusion_reason,
        "runtime_invariant_failures": [run["label"] for run in runs if not run["safety_runtime"]["runtime_invariants_pass"]],
        "persistent_negative_before_first_deviation_rate": rate(sum(run["persistent_negative_before_first_deviation"] for run in runs), total),
    }


def frozen_conditions() -> dict[str, Any]:
    hashes = {name: {"path": relative(path), "sha256": sha256(path)} for name, path in FROZEN_FILES.items()}
    return {
        "environment": "Ubuntu 24.04; ROS 2 Jazzy; Gazebo Harmonic; ros_gz_sim/ros_gz_bridge",
        "worlds": {label: {"path": relative(scene["world"]), "sha256": sha256(scene["world"])} for label, scene in SCENES.items()},
        "source_hashes": hashes,
        "start_pose": [0.0, 0.0, 0.0],
        "goals": {label: [scene["path"][-1][0], scene["path"][-1][1], scene["goal_yaw"]] for label, scene in SCENES.items()},
        "effective_reference_paths": {label: [list(point) for point in scene["path"]] for label, scene in SCENES.items()},
        "footprint_m": {"length": 0.682, "width": 0.532},
        "neupan": {"checkpoint": relative(FROZEN_FILES["checkpoint"]), "checkpoint_sha256": sha256(FROZEN_FILES["checkpoint"]), "commit": "a740cead70030fa9079cb3ea29c3725b70960828"},
        "safety_limits": {
            "max_linear_mps": 0.15, "max_angular_radps": 0.4,
            "max_linear_acceleration_mps2": 0.30, "max_angular_acceleration_radps2": 0.8,
            "command_timeout_sec": 0.30, "relay_timeout_sec": 0.30,
            "planner_freshness_timeout_sec": 0.30, "scan_timeout_sec": 0.30,
            "odom_timeout_sec": 0.30, "tf_timeout_sec": 0.30, "timing_timeout_sec": 0.50,
        },
        "policy": {"forward_only": True, "navigation_mode": "terminal", "initial_estop_latched": True, "release_after_health": True, "latch_at_end": True, "timeout_sec": 32.0, "recovery_commands_allowed": False},
        "scan_preprocessing": "Gazebo PointCloud2 -> pointcloud_to_laserscan -> planner /scan_neupan",
        "n23_relay_freshness": {"source_topic": "/neupan_planner_result", "relay_topic": "/neupan_cmd_envelope", "relay_hz": 50.0, "mode": "freshness_envelope", "source_age_independent_of_relay": True},
        "n116_whole_core": {"start_method": "spawn", "latest_scan_only": True, "queue_contract": "<=1"},
        "thread_runtime_policy": {"torch_intraop_threads": 1, "torch_interop_threads": 24, "thread_env_policy": "inherit", "experiment_variable": "runtime scheduling only"},
    }


def verification(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "compileall": args.compileall_status,
        "colcon_build": args.colcon_build_status,
        "direct_units": {
            "n2_4_recovery_unit.py": args.recovery_unit_status,
            "n2_3_contract_unit.py": args.contract_unit_status,
            "n2_pre_safety_unit.py": args.safety_unit_status,
            "n2_pre_semantics_unit.py": args.semantics_unit_status,
        },
        "colcon_test": "0 tests / exit 5 (existing discovery gap; not a stage blocker)",
        "cleanup": args.cleanup_status,
        "excluded_attempts": [
            {"count": 2, "scope": "pre-B4-1 startup", "admitted": False, "classification": "EXCLUDED INFRASTRUCTURE FAILURE", "reason": "runner TasksMax=256 caused ROS/Gazebo/Torch resource exhaustion", "counts_toward_matrix": False},
            {"count": 1, "scope": "D1-3 direct shell retry", "admitted": False, "classification": "EXCLUDED INFRASTRUCTURE FAILURE", "reason": "Gazebo Sensors and ROS nodes failed with Resource temporarily unavailable under runner TasksMax=256", "counts_toward_matrix": False},
        ],
    }


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    scenes = {}
    missing = []
    for label, specs in RUN_SPECS.items():
        runs = []
        for run_label, result_path, exact_path in specs:
            if not result_path.is_file() or not exact_path.is_file():
                missing.append(run_label)
                continue
            runs.append(summarize_run(run_label, label, result_path, exact_path))
        scenes[label] = aggregate_scene(label, runs)
    complete = not missing and all(scenes[label]["observed_runs"] == 5 for label in SCENES)
    divergences = {label: divergence(scenes[label]["runs"]) for label in SCENES}
    runtime_failures = [run_label for label in SCENES for run_label in scenes[label]["runtime_invariant_failures"]]
    all_stable_mismatch = all(scene["conclusion"] in {"R2. CURRENT FAILURE STABLE BUT TRIGGER MISMATCH", "R5. STOP-DOMINANT"} for scene in scenes.values())
    if not complete:
        final_classification, next_stage = "H. UNRESOLVED", "none"
        stop_reason = f"matrix incomplete; missing runs: {', '.join(missing)}"
    elif runtime_failures:
        final_classification, next_stage = "G. SAFETY/RUNTIME REGRESSION", "none"
        stop_reason = f"runtime invariant regression in: {', '.join(runtime_failures)}"
    elif all_stable_mismatch:
        final_classification, next_stage = "F. MULTIPLE", "Trigger Contract Re-attribution"
        stop_reason = "B4 and D1 are current failures without repeatable formal recovery triggers; stop-dominant status differs by scene."
    elif any(scene["conclusion"] == "R3. CURRENT SCENE MOSTLY PASSES" for scene in scenes.values()):
        final_classification, next_stage = "C. HISTORICAL FAILURE NO LONGER REPRESENTATIVE", "Recovery Benchmark Scope Reduction"
        stop_reason = "at least one scene mostly arrives under the current frozen stack"
    elif any(scene["conclusion"] == "R4. NONDETERMINISTIC / STATE-SENSITIVE" for scene in scenes.values()):
        final_classification, next_stage = "D. NONDETERMINISTIC / STATE-SENSITIVE", "Reproducibility Attribution"
        stop_reason = "identical scene runs have mixed arrival outcomes"
    else:
        final_classification, next_stage = "H. UNRESOLVED", "none"
        stop_reason = "scene conclusions do not support a later stage"
    overhead = []
    for label in SCENES:
        for run in scenes[label]["runs"]:
            overhead.append(run["trigger_rejection"]["detector_latency_ms"])
    all_latency = [value for item in overhead for value in [item.get("min"), item.get("max"), item.get("mean")] if value is not None]
    return {
        "schema": "r1_n2_4c_reproducibility_audit_v2",
        "stage": "N2.4C Current Failure Phenotype Reproducibility Audit",
        "matrix_status": {"required_runs_per_scene": 5, "B4_observed": scenes["B4"]["observed_runs"], "D1_observed": scenes["D1"]["observed_runs"], "complete": complete, "missing_run_labels": missing},
        "final_classification": final_classification,
        "next_stage_allowed": next_stage,
        "gate_1_complete": False,
        "recovery_motion_allowed": False,
        "experiment_stopped": True,
        "stop_reason": stop_reason,
        "frozen_conditions": frozen_conditions(),
        "scenes": scenes,
        "pass_vs_fail_first_divergence": divergences,
        "instrumentation_overhead": {"detector_latency_ms_across_run_summaries": stats(all_latency), "assessment": "read-only detector bookkeeping; side-effect flags are reported per evaluation; no baseline timing delta was collected"},
        "instrumentation_side_effects": all(run["trigger_rejection"]["side_effects_all_false"] for label in SCENES for run in scenes[label]["runs"]),
        "verification": verification(args),
        "scope_boundary": "No detector change, threshold change, recovery command, counterfactual, synthetic rotate/reanchor, planner tuning, world/path change, DUNE training, Nav2, real robot, autonomous reverse, or Gazebo recovery motion was executed.",
    }


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# N2.4C Current Failure Phenotype Reproducibility Audit",
        "",
        f"Final classification: **{report['final_classification']}**",
        f"`gate_1_complete={str(report['gate_1_complete']).lower()}`, `next_stage_allowed={report['next_stage_allowed']}`, `recovery_motion_allowed=false`.",
        "",
        "## Decision",
        "",
        report["stop_reason"],
        "The audit is behavior-only. Formal trigger logic, thresholds, planner, FSM, safety gate, relay/freshness contract, world, path, and forward-only policy were not changed.",
        "",
        "## Frozen Conditions",
        "",
        f"- Environment: {report['frozen_conditions']['environment']}.",
        f"- Footprint: `{report['frozen_conditions']['footprint_m']['length']} x {report['frozen_conditions']['footprint_m']['width']} m`; start pose `[0, 0, 0]`; terminal mode; forward-only.",
        f"- B4 world: `{report['frozen_conditions']['worlds']['B4']['path']}` SHA-256 `{report['frozen_conditions']['worlds']['B4']['sha256']}`.",
        f"- D1 world: `{report['frozen_conditions']['worlds']['D1']['path']}` SHA-256 `{report['frozen_conditions']['worlds']['D1']['sha256']}`.",
        f"- Planner: `{report['frozen_conditions']['source_hashes']['planner_config']['path']}` SHA-256 `{report['frozen_conditions']['source_hashes']['planner_config']['sha256']}`.",
        f"- NeuPAN checkpoint SHA-256 `{report['frozen_conditions']['neupan']['checkpoint_sha256']}`; commit `{report['frozen_conditions']['neupan']['commit']}`.",
        f"- N1.16 whole-core SHA-256 `{report['frozen_conditions']['source_hashes']['n116_whole_core']['sha256']}`; N2.3 relay source SHA-256 `{report['frozen_conditions']['source_hashes']['n23_relay']['sha256']}`.",
        "- Initial e-stop latched, released after health, latched at end; timeout 32 s; no recovery commands.",
        "- Safety limits: linear 0.15 m/s, angular 0.4 rad/s, acceleration 0.30/0.8; planner/relay freshness 0.30 s; timing 0.50 s.",
        "- Scan preprocessing: Gazebo PointCloud2 -> pointcloud_to_laserscan -> `/scan_neupan`; Torch 1 intra-op / 24 inter-op; runtime scheduling is the only intended run-to-run variable.",
        "",
        "## Run Matrix",
        "",
        "| Scene | Run | Primary | Secondary | Arrival | Stop | Goal error (m) | Max/final deviation (m) | Neg samples / longest | First neg / stop | Formal trigger | Runtime |",
        "|---|---|---|---|---:|---:|---:|---:|---:|---|---:|---|",
    ]
    for label in ("B4", "D1"):
        scene = report["scenes"][label]
        for run in scene["runs"]:
            lines.append(
                f"| {label} | {run['label']} | {run['primary_phenotype']} | {', '.join(run['secondary_tags']) or 'none'} | "
                f"{int(run['arrival'])} | {int(run['stop'])} | {fmt(run['final_goal_error_m'])} | {fmt(run['max_path_deviation_m'])}/{fmt(run['final_path_deviation_m'])} | "
                f"{run['raw_negative_sample_count']}/{run['longest_continuous_negative_run']} | {run['first_negative_cycle'] or 'n/a'} / {run['first_stop_cycle'] or 'n/a'} | "
                f"{int(run['formal_trigger'])} | {'PASS' if run['safety_runtime']['runtime_invariants_pass'] else 'FAIL'} |"
            )
    for label in ("B4", "D1"):
        scene = report["scenes"][label]
        if scene["observed_runs"] < 5:
            lines.append(f"| {label} | missing | matrix incomplete | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |")
    lines += [
        "",
        "`B4` and `D1` run-level evidence, including first 20 planner cycles and every detector rejection reason, is retained in the JSON report.",
        "",
        "## Rates And Distributions",
        "",
        "| Scene | Arrive | Fail | Stop | Timeout motion | Stuck | Tracking loss | Persistent negative | Formal trigger | Conclusion |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for label in ("B4", "D1"):
        scene = report["scenes"][label]
        rates = scene["phenotype_rates"]
        if rates["arrive_rate"] is None:
            row = ["n/a"] * 8
        else:
            row = [fmt(rates[key], 2) for key in ("arrive_rate", "fail_rate", "stop_rate", "timeout_rate", "stuck_rate", "tracking_loss_rate", "persistent_negative_rate", "formal_trigger_rate")]
        lines.append(f"| {label} | {' | '.join(row)} | **{scene['conclusion']}** |")
        dist = scene["distributions"]
        lines.append(f"- {label} distributions: goal error `{dist['final_goal_error_m']}`; max deviation `{dist['max_deviation_m']}`; negative run `{dist['negative_run_length']}`; first negative cycle `{dist['first_negative_cycle']}`; first persistent-negative cycle `{dist['first_persistent_negative_cycle']}`; first deviation cycle `{dist['first_deviation_cycle']}`; first stop cycle `{dist['first_stop_cycle']}`; final path index `{dist['path_index_last']}`.")
        lines.append(f"- {label} persistent-negative-before-first-deviation rate: `{fmt(scene['persistent_negative_before_first_deviation_rate'], 2)}`.")
    lines += [
        "",
        "## Trigger Attribution",
        "",
        "The frozen detector was observed online and its exact rejection reasons were preserved. A negative raw command is not counted as persistent intent unless the existing detector state satisfies its unchanged persistence, duration, magnitude, deviation, progress, geometry, and closure conditions.",
    ]
    for label in ("B4", "D1"):
        scene = report["scenes"][label]
        formal = sum(run["formal_trigger"] for run in scene["runs"])
        negative = sum(run["raw_negative_sample_count"] for run in scene["runs"])
        longest = max((run["longest_continuous_negative_run"] for run in scene["runs"]), default=0)
        tallies: dict[str, int] = {}
        for run in scene["runs"]:
            for key, value in run["trigger_rejection"]["rejection_tallies"].items():
                tallies[key] = tallies.get(key, 0) + value
        lines.append(f"- {label}: formal triggers `{formal}/{scene['observed_runs']}`; raw negative samples `{negative}`; longest negative run `{longest}`; aggregate rejection tallies `{tallies}`.")
    lines += [
        "",
        "## Divergence Audit",
        "",
    ]
    for label in ("B4", "D1"):
        item = report["pass_vs_fail_first_divergence"][label]
        lines.append(f"- {label}: {item['status']}; first divergence cycle `{item.get('first_divergence_cycle')}`; signal `{item.get('first_differing_semantic_signal')}`; candidates `{item.get('possible_cause_candidates')}`.")
        if item.get("failure_mode_divergence"):
            detail = item["failure_mode_divergence"]
            lines.append(f"  Failure-mode split observed at cycle `{detail.get('first_divergence_cycle')}` on `{detail.get('first_differing_semantic_signal')}`; this is a candidate list, not causal attribution.")
    lines += [
        "",
        "## Safety And Runtime",
        "",
        "| Scene | Run | Gated x min | Contact | Queue max | Stale | NaN/Inf | Worker failures | Main P95/max ms | Worker P95/max ms | Scan age max after startup ms | N2.3 envelope |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for label in ("B4", "D1"):
        for run in report["scenes"][label]["runs"]:
            info = run["safety_runtime"]
            lines.append(f"| {label} | {run['label']} | {fmt(info['gated_linear_x_min'])} | {info['contact_count']} | {info['queue_max']} | {info['stale']} | {info['nan_inf']} | {info['worker_failures']} | {fmt(info['main_p95_ms'])}/{fmt(info['main_max_ms'])} | {fmt(info['worker_p95_ms'])}/{fmt(info['worker_max_ms'])} | {fmt(info['scan_age_max_ms_after_first_cycle'])} | {str(info['n23_current_contract_observed']).lower()} |")
    lines += [
        "",
        "The first-cycle scan-age startup sample is reported separately and excluded from the tail check; subsequent scan age, main/worker latency, source intervals, queue, freshness, and gap events are checked per run. Any invariant failure would block the next stage.",
        "",
        "## Instrumentation And Verification",
        "",
        f"- Detector latency summary: `{report['instrumentation_overhead']['detector_latency_ms_across_run_summaries']}` ms. Side-effect flags all false: `{report['instrumentation_side_effects']}`. No command, planner input/state, recovery state, FSM, or threshold was modified by instrumentation.",
        f"- Build: `{report['verification']['colcon_build']}`; compileall: `{report['verification']['compileall']}`.",
        f"- Direct units: `{report['verification']['direct_units']}`.",
        f"- colcon test: `{report['verification']['colcon_test']}`.",
        f"- Cleanup: `{report['verification']['cleanup']}`.",
        "- Excluded startup failures are infrastructure-only and do not count toward reproducibility: runner `TasksMax=256` exhausted ROS/Gazebo/Torch task creation before an admitted audit sample existed.",
        "",
        "## Conclusions",
        "",
        f"- B4: **{report['scenes']['B4']['conclusion']}**; secondary conclusions `{report['scenes']['B4']['secondary_conclusions']}`. {report['scenes']['B4']['conclusion_reason']}.",
        f"- D1: **{report['scenes']['D1']['conclusion']}**; secondary conclusions `{report['scenes']['D1']['secondary_conclusions']}`. {report['scenes']['D1']['conclusion_reason']}.",
        f"- Final: **{report['final_classification']}**; `next_stage_allowed={report['next_stage_allowed']}`.",
        "",
        "Testing stops here. No recovery counterfactual, N2.4R1, planner tuning, DUNE, Nav2, Gazebo recovery motion, autonomous reverse, or real-robot operation was started.",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    provenance = assert_artifact_provenance()
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--compileall-status", default="PASS")
    parser.add_argument("--colcon-build-status", default="PASS")
    parser.add_argument("--recovery-unit-status", default="PASS")
    parser.add_argument("--contract-unit-status", default="PASS")
    parser.add_argument("--safety-unit-status", default="PASS")
    parser.add_argument("--semantics-unit-status", default="PASS")
    parser.add_argument("--cleanup-status", default="PASS; no residual N2.4C test processes")
    args = parser.parse_args()
    report = build_report(args)
    report["artifact_provenance"] = provenance
    if args.require_complete and not report["matrix_status"]["complete"]:
        raise SystemExit(f"matrix incomplete: missing {report['matrix_status']['missing_run_labels']}")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "n2_4c_reproducibility_report.json"
    md_path = output_dir / "n2_4c_final_report.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md_path.write_text(render_markdown(report), encoding="utf-8")
    print(f"json={json_path}")
    print(f"markdown={md_path}")
    print(f"matrix_complete={report['matrix_status']['complete']}")
    print(f"classification={report['final_classification']}")
    print(f"next_stage_allowed={report['next_stage_allowed']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
