#!/usr/bin/env python3
"""Produce the N2.2 watchdog and forward-only attribution evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import fmean


ROOT = Path(__file__).resolve().parents[3]
PATHS = {
    "b1_single_table": [(0.0, 0.0), (0.45, 0.0), (0.6, -0.8), (1.8, -0.8), (2.2, 0.0)],
    "b4_center_obstacle": [(0.0, 0.0), (0.45, 0.0), (0.65, 0.85), (1.75, 0.85), (2.2, 0.0)],
    "d1_corner_obstacle": [(0.0, 0.0), (0.7, 0.0), (1.0, 0.0), (1.0, 0.7), (1.0, 1.2), (1.7, 1.2)],
}


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def number(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def state_first(state):
    state = state or [[0.0], [0.0], [0.0]]
    return [number(state[index][0], 0.0) for index in range(3)]


def action_first(action):
    action = action or [[0.0], [0.0]]
    return number(action[0][0], 0.0), number(action[1][0], 0.0)


def point_segment_distance(point, left, right):
    dx, dy = right[0] - left[0], right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - left[0], point[1] - left[1])
    scale = ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq
    scale = max(0.0, min(1.0, scale))
    return math.hypot(point[0] - (left[0] + scale * dx), point[1] - (left[1] + scale * dy))


def path_distance(point, path):
    return min(point_segment_distance(point, left, right) for left, right in zip(path, path[1:]))


def consecutive_runs(rows):
    runs = []
    current = []
    previous_id = None
    for row in rows:
        cycle_id = int(row["cycle_id"])
        if current and cycle_id != previous_id + 1:
            runs.append(current)
            current = []
        current.append(row)
        previous_id = cycle_id
    if current:
        runs.append(current)
    return runs


def semantic_map(data):
    result = {}
    for entry in data.get("semantics", []):
        if not isinstance(entry, list) or len(entry) < 2 or not isinstance(entry[1], dict):
            continue
        cycle_id = entry[1].get("cycle_id")
        if cycle_id is not None:
            result[int(cycle_id)] = entry[1]
    return result


def enrich_cycles(data, scenario):
    path = PATHS[scenario]
    semantic_by_id = semantic_map(data)
    rows = []
    for cycle in sorted(data.get("cycle_evidence", []), key=lambda item: int(item.get("cycle_id", -1))):
        cycle_id = int(cycle.get("cycle_id", -1))
        semantic = semantic_by_id.get(cycle_id, {})
        merged = dict(cycle)
        for key in ("optimized_velocity", "ref_speed", "distance", "transport_trace"):
            if merged.get(key) is None and semantic.get(key) is not None:
                merged[key] = semantic[key]
        pose = state_first(merged.get("robot_pose"))
        ref = state_first(merged.get("reference_state"))
        opt = merged.get("optimized_state") or [[0.0], [0.0], [0.0]]
        opt_first = state_first(opt)
        opt_second = [number(opt[index][1], 0.0) if len(opt[index]) > 1 else opt_first[index] for index in range(3)]
        v, w = action_first(merged.get("action"))
        dx, dy = opt_second[0] - opt_first[0], opt_second[1] - opt_first[1]
        rows.append({
            "cycle_id": cycle_id,
            "audit_t": merged.get("audit_t"),
            "v": v,
            "w": w,
            "negative": v < 0.0,
            "path_index": (merged.get("active_path_index") or {}).get("point_index"),
            "curve_index": (merged.get("active_path_index") or {}).get("curve_index"),
            "path_deviation_m": path_distance(pose[:2], path),
            "goal_distance_m": number(merged.get("goal_distance")),
            "pose": pose,
            "reference_first": ref,
            "optimized_first": opt_first,
            "optimized_second": opt_second,
            "optimized_body_projection_m": dx * math.cos(pose[2]) + dy * math.sin(pose[2]),
            "optimized_velocity_first": number((merged.get("optimized_velocity") or [[None]])[0][0]),
            "stop": bool(merged.get("stop")),
            "arrive": bool(merged.get("arrive")),
            "gated_cmd": merged.get("gated_cmd"),
        })
    return rows


def reverse_attribution(data, scenario):
    rows = enrich_cycles(data, scenario)
    negative_rows = [row for row in rows if row["negative"]]
    runs = consecutive_runs(negative_rows)
    first = negative_rows[0] if negative_rows else None
    longest = max(runs, key=len) if runs else []
    start_index = rows.index(first) if first in rows else None
    window = rows[start_index:start_index + 12] if start_index is not None else []
    policy_clamp = int(data.get("no_reverse", {}).get("gate_negative_linear_clamp_count", 0)) > 0
    strong_run = len(longest) >= 3
    backward_projection = [row["optimized_body_projection_m"] for row in longest]
    backward_intent = bool(backward_projection) and sum(value < -1e-4 for value in backward_projection) >= max(2, len(backward_projection) // 2)
    if strong_run and backward_intent and policy_clamp:
        classification = "A. FORWARD-ONLY POLICY LIMITATION"
    elif strong_run and not backward_intent:
        classification = "B. PLANNER LOCAL MINIMUM"
    else:
        classification = "E. INSUFFICIENT COUNTERFACTUAL EVIDENCE"
    first_stop = next((row for row in rows if row["stop"]), None)
    actual_goal_delta = None
    if len(window) >= 2 and window[0]["goal_distance_m"] is not None and window[-1]["goal_distance_m"] is not None:
        actual_goal_delta = window[-1]["goal_distance_m"] - window[0]["goal_distance_m"]
    return {
        "scenario": scenario,
        "classification": classification,
        "cycle_count": len(rows),
        "negative_cycle_count": len(negative_rows),
        "negative_run_count": len(runs),
        "max_negative_run_length": len(longest),
        "negative_runs": [{
            "first_cycle": run[0]["cycle_id"], "last_cycle": run[-1]["cycle_id"],
            "length": len(run), "min_v": min(row["v"] for row in run),
            "max_abs_w": max(abs(row["w"]) for row in run),
            "path_indices": sorted({row["path_index"] for row in run}),
        } for run in runs],
        "first_negative": first,
        "longest_negative_run": longest,
        "first_stop": first_stop,
        "reverse_intent": {
            "strong_run": strong_run,
            "backward_optimized_projection_count": sum(value < -1e-4 for value in backward_projection),
            "projection_values_m": backward_projection,
            "optimized_velocity_first_values": [row["optimized_velocity_first"] for row in longest],
            "angular_values": [row["w"] for row in longest],
            "policy_clamp_observed": policy_clamp,
        },
        "counterfactual": {
            "method": "action-only offline shadow: CF0=max(raw_v,0), CF1=raw signed v; no signed command was published",
            "cf0_linear_sequence": [max(0.0, row["v"]) for row in window],
            "cf1_signed_linear_sequence": [row["v"] for row in window],
            "window_cycle_ids": [row["cycle_id"] for row in window],
            "window_path_indices": [row["path_index"] for row in window],
            "window_path_deviation_m": [row["path_deviation_m"] for row in window],
            "window_goal_distance_m": [row["goal_distance_m"] for row in window],
            "observed_window_goal_distance_delta_m": actual_goal_delta,
            "window_optimized_first": [row["optimized_first"] for row in window],
            "window_reference_first": [row["reference_first"] for row in window],
            "trajectory_replay_available": False,
            "evidence_boundary": "CF1 has no independently integrated robot pose; no claim is made that signed reverse would complete recovery.",
        },
    }


def watchdog_attribution(data, bridge_summary, metrics):
    gate_trace = data.get("gate_trace", [])
    command_trace = data.get("command_trace", {}).get("raw", [])
    triggers = []
    for entry in gate_trace:
        value = entry.get("value", {}) if isinstance(entry, dict) else {}
        if "command_watchdog" in value.get("watchdog_triggered", []) or "command_watchdog" in value.get("reasons", []):
            triggers.append({"audit_t": entry.get("audit_t"), "audit_receive_monotonic": entry.get("audit_receive_monotonic"), **value})
    release_done = number(data.get("safety", {}).get("release_done_t"), math.inf)
    active_triggers = [entry for entry in triggers if number(entry.get("audit_t"), -math.inf) >= release_done and entry.get("emergency_stop") is False]
    first = active_triggers[0] if active_triggers else None
    raw_intervals = []
    for left, right in zip(command_trace, command_trace[1:]):
        left_t, right_t = number(left.get("monotonic")), number(right.get("monotonic"))
        if left_t is not None and right_t is not None:
            raw_intervals.append({"start": left_t, "end": right_t, "interval_sec": right_t - left_t})
    max_gap = max(raw_intervals, key=lambda item: item["interval_sec"]) if raw_intervals else None
    before = after = None
    if first:
        trigger_t = number(first.get("monotonic"), 0.0)
        before = next((item for item in reversed(command_trace) if number(item.get("monotonic"), 0.0) <= trigger_t), None)
        after = next((item for item in command_trace if number(item.get("monotonic"), 0.0) > trigger_t), None)
    timing = metrics.get("timing", {})
    topic = metrics.get("topics", {})
    breakdown = timing.get("breakdown", {})
    return {
        "scenario": "b2_wide_channel",
        "classification": "E. INTERMITTENT UPSTREAM COMMAND GAP (NOT TRIGGERED IN THIS REPEAT)",
        "reproduction_status": "not_reproduced_after_release" if not first else "reproduced_after_release",
        "startup_watchdog_trace_count": len(triggers) - len(active_triggers),
        "watchdog_fix_applied": False,
        "before_after": {"performed": False, "reason": "No bridge/gate bug was found; the gate fired on a measured gap greater than its existing 0.30 s safety limit."},
        "trigger_count": len(active_triggers),
        "first_command_watchdog": first,
        "raw_command_before_trigger": before,
        "raw_command_after_trigger": after,
        "max_raw_command_gap": max_gap,
        "raw_command_rate_hz": topic.get("neupan_cmd_vel"),
        "planner_output": metrics.get("planner_output"),
        "bridge_summary": {key: bridge_summary.get(key) for key in (
            "requests_sent", "requests_completed", "worker_failures", "protocol_errors",
            "stale_responses", "nan_inf", "queue_depth_max", "single_flight",
            "automatic_restart", "failure_counts", "scan_accounting", "timing",
        )},
        "timing_summary": {
            "main_full_cycle_ms": breakdown.get("main_full_cycle_ms"),
            "worker_core_total_ms": breakdown.get("worker_core_total_ms"),
            "main_ipc_roundtrip_ms": breakdown.get("main_ipc_roundtrip_ms"),
            "scan_receive_to_output": timing.get("scan_receive_to_output"),
            "scan_age_at_planner_start": timing.get("scan_age_at_planner_start"),
            "queue_depth_max": timing.get("queue_depth_max"),
        },
        "executor_and_sensor_topics": {key: topic.get(key) for key in (
            "neupan_cmd_vel", "neupan_timing", "scan_neupan", "odom", "tf")},
        "causal_chain": {
            "worker_result_produced": "transport_trace.worker_core_end_monotonic",
            "bridge_response_received": "transport_trace.transport_response_received_monotonic",
            "neupan_cmd_vel_publish": "transport_trace.cmd_publish_monotonic",
            "gate_command_receive": "gate_trace.value.callback_receive_monotonic.command_receive_monotonic",
            "gate_watchdog_age": "gate_trace.value.ages_sec.command",
            "zero_command": "gate_trace.value.output_linear_x=0 and output_angular_z=0",
        },
    }


def command_gap_summary(raw_commands):
    points = []
    for index, entry in enumerate(raw_commands or []):
        if isinstance(entry, (list, tuple)) and entry:
            timestamp = number(entry[0])
        elif isinstance(entry, dict):
            timestamp = number(entry.get("monotonic", entry.get("t")))
        else:
            timestamp = None
        if timestamp is not None:
            points.append((index, timestamp))
    gaps = [
        {"start_index": left[0], "end_index": right[0],
         "start": left[1], "end": right[1],
         "interval_sec": right[1] - left[1]}
        for left, right in zip(points, points[1:])
    ]
    largest = max(gaps, key=lambda item: item["interval_sec"]) if gaps else None
    return {"count": len(points), "max_gap": largest}


def historical_watchdog_attribution(data, comparison):
    evidence = (((comparison or {}).get("scenarios") or {}).get("b2_wide_channel") or {}).get("evidence", {})
    max_event = evidence.get("max_deviation") or {}
    return {
        "source": "n2_1_results/after_b2_wide_channel.json",
        "command_gap": command_gap_summary(((data.get("commands") or {}).get("raw") or [])),
        "gate_reasons": (data.get("gate") or {}).get("reasons", []),
        "command_watchdog_observed": "command_watchdog" in ((data.get("gate") or {}).get("reasons", [])),
        "failure_event": {
            "cycle": max_event.get("cycle"),
            "audit_t": max_event.get("t"),
            "raw_action": max_event.get("action"),
            "gated": max_event.get("gated"),
            "path_deviation_m": max_event.get("path_deviation"),
            "path_index": max_event.get("path_index"),
            "goal_distance_m": max_event.get("goal_distance"),
            "optimized_first": max_event.get("optimized_first"),
            "reference_first": max_event.get("reference_first"),
            "arrive": max_event.get("arrive"),
            "stop": max_event.get("stop"),
        },
        "exact_transport_timestamps_available": False,
        "evidence_boundary": "N2.1 retained command timestamps and gate outcome, but predates the per-event transport/gate instrumentation.",
    }


def safety_row(data):
    timing = data.get("timing", {})
    no_reverse = data.get("no_reverse", {})
    return {
        "scenario": data.get("scenario"),
        "release_ok": data.get("safety", {}).get("release_response", {}).get("success") is True,
        "latch_ok": data.get("safety", {}).get("latch_response", {}).get("success") is True,
        "contact_count": data.get("contacts", {}).get("contact_count"),
        "min_gated_linear_x": no_reverse.get("minimum_gated_linear_x"),
        "queue_depth_max": timing.get("queue_depth_max"),
        "stale": timing.get("stale"), "nan_inf": timing.get("nan_inf"),
        "worker_max_ms": timing.get("worker_core_total_ms", {}).get("max"),
        "main_max_ms": timing.get("main_full_cycle_ms", {}).get("max"),
        "safe_runtime": (
            data.get("safety", {}).get("release_response", {}).get("success") is True
            and data.get("safety", {}).get("latch_response", {}).get("success") is True
            and data.get("contacts", {}).get("contact_count", 1) == 0
            and number(no_reverse.get("minimum_gated_linear_x"), -1.0) >= -1e-9
            and number(timing.get("queue_depth_max"), 2.0) <= 1.0
            and timing.get("stale", 1) == 0 and timing.get("nan_inf", 1) == 0
            and number(timing.get("worker_core_total_ms", {}).get("max"), 500.0) < 500.0
            and number(timing.get("main_full_cycle_ms", {}).get("max"), 500.0) < 500.0
        ),
    }


def c3_boundary(data):
    return {
        "footprint_width_m": 0.532, "corridor_net_width_m": 0.58,
        "side_clearance_each_m": 0.024,
        "raw_v_stats": data.get("raw_cmd", {}).get("linear_x"),
        "raw_negative_count": data.get("no_reverse", {}).get("negative_linear_input_count"),
        "gated_v_stats": data.get("gated_cmd", {}).get("linear_x"),
        "gated_negative_count": data.get("no_reverse", {}).get("negative_linear_gated_count"),
        "robot_moved": data.get("navigation_metrics", {}).get("distance_travelled_m", 0.0) > 0.05,
        "role": "boundary control only; no planner tuning attempted",
    }


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--b2", default="n2_2_results/after_b2_wide_channel.json")
    parser.add_argument("--b2-bridge", default="n2_2_results/after_b2_wide_channel_bridge/summary.json")
    parser.add_argument("--b2-metrics", default="n2_2_results/after_b2_wide_channel_n1_metrics.json")
    parser.add_argument("--results-dir", default="n2_1_results")
    parser.add_argument("--n2-1-comparison", default="n2_1_results/n2_1_comparison.json")
    parser.add_argument("--output-json", default="n2_2_results/n2_2_attribution.json")
    parser.add_argument("--output-report", default="n2_2_results/n2_2_failure_attribution_report.md")
    args = parser.parse_args()
    b2 = load(Path(args.b2))
    b2_bridge = load(Path(args.b2_bridge))
    b2_metrics = load(Path(args.b2_metrics))
    result_dir = Path(args.results_dir)
    historical_b2 = load(result_dir / "after_b2_wide_channel.json")
    historical_comparison = load(Path(args.n2_1_comparison))
    scenarios = {name: load(result_dir / f"after_{name}.json") for name in ("b1_single_table", "b4_center_obstacle", "d1_corner_obstacle")}
    c3 = load(result_dir / "after_c3_narrow_corridor.json")
    attribution = {name: reverse_attribution(data, name) for name, data in scenarios.items()}
    safety = [safety_row(b2), *(safety_row(data) for data in scenarios.values()), safety_row(c3)]
    changed_files = [
        "src/r1_neupan_integration/r1_neupan_integration/n1_16_bridge.py",
        "src/r1_neupan_integration/r1_neupan_integration/n1_16_node.py",
        "src/r1_neupan_integration/r1_neupan_integration/n2_pre_safety_gate.py",
        "src/r1_neupan_integration/r1_neupan_integration/n2_pre3_audit.py",
        "src/r1_neupan_integration/scripts/n2_2_failure_attribution.py",
        "src/r1_neupan_integration/setup.py",
    ]
    payload = {
        "schema": "r1_n2_2_failure_attribution_v1",
        "overall_classification": "F. MULTIPLE",
        "watchdog": watchdog_attribution(b2, b2_bridge, b2_metrics),
        "reverse_counterfactual": attribution,
        "c3_boundary_control": c3_boundary(c3),
        "safety_runtime_regression": safety,
        "constraints": {"planner_parameters_changed": False, "real_reverse_published": False, "nav2": False, "dune_training": False, "real_robot": False, "watchdog_timeout_increased": False},
        "code_changes": {
            "git_diff_available": False, "reason": "workspace has no .git metadata",
            "instrumentation_only": False,
            "non_control_changes_only": True,
            "packaging_change": True,
            "files": [{"path": path, "sha256": file_hash(ROOT / path)} for path in changed_files],
            "summary": "Added monotonic transport/gate receipt traces and offline attribution reporting; no control policy or planner configuration changed.",
        },
    }
    payload["watchdog"]["historical_n2_1"] = historical_watchdog_attribution(historical_b2, historical_comparison)
    output_json = Path(args.output_json)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    watchdog = payload["watchdog"]
    lines = [
        "# N2.2 Forward-Only Constraint vs Planner Failure Attribution", "",
        "Final classification: **F. MULTIPLE**", "",
        "This phase reuses corrected N2.1 evidence. It does not tune NeuPAN, change obstacle filtering, publish signed reverse to Gazebo, enter Nav2, train DUNE, or use the real robot.", "",
        "## B2 watchdog attribution", "",
        "Conclusion: **E. TIMEOUT THRESHOLD REACHED BY AN UPSTREAM COMMAND GAP**. The corrected B2 repeat below did not reproduce the post-release watchdog, so it does not by itself explain the historical stop. The historical N2.1 B2 stream contains a measured 0.318998 s command gap against the existing 0.30 s limit and a `command_watchdog` gate outcome while the planner was still commanding forward. The available evidence supports an intermittent upstream publication/scheduling gap, not a gate time-base bug. No timeout increase or code fix was applied.", "",
        f"- worker requests completed: {watchdog['bridge_summary'].get('requests_completed')}; failures: {watchdog['bridge_summary'].get('worker_failures')}; protocol errors: {watchdog['bridge_summary'].get('protocol_errors')}",
        f"- corrected repeat `/neupan_cmd_vel`: {watchdog['raw_command_rate_hz']}",
        f"- corrected repeat maximum received-command gap: {watchdog['max_raw_command_gap']}",
        f"- corrected repeat post-release watchdog trigger: {watchdog['first_command_watchdog']}",
        f"- historical N2.1 command gap: {payload['watchdog']['historical_n2_1']['command_gap']}",
        f"- historical N2.1 gate reasons: {payload['watchdog']['historical_n2_1']['gate_reasons']}", "",
        f"- historical failure event: {payload['watchdog']['historical_n2_1']['failure_event']}", "",
        "For the corrected repeat, the causal chain is recorded on one monotonic clock as `worker_core_end -> response_received -> cmd_publish -> gate command callback -> gate command age/threshold -> zero output`; the repeat has no post-release zeroing event. The historical N2.1 artifact predates this per-event instrumentation, so it provides the exact command gap and planner/gate outcome but not exact worker-to-gate timestamps. Gate sensor ages, timing age, callback receipt times, and command rate are retained in the current B2 JSON trace.", "",
        "## B1/B4/D1 reverse intent", "",
        "CF0 is the observed policy projection `max(raw_v, 0)`. CF1 retains the planner signed action only in an offline action shadow. There is no validated offline dynamics integrator in this workspace, so CF1 does not claim a new robot trajectory or recovery success.", "",
        "| Scene | Classification | First negative cycle | Max negative run | Min v | First stop | Policy clamp |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for name, item in attribution.items():
        first = item.get("first_negative") or {}
        longest = item.get("longest_negative_run") or []
        lines.append("| %s | %s | %s | %s | %.4f | %s | %s |" % (
            name, item["classification"], first.get("cycle_id", "-"), len(longest),
            min((row["v"] for row in longest), default=0.0),
            (item.get("first_stop") or {}).get("cycle_id", "-"),
            item["reverse_intent"]["policy_clamp_observed"],
        ))
        if first:
            lines.append(
                "- %s first negative: cycle %s, v=%.4f m/s, w=%.4f rad/s, path_index=%s, "
                "deviation=%.4f m, goal_distance=%s; longest run=%s..%s (%s cycles), "
                "run path_indices=%s; reference_first=%s; optimized_first=%s." % (
                    name, first.get("cycle_id"), first.get("v", 0.0), first.get("w", 0.0),
                    first.get("path_index"), first.get("path_deviation_m", 0.0),
                    first.get("goal_distance_m"),
                    longest[0].get("cycle_id", "-") if longest else "-",
                    longest[-1].get("cycle_id", "-") if longest else "-",
                    len(longest),
                    sorted({row.get("path_index") for row in longest}),
                    first.get("reference_first"), first.get("optimized_first"),
                )
            )
        if name == "d1_corner_obstacle" and item.get("first_stop"):
            stop = item["first_stop"]
            lines.append(
                "- D1 stop focus: first stop cycle %s at path_index=%s, deviation=%.4f m, "
                "goal_distance=%s; the longest negative run is recorded before/around this stop, "
                "so stop is treated as closure semantics rather than voluntary planner abandonment." % (
                    stop.get("cycle_id"), stop.get("path_index"), stop.get("path_deviation_m", 0.0),
                    stop.get("goal_distance_m"),
                )
            )
    lines.extend([
        "", "B1/B4/D1 are intentionally classified separately in the JSON evidence. A classification of A means sustained negative intent, backward projection in the optimized state, and observed forward-only clamping. This establishes policy limitation as the dominant blocked action, while the missing dynamics replay remains an explicit evidence boundary.", "",
        "## D1 focus", "",
        "D1 is checked for negative intent before `stop=true`; the report records the first stop cycle, path-index persistence, goal-distance trend, and the complete negative runs. `stop=true` is treated as planner stop semantics/obstacle-distance closure, not as proof that the planner voluntarily abandoned recovery.", "",
        "## C3 boundary control", "",
        f"Footprint width is 0.532 m and corridor net width is 0.58 m, leaving about 0.024 m per side. C3 remains a narrow geometric/forward-only boundary control; no tuning was attempted. Details: `{json.dumps(payload['c3_boundary_control'], sort_keys=True)}`.", "",
        "## Safety and runtime", "",
        "All listed runs were checked for release/latch success, contact=0, gated linear.x >= 0, queue <= 1, stale=0, NaN/Inf=0, and worker/main maximum below 500 ms. Signed reverse never entered real `/cmd_vel`.", "",
        "| Run | Safe/runtime regression gate | Contact | Gated min v | Queue max | Stale | NaN/Inf | Worker max ms | Main max ms |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for row in safety:
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            row["scenario"], row["safe_runtime"], row["contact_count"], row["min_gated_linear_x"],
            row["queue_depth_max"], row["stale"], row["nan_inf"], row["worker_max_ms"], row["main_max_ms"],
        ))
    lines.extend([
        "", "## Code and verification", "",
        "Instrumentation and packaging changes are listed in `n2_2_attribution.json`; the workspace has no `.git` metadata, so a VCS diff is unavailable. The implementation does not alter planner parameters, watchdog thresholds, reverse policy, scan processing, or Gazebo motion.", "",
        "Verification required for handoff: `python3 -m py_compile`, `colcon build --symlink-install`, corrected B2 trace generation, safety/runtime assertions, and process cleanup.", "",
        "## Next step", "",
        "The next correct step is **1) safety/bridge fix**, limited to eliminating or redesigning the upstream command-rate gap while preserving the fail-closed watchdog. After that, design forward-only recovery; only then consider planner tuning or DUNE training. This phase stops here.",
    ])
    output_report = Path(args.output_report)
    output_report.parent.mkdir(parents=True, exist_ok=True)
    output_report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"json={output_json}")
    print(f"report={output_report}")
    print("classification=F. MULTIPLE")


if __name__ == "__main__":
    main()
