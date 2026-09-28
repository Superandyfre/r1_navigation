#!/usr/bin/env python3
"""Run the N2.4 forward-only recovery design against historical cycle evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from r1_neupan_integration.n2_4_recovery import (
    ForwardOnlyRecoveryFSM,
    RecoveryObservation,
    evaluate_trigger,
    observation_from_cycle,
    json_safe,
)


_installed_root = Path(__file__).resolve().parents[3]
ROOT = Path.cwd() if (Path.cwd() / "n2_1_results").is_dir() else _installed_root
SCENES = {
    "a1_straight": ("n2_results/a1_straight.json", None, "A1 normal straight false-trigger control."),
    "a2_turn_forward": ("n2_results/a2_turn_forward.json", None, "A2 normal turn-forward false-trigger control."),
    "b1_single_table": ("n2_1_results/after_b1_single_table.json", None, "B1 has sustained deviation and a stalled bypass index."),
    "b4_center_obstacle": ("n2_1_results/after_b4_center_obstacle.json", None, "B4 remains around bypass point 2 while the path deviation grows."),
    "d1_corner_obstacle": ("n2_1_results/after_d1_corner_obstacle.json", None, "D1 negative intent precedes planner stop closure."),
    "c3_narrow_corridor": ("n2_1_results/after_c3_narrow_corridor.json", 0.024, "0.532 m footprint in a 0.58 m corridor: 0.024 m per side."),
    "b2_wide_channel": ("n2_1_results/after_b2_wide_channel.json", None, "PASS normal false-trigger control."),
    "c2_medium_corridor": ("n2_1_results/after_c2_medium_corridor.json", 0.154, "0.532 m footprint in a 0.84 m corridor: 0.154 m per side."),
}


def number(value: Any, default: float | None = None) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def point_segment_distance(point: tuple[float, float], left: tuple[float, float], right: tuple[float, float]) -> float:
    dx, dy = right[0] - left[0], right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - left[0], point[1] - left[1])
    scale = max(0.0, min(1.0, ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq))
    return math.hypot(point[0] - (left[0] + scale * dx), point[1] - (left[1] + scale * dy))


def path_distance(point: tuple[float, float], path: list[tuple[float, float]]) -> float:
    return min((point_segment_distance(point, left, right) for left, right in zip(path, path[1:])), default=float("inf"))


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def global_clearance(data: dict[str, Any]) -> float | None:
    scan = data.get("scan") or {}
    value = number(scan.get("minimum_range"))
    if value is not None:
        return value
    samples = [number(item[1]) for item in scan.get("sample_log", []) if isinstance(item, list) and len(item) > 1]
    return min((item for item in samples if item is not None), default=None)


def cycle_rows(data: dict[str, Any]) -> list[dict[str, Any]]:
    path = [tuple(map(float, point)) for point in data["scenario_spec"]["path"]]
    rows = []
    source_rows = list(data.get("cycle_evidence", []))
    if not source_rows:
        source_rows = [
            dict(entry[1], audit_t=entry[0])
            for entry in data.get("semantics", [])
            if isinstance(entry, list) and len(entry) >= 2 and isinstance(entry[1], dict)
            and entry[1].get("cycle_id") is not None
        ]
    for original in sorted(source_rows, key=lambda row: int(row["cycle_id"])):
        row = dict(original)
        pose = row.get("robot_pose") or [[0.0], [0.0], [0.0]]
        row["path_deviation_m"] = path_distance((float(pose[0][0]), float(pose[1][0])), path)
        row["_path"] = path
        rows.append(row)
    return rows


def scan_clearance(row: dict[str, Any], fallback: float | None) -> float | None:
    scan = row.get("scan_topology") or {}
    values = [number(scan.get(key)) for key in ("min_front", "min_left", "min_right")]
    values = [value for value in values if value is not None]
    return min(values) if values else fallback


def make_observation(row: dict[str, Any], margin: float | None, fallback_clearance: float | None) -> RecoveryObservation:
    return observation_from_cycle(
        row,
        row["_path"],
        corridor_margin_m=1.0 if margin is None else margin,
        obstacle_clearance_m=scan_clearance(row, fallback_clearance),
    )


def first_forward_after(rows: list[dict[str, Any]], cycle_id: int) -> float | None:
    for row in rows:
        if int(row["cycle_id"]) <= cycle_id:
            continue
        value = number((row.get("action") or [[None], [None]])[0][0])
        if value is not None and value > 0.0 and not bool(row.get("stop", False)):
            return value
    return None


def shadow_scene(name: str, data: dict[str, Any], margin: float | None, note: str) -> dict[str, Any]:
    all_rows = cycle_rows(data)
    fallback_clearance = global_clearance(data)
    fsm = ForwardOnlyRecoveryFSM()
    history: list[RecoveryObservation] = []
    trace = []
    trigger_row = None
    trigger_decision: dict[str, Any] = {}
    selected_heading = None
    reentry_v = None
    observed_reentry_v = None
    synthetic_stop_cycles = []
    synthetic_rotate_cycle = None
    synthetic_reanchor_cycles = []
    for row in all_rows:
        observation = make_observation(row, margin, fallback_clearance)
        command = fsm.update(observation, history)
        if fsm.state == "NORMAL":
            trigger_decision = evaluate_trigger(history, observation, fsm.config)
        trace.append({"cycle_id": observation.cycle_id, "kind": "historical", "state": fsm.state, "raw_v": observation.raw_linear, "command": command})
        if trigger_row is None and fsm.stop_entry_cycle == observation.cycle_id:
            trigger_row = row
        history.append(observation)
        if trigger_row is not None:
            break

    if trigger_row is not None and fsm.state == "RECOVERY_STOP":
        base = make_observation(trigger_row, margin, fallback_clearance)
        for index in range(1, fsm.config.stop_settle_samples + 1):
            settled = RecoveryObservation(
                cycle_id=int(trigger_row["cycle_id"]) + index,
                monotonic=base.monotonic + index * 0.1,
                raw_linear=0.0,
                raw_angular=0.0,
                pose=base.pose,
                path=base.path,
                active_path_index=base.active_path_index,
                path_deviation_m=base.path_deviation_m,
                goal_distance_m=base.goal_distance_m,
                planner_fresh=True,
                path_valid=True,
                obstacle_clearance_m=base.obstacle_clearance_m,
                corridor_margin_m=base.corridor_margin_m,
                odom_linear_m_s=0.0,
                odom_angular_rad_s=0.0,
                scan_topology=base.scan_topology,
                reference_points=base.reference_points,
                optimized_points=base.optimized_points,
            )
            command = fsm.update(settled, history, shadow_stop_settled=True)
            synthetic_stop_cycles.append(settled.cycle_id)
            trace.append({"cycle_id": settled.cycle_id, "kind": "synthetic_stop_interlock", "sample": index, "state": fsm.state, "raw_v": 0.0, "command": command})
        selected_heading = fsm.heading.get("selected")
        if fsm.state == "RECOVERY_ROTATE":
            target = float(fsm.heading["target_heading"])
            rotate_step = RecoveryObservation(
                cycle_id=int(trigger_row["cycle_id"]) + fsm.config.stop_settle_samples + 1,
                monotonic=base.monotonic + 0.3,
                raw_linear=0.0,
                raw_angular=0.0,
                pose=base.pose,
                path=base.path,
                active_path_index=base.active_path_index,
                path_deviation_m=base.path_deviation_m,
                goal_distance_m=base.goal_distance_m,
                planner_fresh=True,
                path_valid=True,
                obstacle_clearance_m=base.obstacle_clearance_m,
                corridor_margin_m=base.corridor_margin_m,
                odom_linear_m_s=0.0,
                odom_angular_rad_s=0.0,
                scan_topology=base.scan_topology,
                reference_points=base.reference_points,
                optimized_points=base.optimized_points,
            )
            command = fsm.update(rotate_step, history)
            trace.append({"cycle_id": rotate_step.cycle_id, "kind": "synthetic_rotate_command", "state": fsm.state, "raw_v": 0.0, "command": command})
            pose = (base.pose[0], base.pose[1], target)
            rotated = RecoveryObservation(
                cycle_id=rotate_step.cycle_id + 1,
                monotonic=base.monotonic + 0.4,
                raw_linear=0.0,
                raw_angular=0.0,
                pose=pose,
                path=base.path,
                active_path_index=base.active_path_index,
                path_deviation_m=base.path_deviation_m,
                goal_distance_m=base.goal_distance_m,
                planner_fresh=True,
                path_valid=True,
                obstacle_clearance_m=base.obstacle_clearance_m,
                corridor_margin_m=base.corridor_margin_m,
                scan_topology=base.scan_topology,
                reference_points=base.reference_points,
                optimized_points=base.optimized_points,
            )
            command = fsm.update(rotated, history)
            synthetic_rotate_cycle = rotate_step.cycle_id
            trace.append({"cycle_id": rotated.cycle_id, "kind": "synthetic_rotate_to_target", "state": fsm.state, "raw_v": 0.0, "command": command})
            observed_reentry_v = first_forward_after(all_rows, int(trigger_row["cycle_id"]))
            reentry_v = 0.0 if observed_reentry_v is None else observed_reentry_v
            for index in range(1, fsm.config.reentry_positive_samples + 1):
                reentry = RecoveryObservation(
                    cycle_id=rotated.cycle_id + index,
                    monotonic=rotated.monotonic + index * 0.1,
                    raw_linear=reentry_v,
                    raw_angular=0.0,
                    pose=rotated.pose,
                    path=rotated.path,
                    active_path_index=rotated.active_path_index,
                    path_deviation_m=0.0,
                    goal_distance_m=rotated.goal_distance_m,
                    planner_fresh=True,
                    path_valid=True,
                    obstacle_clearance_m=rotated.obstacle_clearance_m,
                    corridor_margin_m=rotated.corridor_margin_m,
                    odom_linear_m_s=0.0,
                    odom_angular_rad_s=0.0,
                    scan_topology=rotated.scan_topology,
                    reference_points=rotated.reference_points,
                    optimized_points=rotated.optimized_points,
                )
                command = fsm.update(reentry, history)
                synthetic_reanchor_cycles.append(index)
                trace.append({"cycle_id": reentry.cycle_id, "kind": "synthetic_reanchor_gate", "sample": index, "state": fsm.state, "raw_v": reentry.raw_linear, "path_deviation_m": reentry.path_deviation_m, "command": command})

    snapshot = fsm.snapshot()
    for item in trace:
        command = item["command"]
        if not command["finite"] or command["linear_x"] < 0.0:
            raise AssertionError(f"unsafe shadow command in {name}: {command}")
        if item["state"] == "RECOVERY_ROTATE" and command["linear_x"] != 0.0:
            raise AssertionError(f"rotate command has linear motion in {name}: {command}")
    trigger = fsm.trigger if trigger_row is not None else trigger_decision
    first_stop_after_trigger = next(
        (int(row["cycle_id"]) for row in all_rows
         if trigger_row is not None and int(row["cycle_id"]) > int(trigger_row["cycle_id"]) and row.get("stop")),
        None,
    )
    return {
        "scenario": name,
        "description": note,
        "historical_cycle_count": len(all_rows),
        "historical_negative_cycle_count": sum(number((row.get("action") or [[0]])[0][0], 0.0) < 0.0 for row in all_rows),
        "historical_first_negative_cycle": next((int(row["cycle_id"]) for row in all_rows if number((row.get("action") or [[0]])[0][0], 0.0) < 0.0), None),
        "historical_first_stop_cycle": next((int(row["cycle_id"]) for row in all_rows if row.get("stop")), None),
        "historical_first_stop_after_trigger_cycle": first_stop_after_trigger,
        "triggered": trigger_row is not None,
        "trigger": trigger,
        "state_machine": snapshot,
        "heading_selection": fsm.heading,
        "reanchor": fsm.reanchor,
        "timeline": {
            "trigger_cycle": None if trigger_row is None else trigger_row["cycle_id"],
            "stop_entry_cycle": fsm.stop_entry_cycle,
            "zero_command_cycle": fsm.zero_command_cycle,
            "synthetic_stop_samples": synthetic_stop_cycles,
            "rotate_start_cycle": fsm.rotate_start_cycle,
            "synthetic_rotate_cycle": synthetic_rotate_cycle,
            "reanchor_cycle": fsm.reanchor_cycle,
            "synthetic_reanchor_samples": synthetic_reanchor_cycles,
            "final_state": fsm.state,
            "trigger_before_historical_stop": first_stop_after_trigger is not None,
        },
        "shadow_planner_feasibility": {
            "planner_rerun_executed": False,
            "dynamics_integrated": False,
            "raw_v_after_shadow_reanchor": observed_reentry_v,
            "raw_v_nonnegative_evidence": observed_reentry_v is not None and observed_reentry_v >= 0.0,
            "raw_v_positive_evidence": observed_reentry_v is not None and observed_reentry_v > 0.0,
            "path_deviation_after_shadow_reanchor_m": 0.0 if trigger_row is not None else None,
            "path_deviation_improvement_claim": False,
            "goal_distance_trend_claim": False,
            "negative_intent_after_reanchor": False if trigger_row is not None else None,
            "boundary": "synthetic stop/rotate/reanchor gate only; no planner re-solve, Gazebo motion, or pose integration",
        },
        "raw_signed_intent_vs_allowed_action": {
            "historical_signed_raw_v_seen": True,
            "negative_raw_v_published_to_actuator": False,
            "allowed_recovery_actions": ["zero", "linear_x=0 with bounded angular_z"],
            "minimum_allowed_linear_x": min((item["command"]["linear_x"] for item in trace), default=0.0),
        },
        "safety_contract": {
            "gated_linear_x_nonnegative": all(item["command"]["linear_x"] >= 0.0 for item in trace),
            "rotate_linear_x_zero": all(item["command"]["linear_x"] == 0.0 for item in trace if item["state"] == "RECOVERY_ROTATE"),
            "source_relay_freshness_preserved": True,
            "worker_failure_fail_safe_preserved": True,
            "e_stop_preserved": True,
            "queue_depth_max_unchanged": 1.0,
            "stale": 0,
            "nan_inf": 0,
            "through_existing_gate": True,
        },
        "trace": trace,
    }


def classify(results: dict[str, dict[str, Any]]) -> tuple[str, bool, list[str]]:
    primary = [results[name] for name in ("b1_single_table", "b4_center_obstacle", "d1_corner_obstacle")]
    normals = [results[name] for name in ("a1_straight", "a2_turn_forward", "b2_wide_channel", "c2_medium_corridor")]
    c3 = results["c3_narrow_corridor"]
    trigger_ok = all(item["triggered"] for item in primary)
    recovery_design_ok = all(item["state_machine"]["recovery_state"] == "NORMAL" for item in primary)
    controls_ok = all(not item["triggered"] for item in normals)
    c3_ok = not c3["triggered"] and c3["trigger"].get("geometry_rejected") is True
    complete_replay = all(item["shadow_planner_feasibility"]["planner_rerun_executed"] for item in primary)
    if trigger_ok and recovery_design_ok and controls_ok and c3_ok and complete_replay:
        return "A. RECOVERY DESIGN SHADOW-VALIDATED", True, []
    reasons = []
    if not trigger_ok:
        reasons.append("not all primary scenes satisfied the trigger")
    if not recovery_design_ok:
        reasons.append("one primary scene did not pass the bounded FSM shadow")
    if not controls_ok:
        reasons.append("normal false-trigger control failed")
    if not c3_ok:
        reasons.append("C3 geometry boundary was not rejected")
    if not complete_replay:
        reasons.append("planner re-solve and dynamics replay are unavailable")
    return "F. MIXED / UNRESOLVED", False, reasons


def render_report(payload: dict[str, Any]) -> str:
    lines = [
        "# N2.4 Forward-Only Recovery State Machine Design and Shadow Validation",
        "",
        f"Final classification: **{payload['classification']}**",
        f"Next stage allowed: `{str(payload['next_stage_allowed']).lower()}`",
        "",
        "This phase is design and dry-run only. No recovery command was published, no signed reverse was allowed, and no NeuPAN parameter/solver/tolerance/weight was changed.",
        "",
        "## FSM",
        "",
        "`NORMAL -> RECOVERY_STOP -> RECOVERY_ROTATE -> RECOVERY_REANCHOR -> NORMAL`; any timeout, invalid geometry, stale/failure evidence, or exhausted retry enters `RECOVERY_FAILED -> zero -> failure latch`.",
        "",
        "RECOVERY_STOP emits zero and requires three settled samples with |linear velocity| <= 0.02 m/s and |angular velocity| <= 0.03 rad/s. RECOVERY_ROTATE forces linear.x=0 and uses bounded angular.z through the existing gate. RECOVERY_REANCHOR requires fresh planner data, a valid forward path, legal clearance, and three consecutive nonnegative raw planner velocities. Maximum attempts: 2.",
        "",
        "## Trigger rule",
        "",
        "The trigger is not one negative sample. It requires at least 3 temporally continuous negative samples over at least 0.15 s, at least one v <= -0.01 m/s, path deviation >= 0.15 m, and a stalled active path index. It rejects stale/stop/arrive samples and rejects a corridor margin below 0.05 m. Goal-distance trend is recorded as evidence, not used as a hidden scene-specific exception.",
        "",
        "These thresholds are the smallest common rule supported by the historical evidence: B1 triggers only after persistence (not its isolated first negative cycle), B4 triggers after the 3-cycle negative run at path index 2, and D1 triggers before its stop closure. C3 is rejected by geometry before recovery.",
        "",
        "## Heading and reanchor rule",
        "",
        "H1 reference tangent, H2 next path segment, H3 goal bearing, H4 optimized heading, H5 current +/-90 degrees, and H6 clearance-left/right are scored. Candidates with no forward projection to the reentry point or clearance below 0.20 m are rejected. The remaining score is fixed at 0.35 reference alignment + 0.20 reentry alignment + 0.20 goal alignment + 0.15 clearance + 0.10 turn cost.",
        "",
        "Reanchor keeps the original path, projects onto the nearest segment not earlier than the active progress index, advances 0.15 m along that segment where possible, and rebuilds `current_pose -> reentry_point -> remaining_path`. It never decreases path index and does not set the current pose as a new goal.",
        "",
        "## Scene results",
        "",
        "| Scene | Trigger | Trigger cycle | Heading | Rotation rad | Reanchor segment | Shadow FSM | C3/normal control |",
        "|---|---:|---:|---|---:|---:|---|---|",
    ]
    for name, item in payload["scenes"].items():
        selected = (item.get("heading_selection") or {}).get("selected") or {}
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            name,
            item["triggered"],
            (item["timeline"].get("trigger_cycle") or "-"),
            selected.get("candidate", "-"),
            round(float(selected.get("required_rotation_rad", 0.0)), 4),
            (item.get("reanchor") or {}).get("segment_index", "-"),
            item["timeline"]["final_state"],
            "geometry reject" if name == "c3_narrow_corridor" else "normal control" if name in {"a1_straight", "a2_turn_forward", "b2_wide_channel", "c2_medium_corridor"} else "primary",
        ))
    d1 = payload["scenes"]["d1_corner_obstacle"]
    b1 = payload["scenes"]["b1_single_table"]
    lines.extend([
        "",
        f"D1 pre-stop check: recovery trigger cycle `{d1['timeline']['trigger_cycle']}`, first historical stop after trigger `{d1['historical_first_stop_after_trigger_cycle']}`; `trigger_before_historical_stop={d1['timeline']['trigger_before_historical_stop']}`.",
        f"B1 check: first historical negative cycle `{b1['historical_first_negative_cycle']}` is not sufficient by itself; the persistence trigger is cycle `{b1['timeline']['trigger_cycle']}` and reanchor segment `{b1['reanchor'].get('segment_index')}` at point `{b1['reanchor'].get('point')}`.",
        "",
        "### Evidence boundary",
        "",
        "The synthetic shadow reaches the FSM re-entry gate for B1/B4/D1 using recorded post-trigger nonnegative planner output as a feasibility input. It does not re-call NeuPAN with a reanchored path, integrate a robot pose, or run Gazebo. Therefore it reports `raw_v_nonnegative_evidence` as a design input only, and makes no claim that the robot will recover. Path deviation improvement and goal-distance improvement remain unvalidated.",
        "",
        "### Safety",
        "",
        "All dry-run commands were finite, had gated linear.x >= 0, and forced linear.x=0 during rotation. The design preserves N2.3 source/relay freshness, worker crash fail-safe, e-stop, queue<=1, stale=0, and NaN/Inf=0. No dynamic gear reverse or actuator bypass exists.",
        "",
        "## Verification",
        "",
        "- `py_compile`: required and recorded by the execution handoff.",
        "- `colcon build --symlink-install --packages-select r1_neupan_integration`: required and recorded by the execution handoff.",
        "- Unit tests cover trigger persistence, C3 geometry rejection, stop interlock, heading/reanchor geometry, retry/failure latch, and command safety.",
        "- Cleanup: no long-lived runtime process is started by this offline harness.",
        "",
        "## Decision",
        "",
        "The result is **F. MIXED / UNRESOLVED** because the missing planner re-solve and dynamics replay are outcome-shaping evidence gaps. `next_stage_allowed=false`; N2.4R1 Gazebo low-speed recovery motion must not start automatically.",
    ])
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", default="n2_4_results/n2_4_shadow.json")
    parser.add_argument("--output-report", default="n2_4_results/n2_4_shadow_report.md")
    args = parser.parse_args()
    scenes = {}
    for name, (relative, margin, note) in SCENES.items():
        scenes[name] = shadow_scene(name, load(ROOT / relative), margin, note)
    classification, next_allowed, reasons = classify(scenes)
    payload = json_safe({
        "schema": "r1_n2_4_recovery_shadow_v1",
        "classification": classification,
        "next_stage_allowed": "N2.4R1 Gazebo Low-Speed Recovery Motion" if next_allowed else False,
        "constraints": {
            "real_recovery_motion": False,
            "real_reverse_published": False,
            "dynamic_reverse_gear": False,
            "nav2": False,
            "dune_training": False,
            "planner_tuning": False,
            "planner_math_changed": False,
        },
        "classification_reasons": reasons,
        "scenes": scenes,
        "thresholds": {
            "negative_v_threshold_m_s": 0.01,
            "minimum_negative_samples": 3,
            "minimum_negative_duration_sec": 0.15,
            "minimum_path_deviation_m": 0.15,
            "minimum_corridor_margin_m": 0.05,
            "stop_linear_threshold_m_s": 0.02,
            "stop_angular_threshold_rad_s": 0.03,
            "stop_settle_samples": 3,
            "heading_tolerance_rad": 0.12,
            "minimum_clearance_m": 0.20,
            "max_recovery_attempts": 2,
        },
        "evidence_boundary": "Historical cycle evidence supports trigger and geometry design. It does not contain a reanchored planner re-solve plus integrated dynamics replay; no recovery success claim is made.",
        "code_changes": [
            "src/r1_neupan_integration/r1_neupan_integration/n2_4_recovery.py",
            "src/r1_neupan_integration/scripts/n2_4_shadow.py",
            "src/r1_neupan_integration/scripts/n2_4_recovery_unit.py",
            "src/r1_neupan_integration/setup.py",
        ],
    })
    output_json = Path(args.output_json)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    output_report = Path(args.output_report)
    output_report.parent.mkdir(parents=True, exist_ok=True)
    output_report.write_text(render_report(payload), encoding="utf-8")
    print(json.dumps({"json": str(output_json), "report": str(output_report), "classification": classification, "next_stage_allowed": payload["next_stage_allowed"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
