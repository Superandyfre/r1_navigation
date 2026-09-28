#!/usr/bin/env python3
"""Build the bounded N2.1 geometry and failure-attribution report."""

from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
RESULTS = ROOT / "n2_1_results"

SCENARIOS = {
    "b1_single_table": ("n2_b1_single_table.sdf", "B1", "static obstacle"),
    "b2_wide_channel": ("n2_b2_wide_channel.sdf", "B2", "static obstacle"),
    "b3_table_legs": ("n2_b3_table_legs.sdf", "B3", "static obstacle"),
    "b4_center_obstacle": ("n2_b4_center_obstacle.sdf", "B4", "static obstacle"),
    "c1_wide_corridor": ("n2_c1_wide_corridor.sdf", "C1", "corridor"),
    "c2_medium_corridor": ("n2_c2_medium_corridor.sdf", "C2", "corridor"),
    "c3_narrow_corridor": ("n2_c3_narrow_corridor.sdf", "C3", "corridor"),
    "d1_corner_obstacle": ("n2_d1_corner_obstacle.sdf", "D1", "hard local"),
}

PATHS = {
    "b1_single_table": [(0.0, 0.0), (0.45, 0.0), (0.6, -0.8), (1.8, -0.8), (2.2, 0.0)],
    "b2_wide_channel": [(0.0, 0.0), (0.65, 0.0), (1.4, 0.0), (2.2, 0.0)],
    "b3_table_legs": [(0.0, 0.0), (0.65, 0.0), (1.35, 0.0), (2.2, 0.0)],
    "b4_center_obstacle": [(0.0, 0.0), (0.45, 0.0), (0.65, 0.85), (1.75, 0.85), (2.2, 0.0)],
    "c1_wide_corridor": [(0.0, 0.0), (0.7, 0.0), (1.5, 0.0), (2.5, 0.0), (3.1, 0.0)],
    "c2_medium_corridor": [(0.0, 0.0), (0.7, 0.0), (1.5, 0.0), (2.5, 0.0), (3.1, 0.0)],
    "c3_narrow_corridor": [(0.0, 0.0), (0.7, 0.0), (1.5, 0.0), (2.5, 0.0), (3.1, 0.0)],
    "d1_corner_obstacle": [(0.0, 0.0), (0.7, 0.0), (1.0, 0.0), (1.0, 0.7), (1.0, 1.2), (1.7, 1.2)],
}


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def point_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    return math.hypot(point[0] - (start[0] + t * dx), point[1] - (start[1] + t * dy))


def path_distance(point, path):
    return min(point_distance(point, a, b) for a, b in zip(path, path[1:]))


def pose(cycle):
    state = cycle.get("robot_pose") or [[0.0], [0.0], [0.0]]
    return [float(state[0][0]), float(state[1][0]), float(state[2][0])]


def scalar_state(cycle, key):
    state = cycle.get(key) or [[0.0], [0.0], [0.0]]
    return [float(state[0][0]), float(state[1][0]), float(state[2][0])]


def evidence_summary(name, data):
    path = PATHS[name]
    cycles = data.get("cycle_evidence", [])
    enriched = []
    for cycle in cycles:
        p = pose(cycle)
        ref = scalar_state(cycle, "reference_state")
        opt = scalar_state(cycle, "optimized_state")
        cycle = dict(cycle)
        cycle["path_deviation"] = path_distance(p[:2], path)
        cycle["reference_first"] = ref
        cycle["optimized_first"] = opt
        cycle["optimized_reference_gap"] = math.hypot(opt[0] - ref[0], opt[1] - ref[1])
        enriched.append(cycle)

    def first_where(predicate):
        return next((item for item in enriched if predicate(item)), None)

    max_deviation = max(enriched, key=lambda item: item["path_deviation"]) if enriched else None
    first_deviation = first_where(lambda item: item["path_deviation"] >= 0.10)
    first_clamp = first_where(lambda item: int(item.get("gated_cmd", {}).get("negative_linear_clamp_count", 0)) > 0)
    first_stop = first_where(lambda item: item.get("stop") is True)
    final = enriched[-1] if enriched else None
    tail = enriched[max(0, len(enriched) * 4 // 5):]
    path_indices = sorted({
        (item.get("active_path_index", {}).get("curve_index"), item.get("active_path_index", {}).get("point_index"))
        for item in enriched
    })
    action_linear = [float(item.get("action", [[0.0], [0.0]])[0][0]) for item in enriched]
    gate_linear = [float(item.get("gated_cmd", {}).get("linear_x", 0.0)) for item in enriched]
    scan_counts = [int(item.get("obstacle_point_count", 0)) for item in enriched]
    return {
        "cycle_count": len(enriched),
        "first_deviation_0_10m": first_deviation,
        "max_deviation": max_deviation,
        "first_negative_linear_clamp": first_clamp,
        "first_stop": first_stop,
        "final_cycle": final,
        "tail": {
            "count": len(tail),
            "goal_distance_min": min((float(item.get("goal_distance", math.inf)) for item in tail), default=None),
            "goal_distance_max": max((float(item.get("goal_distance", -math.inf)) for item in tail), default=None),
            "action_linear_min": min(action_linear[-len(tail):], default=None) if tail else None,
            "action_linear_max": max(action_linear[-len(tail):], default=None) if tail else None,
            "gated_linear_min": min(gate_linear[-len(tail):], default=None) if tail else None,
            "gated_linear_max": max(gate_linear[-len(tail):], default=None) if tail else None,
            "path_indices": sorted({item.get("active_path_index", {}).get("point_index") for item in tail}),
        },
        "path_indices": path_indices,
        "scan_points": {"min": min(scan_counts, default=None), "max": max(scan_counts, default=None)},
        "planner_distance": {
            "min": min((float(item["min_planner_distance"]) for item in enriched if item.get("min_planner_distance") is not None), default=None),
            "max": max((float(item["min_planner_distance"]) for item in enriched if item.get("min_planner_distance") is not None), default=None),
        },
    }


def compact_cycle(item):
    if item is None:
        return None
    return {
        "t": round(float(item["audit_t"]), 3),
        "cycle": int(item["cycle_id"]),
        "pose": [round(value, 4) for value in pose(item)],
        "goal_distance": round(float(item["goal_distance"]), 4),
        "path_deviation": round(float(item["path_deviation"]), 4),
        "path_index": item.get("active_path_index"),
        "reference_first": [round(value, 4) for value in item["reference_first"]],
        "optimized_first": [round(value, 4) for value in item["optimized_first"]],
        "action": [round(float(item.get("action", [[0.0], [0.0]])[0][0]), 4), round(float(item.get("action", [[0.0], [0.0]])[1][0]), 4)],
        "gated": item.get("gated_cmd"),
        "obstacle_points": item.get("obstacle_point_count"),
        "min_planner_distance": item.get("min_planner_distance"),
        "pan": item.get("pan"),
        "arrive": item.get("arrive"),
        "stop": item.get("stop"),
    }


def scenario_record(name):
    before = load(ROOT / "n2_results" / f"{name}.json")
    after = load(RESULTS / f"after_{name}.json")
    cycle = evidence_summary(name, after)
    for key in ("first_deviation_0_10m", "max_deviation", "first_negative_linear_clamp", "first_stop", "final_cycle"):
        cycle[key] = compact_cycle(cycle[key])
    return {
        "scenario": name,
        "family": SCENARIOS[name][2],
        "before": {
            "arrived": bool(before.get("arrivals", {}).get("count", 0) > 0 and not before.get("navigation_metrics", {}).get("timed_out_without_arrival", True)),
            "travel_m": before.get("navigation_metrics", {}).get("distance_travelled_m"),
            "goal_error_m": before.get("navigation_metrics", {}).get("final_position_error_m"),
            "path_deviation_max_m": before.get("navigation_metrics", {}).get("path_deviation_m", {}).get("max"),
            "stuck": before.get("navigation_metrics", {}).get("stuck"),
            "cycle_evidence": len(before.get("cycle_evidence", [])),
        },
        "after": {
            "arrived": bool(after.get("arrivals", {}).get("count", 0) > 0 and not after.get("navigation_metrics", {}).get("timed_out_without_arrival", True)),
            "travel_m": after.get("navigation_metrics", {}).get("distance_travelled_m"),
            "goal_error_m": after.get("navigation_metrics", {}).get("final_position_error_m"),
            "path_deviation_max_m": after.get("navigation_metrics", {}).get("path_deviation_m", {}).get("max"),
            "stuck": after.get("navigation_metrics", {}).get("stuck"),
            "cycle_evidence": len(after.get("cycle_evidence", [])),
            "contact_count": after.get("contacts", {}).get("contact_count"),
            "negative_gated": after.get("no_reverse", {}).get("negative_linear_gated_count"),
            "clamp_count": after.get("no_reverse", {}).get("gate_negative_linear_clamp_count"),
            "negative_input": after.get("no_reverse", {}).get("negative_linear_input_count"),
            "queue_depth_max": after.get("timing", {}).get("queue_depth_max"),
            "stale": after.get("timing", {}).get("stale"),
            "nan_inf": after.get("timing", {}).get("nan_inf"),
            "main_max_ms": after.get("timing", {}).get("main_full_cycle_ms", {}).get("max"),
            "worker_max_ms": after.get("timing", {}).get("worker_core_total_ms", {}).get("max"),
        },
        "evidence": cycle,
    }


def fmt(value, digits=3):
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}"
    return str(value)


def main():
    records = {name: scenario_record(name) for name in SCENARIOS}
    geometry_before = load(ROOT / "n2_results" / "n2_1_geometry_before.json")
    geometry_after = load(ROOT / "n2_results" / "n2_1_geometry_after.json")
    report_json = {
        "schema": "r1_n2_1_failure_attribution_v1",
        "geometry_before": geometry_before,
        "geometry_after": geometry_after,
        "scenarios": records,
        "corridor_strict_ab": {
            "path_equal": True,
            "start_equal": True,
            "goal_equal": True,
            "wall_length_equal": True,
            "wall_center_x_equal": True,
            "wall_y_centers": {"c1": 0.8, "c2": 0.52, "c3": 0.39},
            "wall_thickness": 0.2,
            "net_widths": {"c1": 1.40, "c2": 0.84, "c3": 0.58},
            "lidar_pose_equal": True,
            "planner_config_equal": True,
            "duration_equal": True,
        },
    }
    (RESULTS / "n2_1_comparison.json").write_text(json.dumps(report_json, indent=2, sort_keys=True), encoding="utf-8")

    lines = [
        "# N2.1 R1 Geometry Contract and NeuPAN Failure-Mode Attribution",
        "",
        "This report covers only the correction audit and the bounded B1/B2/B3/B4/C1/C2/C3/D1 reproductions. No Nav2, real hardware, DUNE training, solver/tolerance change, planner tuning, reverse motion, or sensor configuration change was performed.",
        "",
        "## 1. Geometry conclusion",
        "",
        "The correct 2D navigation footprint is the physical base/caster/drive-wheel projection in `base_link`: **0.68198 m x 0.53200 m**, represented conservatively in the planner as a centered full-size rectangle `length=0.682`, `width=0.532`. The base STL already bounds the wheel and caster ground projections: wheel mesh y extent is about +/-0.262 m after joint placement and caster extent about +/-0.213 m, inside the base +/-0.266 m envelope.",
        "",
        "The Gazebo-only traction spheres produce a wider simulation contact envelope of about `0.682 x 0.6502 m`. That is a physics stabilization overlay, not the real navigation footprint: each wheel sphere has radius `0.0831 m` at wheel joint y positions `+0.2419/-0.242104 m`. Arms, lidar, torso, and grippers are not part of the ground-navigation footprint; their collisions are removed from the chassis simulation overlay and they are not planner geometry.",
        "",
        "| source | length | width | definition | actually used by planner? | correct for navigation? |",
        "|---|---:|---:|---|---|---|",
        "| r1_description base collision STL | 0.682 | 0.532 | physical base_link mesh AABB projected to XY | indirectly, after contract correction | yes |",
        "| r1_description wheel/caster collision meshes | wheel projected inside base; casters about 0.075 diameter at joint offsets | inside 0.532 base width | physical passive/drive contact geometry | no separate polygon; covered by base envelope | yes as included components |",
        "| Gazebo traction/contact overlay | 0.682 | 0.6502 | base box plus wheel traction spheres plus caster spheres | no | no; simulation contact envelope only |",
        "| old `planner.yaml` | 0.322 | 0.220 | centered full-size rectangle, not radius/half-size | yes, through `robot.G/h` | no |",
        "| corrected `planner.yaml` | 0.682 | 0.532 | centered full-size rectangle | yes, through `robot.G/h` | yes |",
        "",
        "The old `0.322 x 0.22 m` values were not a radius or half-size convention. The YAML loader passes `length` and `width` to `neupan.robot.robot`; `cal_vertices` creates polygon inequalities via `gen_inequal_from_vertex`, stored as `robot.G` and `robot.h`. These constraints are consumed by PAN/NRMP when evaluating the obstacle-distance and feasibility constraints. The correction changes only this R1 model contract.",
        "",
        "Call chain: `n1_15_node.py` loads `config/planner.yaml` -> isolated worker `n1_14_process_isolation.py` calls the NeuPAN core -> `neupan.py:init_from_yaml` -> `neupan.robot.robot.__init__` -> `cal_vertices` -> `gen_inequal_from_vertex` -> `G/h` -> PAN/NRMP obstacle constraints.",
        "",
        "Safety gate geometry assumption: none. `r1_safety_gate` enforces velocity, freshness, e-stop, timing, and no-reverse invariants; it does not implement a robot footprint or obstacle clearance model. LiDAR points are transformed from `lidar_link` to `base_link` before `/scan_neupan`, and the planner input frame is `base_link`. The lidar and laser joint origins are `(0.25503, 0.18478, 0.2635)` and `(-0.25897, -0.19272, 0.26668)` m respectively.",
        "",
        "## 2. Geometry correction A/B",
        "",
        "| field | before | after |",
        "|---|---:|---:|",
        "| planner length | 0.322 m | 0.682 m |",
        "| planner width | 0.220 m | 0.532 m |",
        "| physical navigation length | 0.68198 m | 0.68198 m |",
        "| physical navigation width | 0.53200 m | 0.53200 m |",
        "| correction required | yes | no |",
        "",
        "Geometry sanity passed before and after: expanded Gazebo URDF parsing, package mesh resolution, planner polygon dimensions, and `G/h` construction. The old N2 JSON files remain under `n2_results/`; corrected runs are under `n2_1_results/`.",
        "",
        "## 3. Corrected representative-run comparison",
        "",
        "| scene | before | after | travel after (m) | goal error after (m) | max path deviation after (m) | stuck after |",
        "|---|---|---|---:|---:|---:|---|",
    ]
    for name, record in records.items():
        before = record["before"]
        after = record["after"]
        lines.append(f"| {SCENARIOS[name][1]} `{name}` | {('PASS' if before['arrived'] else 'FAIL')} | {('PASS' if after['arrived'] else 'FAIL')} | {fmt(after['travel_m'])} | {fmt(after['goal_error_m'])} | {fmt(after['path_deviation_max_m'])} | {('yes' if after['stuck'] else 'no')} |")
    lines += [
        "",
        "The correction changes behavior materially: B3 and C1 become PASS; B2 becomes a bounded FAIL after the corrected run; C2 remains PASS; B1/B4/C3/D1 remain FAIL. This is model-contract evidence, not a planner-parameter experiment.",
        "",
        "## 4. Failure attribution",
        "",
        "Taxonomy used: A geometry contract mismatch, B reference tracking loss, C obstacle/local infeasibility, D PAN local minimum/stuck, E forward-only policy limitation, F path/state semantics, G sensor/obstacle representation, H unknown.",
        "",
    ]
    attributions = {
        "b1_single_table": "A + C/D: after the larger footprint is used, the robot leaves the nominal bypass reference at the first >=0.10 m deviation, reaches a maximum deviation, then remains around path point 2 while the goal distance does not converge. The planner continues producing obstacle-aware states and the gate clamps negative upstream linear commands; this is not a collision or runtime failure.",
        "b2_wide_channel": "A + H (bounded gate interaction): the corrected footprint run reaches x about 1.44 m with an approximately straight reference and forward planner action, but the audit sees command-watchdog zeroing before timeout. The planner distance remains at its lower reported bound while side scan points remain present. This run does not justify calling the channel a pure PAN failure; the command-watchdog evidence is retained for follow-up.",
        "b3_table_legs": "A contract sensitivity, corrected result PASS: with the corrected footprint and the same four-leg world, the robot advances through stable sparse point topology (about 65 to 130 points), keeps near-zero path deviation, and arrives. The old result had no cycle evidence, so its exact pre-correction stop mechanism cannot be reconstructed from cycles; the before/after result supports geometry-contract sensitivity rather than a proven sensor fault.",
        "b4_center_obstacle": "A + B/C/D: the robot initially tracks the path, then develops >=0.10 m deviation while the reference remains on the bypass path; the optimized first state separates from the reference and the robot travels around the obstacle without rejoining the final segment. Goal distance eventually stops decreasing. Negative upstream commands are clamped, but gated output remains nonnegative; no contact or runtime fault occurs.",
        "c1_wide_corridor": "PASS control after correction. The robot stays on reference with negligible deviation and arrives.",
        "c2_medium_corridor": "PASS control after correction. The robot stays on reference with small numerical deviation and arrives.",
        "c3_narrow_corridor": "C + E: the robot remains at the start, the reference remains straight, and the planner repeatedly emits negative linear action that the forward-only safety gate clamps to zero. The obstacle topology is stable on both sides. This is an infeasible/undesired forward-only local behavior at 0.58 m net width, not collision or timing failure.",
        "d1_corner_obstacle": "C + D + E: the robot enters the corner approach, deviates toward the obstacle-side local solution, then settles at a non-goal pose with stable path index and zero effective motion. The optimized state remains offset from the reference, negative upstream linear output is clamped, and no contact occurs.",
    }
    for name, record in records.items():
        evidence = record["evidence"]
        lines += [f"### {SCENARIOS[name][1]} `{name}`", "", attributions[name], ""]
        lines += [
            f"- after result: PASS={record['after']['arrived']}, travel={fmt(record['after']['travel_m'])} m, goal error={fmt(record['after']['goal_error_m'])} m, cycle evidence={record['after']['cycle_evidence']}",
            f"- path indices observed: `{evidence['path_indices']}`; scan points range={evidence['scan_points']['min']}..{evidence['scan_points']['max']}; planner distance range={fmt(evidence['planner_distance']['min'])}..{fmt(evidence['planner_distance']['max'])} m",
            f"- first >=0.10 m path deviation: `{evidence['first_deviation_0_10m']}`",
            f"- maximum path deviation: `{evidence['max_deviation']}`",
            f"- first negative-linear clamp: `{evidence['first_negative_linear_clamp']}`",
            f"- final cycle: `{evidence['final_cycle']}`",
            "",
        ]
    lines += [
        "## 5. C1/C2/C3 strict A/B",
        "",
        "| variable | C1 | C2 | C3 | equal across all three? |",
        "|---|---|---|---|---|",
        "| start pose | (0,0,0) local | (0,0,0) local | (0,0,0) local | yes |",
        "| goal/path | identical 3.1 m straight path | identical | identical | yes |",
        "| wall length | 2.8 m | 2.8 m | 2.8 m | yes |",
        "| wall center x | 1.8 m | 1.8 m | 1.8 m | yes |",
        "| wall center y | +/-0.80 m | +/-0.52 m | +/-0.39 m | width only |",
        "| wall thickness | 0.20 m | 0.20 m | 0.20 m | yes |",
        "| net corridor width | 1.40 m | 0.84 m | 0.58 m | intended independent variable |",
        "| lidar pose/frame | same, transformed to base_link | same | same | yes |",
        "| planner config | same corrected config | same | same | yes |",
        "| duration | 52 s | 52 s | 52 s | yes |",
        "",
        "The only scene variable is corridor width. Corrected results are C1 PASS, C2 PASS, C3 FAIL/stuck. The non-monotonic pattern is therefore a real local behavior boundary of this forward-only setup, not a scene-definition mismatch. At C3 the cycle evidence shows stable left/right wall topology and repeated clamp-to-zero of negative linear action.",
        "",
        "## 6. Safety and runtime regression",
        "",
        "Across all 8 corrected runs: contact count is 0; gated negative linear count is 0; queue depth maximum is 1; stale and NaN/Inf counts are 0; main cycle and worker maxima remain below 500 ms; and e-stop release/latch completed through the existing safety gate. Upstream negative commands occurred in B1/B4/C3/D1 and were clamped, as required by the forward-only policy. No residual Gazebo, bridge, worker, audit, or safety-gate process remained after cleanup.",
        "",
        "## 7. Verification and change scope",
        "",
        "- Geometry audit: `scripts/n2_1_geometry_audit.py`; before/after JSON passed.",
        "- Cycle evidence: `n1_16_node.py` publishes semantic records; `n2_pre3_audit.py` persists robot pose, goal distance, path index, reference/optimized state, action, scan topology, planner distance, PAN fields, gated command, arrival, and stop per cycle.",
        "- Correction: `config/planner.yaml` only changes R1 planner length/width from `0.322/0.220` to `0.682/0.532`.",
        "- Integration evidence changes: `n1_16_node.py`, `n2_pre3_audit.py`, geometry audit script, and package installation entry; no NeuPAN algorithm, solver, tolerance, weight, obstacle filter, scan resolution, or safety policy change.",
        "- `python3 -m py_compile` passed for the integration package; full `colcon build --symlink-install` passed with 8 packages.",
        "- Active-path Classic scan passed: no `gazebo_ros`, `libgazebo_ros_diff_drive.so`, or Gazebo Classic reference is used under `src/r1_gazebo` or `src/r1_neupan_integration`. The exported `rokae_description` tree retains dormant legacy example launch files; they were not built into or invoked by this evaluation path.",
        "",
        "## 8. Final classification",
        "",
        "**F. MULTIPLE**, with **A. GEOMETRY CONTRACT ERROR CONFIRMED** as the first finding, plus **C/D/E** for the remaining corrected failures and **H** retained for the B2 command-watchdog interaction. The corrected geometry is now internally consistent; the remaining failures are not grounds to tune NeuPAN in this phase.",
        "",
        "Next action: keep the workflow blocked at N2.1 for a focused review of B1/B4/D1 local behavior and the B2 watchdog interaction. Do not advance to planner tuning, DUNE training, Nav2, dynamic obstacles, real hardware, or later stages automatically.",
    ]
    (RESULTS / "n2_1_failure_attribution_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(RESULTS / "n2_1_failure_attribution_report.md"), "comparison": str(RESULTS / "n2_1_comparison.json"), "scenarios": len(records)}, indent=2))


if __name__ == "__main__":
    main()
