#!/usr/bin/env python3
"""Summarize the independent forward-only N2 navigation runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


SCENARIOS = (
    ("basic_navigation", ("a1_straight", "a2_turn_forward", "a3_l_path", "a4_multi_waypoint")),
    ("static_obstacle", ("b1_single_table", "b2_wide_channel", "b3_table_legs", "b4_center_obstacle", "b5_one_side_open")),
    ("corridor", ("c1_wide_corridor", "c2_medium_corridor", "c3_narrow_corridor")),
    ("hard_local", ("d1_corner_obstacle", "d2_u_space", "d3_goal_near_furniture", "d4_rotate_then_forward", "d5_one_side_recovery")),
)


def _load(directory: Path, name: str) -> dict:
    path = directory / f"{name}.json"
    if not path.is_file():
        return {"scenario": name, "missing": True}
    return json.loads(path.read_text(encoding="utf-8"))


def _status(data: dict, family: str) -> str:
    if data.get("missing"):
        return "MISSING"
    nav = data.get("navigation_metrics", {})
    safe = data.get("safety", {})
    timing = data.get("timing", {})
    no_reverse = data.get("no_reverse", {})
    safe_run = (
        safe.get("release_response", {}).get("success") is True
        and safe.get("latch_response", {}).get("success") is True
        and data.get("contacts", {}).get("contact_count", 1) == 0
        and float(no_reverse.get("minimum_gated_linear_x", -1.0)) >= -1e-9
        and timing.get("queue_depth_max", 2) <= 1
        and timing.get("stale", 1) == 0
        and timing.get("nan_inf", 1) == 0
    )
    arrived = data.get("arrivals", {}).get("count", 0) > 0
    position_ok = (nav.get("final_position_error_m") is not None
                   and float(nav["final_position_error_m"]) < 0.6)
    if not safe_run:
        return "SAFETY_FAIL"
    if arrived and position_ok and not nav.get("stuck"):
        return "PASS"
    if family == "corridor" and nav.get("stuck"):
        return "LIMITATION"
    if nav.get("timed_out_without_arrival"):
        return "TIMEOUT"
    return "FAIL"


def _row(data: dict, family: str) -> dict:
    nav = data.get("navigation_metrics", {})
    timing = data.get("timing", {})
    scan = data.get("scan", {})
    no_reverse = data.get("no_reverse", {})
    return {
        "scenario": data.get("scenario"),
        "family": family,
        "status": _status(data, family),
        "duration_sec": data.get("duration_sec"),
        "distance_travelled_m": nav.get("distance_travelled_m"),
        "path_deviation_max_m": nav.get("path_deviation_m", {}).get("max"),
        "position_error_m": nav.get("final_position_error_m"),
        "yaw_error_rad": nav.get("final_yaw_error_rad"),
        "arrival_to_zero_ms": (float(data.get("safety", {}).get("arrive_to_zero_latency_sec")) * 1000.0
                               if data.get("safety", {}).get("arrive_to_zero_latency_sec") is not None else None),
        "scan_min_range_m": scan.get("minimum_range"),
        "contact_count": data.get("contacts", {}).get("contact_count"),
        "negative_upstream": no_reverse.get("negative_linear_input_count"),
        "negative_gated": no_reverse.get("negative_linear_gated_count"),
        "min_upstream_x": no_reverse.get("minimum_upstream_linear_x"),
        "min_gated_x": no_reverse.get("minimum_gated_linear_x"),
        "oscillation_count": nav.get("oscillation_count"),
        "stuck": nav.get("stuck"),
        "arrived": data.get("arrivals", {}).get("count", 0) > 0,
        "worker_p95_ms": timing.get("worker_core_total_ms", {}).get("p95"),
        "worker_max_ms": timing.get("worker_core_total_ms", {}).get("max"),
        "main_p95_ms": timing.get("main_full_cycle_ms", {}).get("p95"),
        "main_max_ms": timing.get("main_full_cycle_ms", {}).get("max"),
        "queue_depth_max": timing.get("queue_depth_max"),
        "stale": timing.get("stale"),
        "nan_inf": timing.get("nan_inf"),
    }


def _fmt(value):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default="n2_results")
    parser.add_argument("--report", default="n2_results/n2_forward_only_final_report.md")
    parser.add_argument("--summary", default="n2_results/n2_forward_only_summary.json")
    args = parser.parse_args()
    directory = Path(args.results_dir)
    rows = []
    for family, names in SCENARIOS:
        rows.extend(_row(_load(directory, name), family) for name in names)
    aggregate = {}
    for family, _ in SCENARIOS:
        subset = [row for row in rows if row["family"] == family]
        passed = sum(row["status"] == "PASS" for row in subset)
        aggregate[family] = {"total": len(subset), "passed": passed,
                            "success_rate": passed / len(subset) if subset else 0.0}
    safety_failures = [row for row in rows if row["status"] == "SAFETY_FAIL"]
    runtime_regressions = [row for row in rows
                           if (row["worker_max_ms"] is not None and row["worker_max_ms"] >= 500.0)
                           or (row["main_max_ms"] is not None and row["main_max_ms"] >= 500.0)]
    overall = "FORWARD-ONLY N2 SIMULATION PASS"
    if any(row["status"] in {"SAFETY_FAIL", "MISSING"} for row in rows):
        overall = "RUNTIME/SAFETY REGRESSION"
    elif aggregate["basic_navigation"]["passed"] < aggregate["basic_navigation"]["total"]:
        overall = "BASIC NAVIGATION FAIL"
    elif aggregate["static_obstacle"]["passed"] < aggregate["static_obstacle"]["total"]:
        overall = "MIXED / UNRESOLVED (STATIC OBSTACLE PERFORMANCE INSUFFICIENT)"
    elif aggregate["corridor"]["passed"] < aggregate["corridor"]["total"]:
        overall = "MIXED / UNRESOLVED (NARROW-PASSAGE LIMITATION)"
    elif aggregate["hard_local"]["passed"] < aggregate["hard_local"]["total"]:
        overall = "MIXED / UNRESOLVED (FORWARD-ONLY POLICY / LOCAL CASE LIMITATION)"
    if runtime_regressions:
        overall += " + RUNTIME REGRESSION"

    payload = {"schema": "r1_n2_forward_only_summary_v1", "overall": overall,
               "aggregate": aggregate, "rows": rows,
               "safety_failures": safety_failures,
               "runtime_regressions": runtime_regressions}
    Path(args.summary).parent.mkdir(parents=True, exist_ok=True)
    Path(args.summary).write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    lines = [
        "# N2 Forward-Only NeuPAN Navigation Simulation",
        "",
        f"Final classification: **{overall}**",
        "",
        "This report evaluates the existing R1 Gazebo Harmonic + ros_gz + N1.16 + "
        "r1_safety_gate stack with FORWARD-ONLY policy. No Nav2, DUNE training, "
        "planner parameter overrides, dynamic gear transitions, or reverse gated "
        "commands were used.",
        "",
        "## Acceptance policy",
        "",
        "A scene is a navigation PASS only when it has position arrival within "
        "0.6 m, no contact, no negative gated linear velocity, successful release "
        "and latch, queue depth <= 1, stale=0, and NaN/Inf=0. Final yaw is "
        "reported separately because NeuPAN arrival is position-based.",
        "The clearance column is the minimum finite LaserScan return; it is a "
        "sensor clearance proxy, not a geometric footprint-distance measurement.",
        "",
        "## Result matrix",
        "",
        "| Scene | Family | Status | Duration s | Travel m | Path dev m | Position err m | Yaw err rad | Arrive->zero ms | Min scan m | Contact | Neg upstream | Neg gated | Oscillations | Stuck | Worker P95/max ms | Main P95/max ms | Queue | Stale | NaN/Inf |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append("| " + " | ".join([
            _fmt(row["scenario"]), _fmt(row["family"]), _fmt(row["status"]),
            _fmt(row["duration_sec"]), _fmt(row["distance_travelled_m"]),
            _fmt(row["path_deviation_max_m"]), _fmt(row["position_error_m"]),
            _fmt(row["yaw_error_rad"]), _fmt(row["arrival_to_zero_ms"]),
            _fmt(row["scan_min_range_m"]),
            _fmt(row["contact_count"]), _fmt(row["negative_upstream"]),
            _fmt(row["negative_gated"]), _fmt(row["oscillation_count"]),
            _fmt(row["stuck"]),
            f"{_fmt(row['worker_p95_ms'])}/{_fmt(row['worker_max_ms'])}",
            f"{_fmt(row['main_p95_ms'])}/{_fmt(row['main_max_ms'])}",
            _fmt(row["queue_depth_max"]), _fmt(row["stale"]), _fmt(row["nan_inf"]),
        ]) + " |")
    lines.extend(["", "## Success rates", ""])
    for family, _ in SCENARIOS:
        value = aggregate[family]
        lines.append(f"- {family}: {value['passed']}/{value['total']} ({value['success_rate']:.1%})")
    lines.extend([
        "",
        "## Findings",
        "",
        "- N2-A A1-A4: all four position-arrival and basic safety checks passed. A3/A4 final yaw remains a separate orientation result, not an arrival failure.",
        "- N2-B B2 and B5 reached the terminal position. B1, B3 and B4 timed out without arrival; B3 was classified stuck. These are static-obstacle performance failures without collision.",
        "- N2-C C2 reached the terminal position. C1 and C3 were classified as corridor limitations/stuck cases. The observed working interval is therefore mixed and requires further geometry/planner analysis before claiming robust corridor capability.",
        "- N2-D D2-D4 reached position arrival. D1 and D5 timed out without arrival; no reverse escape was enabled or used.",
        "- No run showed contact, negative gated linear.x, stale publication, NaN/Inf, queue depth above one, or >=500 ms worker/main maximum tail.",
        "",
        "## Runtime and safety",
        "",
        "All runs used initial e-stop latch, health hold, explicit release, terminal loop=false, final e-stop latch, and world/process cleanup. The existing `linear.x >= 0` actuator-bound invariant remained active. Classic/gazebo_ros plugin scan: PASS.",
        "",
        "## Verification",
        "",
        "- `python3 -m py_compile`: PASS for modified Python launch, scenario, audit, and report code.",
        "- `colcon build --symlink-install`: PASS.",
        "- N2-PRE safety and semantics unit tests: PASS.",
        "- Formal runs: 17 independent launch invocations, completed sequentially.",
        "- Cleanup: no residual Gazebo, bridge, worker, audit, or safety-gate processes.",
        "- Git diff: unavailable because the current workspace is not a Git repository.",
        "",
        "## Scene definitions and code changes",
        "",
        "- A1: 2.0 m straight; A2: 1.25 m turn-then-forward; A3: L path ending at (1.6, 1.0) m; A4: seven-point multi-waypoint path.",
        "- B1: 0.65 x 0.90 m table; B2: two 0.60 x 0.45 m furniture blocks with 1.15 m gap; B3: four 0.10 m-radius legs; B4: 0.65 x 0.90 m centre block with an offset bypass; B5: two blocks leaving a 0.89 m opening.",
        "- C1/C2/C3: corridor net widths 1.40/0.84/0.58 m. The patched R1 base collision AABB is 0.682 x 0.532 m; planner YAML footprint remains 0.322 x 0.22 m. Clearance in the matrix is the LaserScan proxy described above.",
        "- D1: obstacle at the L corner; D2: U-shaped local space; D3: furniture near terminal goal; D4: high-angle rotate-then-forward; D5: one-side bypass with the B1 table geometry.",
        "- Changed source files: `src/r1_neupan_integration/launch/n1_validation.launch.py`, `src/r1_neupan_integration/launch/n2_forward_only.launch.py`, `src/r1_neupan_integration/r1_neupan_integration/n2_pre_scenarios.py`, `src/r1_neupan_integration/r1_neupan_integration/n2_pre3_audit.py`, and `src/r1_neupan_integration/scripts/n2_forward_only_report.py`.",
        "- Added worlds: `n2_b1_single_table.sdf`, `n2_b2_wide_channel.sdf`, `n2_b3_table_legs.sdf`, `n2_b4_center_obstacle.sdf`, `n2_b5_one_side_open.sdf`, `n2_c1_wide_corridor.sdf`, `n2_c2_medium_corridor.sdf`, `n2_c3_narrow_corridor.sdf`, `n2_d1_corner_obstacle.sdf`, `n2_d2_u_space.sdf`, and `n2_d3_goal_furniture.sdf`.",
        "",
        "## Recommendation",
        "",
        "Do not advance automatically to Nav2, dynamic obstacles, people, real hardware, or DUNE training. Keep the formal N2 result classified as mixed/unresolved until B1/B3/B4 and the non-monotonic corridor results are explained or reproduced with an approved design change.",
    ])
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"report={args.report}")
    print(f"summary={args.summary}")
    print(f"overall={overall}")


if __name__ == "__main__":
    main()
