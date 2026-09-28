#!/usr/bin/env python3
"""N2.4C4 Experiment D analysis: fixed-command Gazebo repeatability.

Read-only consumer of the sealed ``n2_4c4_fixed_command_replay.py`` outputs.
It never relaunches Gazebo, never edits the world/path/safety, and never
touches planner, DUNE, trigger/FSM state.  For each recorded run it aligns
odometry to the frozen command schedule by interpolation on the harness's own
relative ``t_sec`` clock, then quantifies pose/yaw spread, sensor digest
divergence, the first external divergence, and the safety boundary across the
five whole-core Gazebo launches.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

POSE_TOL = 1e-6  # m / rad; below this a difference is integration noise level
SCAN_SLOT_SEC = 0.1  # sensor publisher cadence in sim time


def _interp_pose(odom: list[dict[str, Any]], t: float) -> dict[str, float] | None:
    """Linear interpolation of x/y/yaw on the relative t_sec clock."""
    if not odom:
        return None
    times = [row["t_sec"] for row in odom]
    if t <= times[0]:
        row = odom[0]
        return {"x": row["x"], "y": row["y"], "yaw": row["yaw"]}
    if t >= times[-1]:
        row = odom[-1]
        return {"x": row["x"], "y": row["y"], "yaw": row["yaw"]}
    for i in range(1, len(times)):
        if times[i] >= t:
            t0, t1 = times[i - 1], times[i]
            a, b = odom[i - 1], odom[i]
            w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return {
                key: a[key] + w * (b[key] - a[key])
                for key in ("x", "y", "yaw")
            }
    row = odom[-1]
    return {"x": row["x"], "y": row["y"], "yaw": row["yaw"]}


def _slot_sensors(rows: list[dict[str, Any]], digest_key: str) -> dict[int, dict[str, Any]]:
    """Bin sensor frames into fixed 0.1 s slots on the relative clock."""
    slots: dict[int, dict[str, Any]] = {}
    for row in rows:
        slot = int(math.floor(row["t_sec"] / SCAN_SLOT_SEC))
        prev = slots.get(slot)
        if prev is None or row["t_sec"] < prev["t_sec"]:
            slots[slot] = row
    return slots


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def analyze_run(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    finite_ok = True
    for row in data["odom"]:
        finite_ok &= all(_finite(row.get(k)) for k in ("x", "y", "yaw", "v", "w"))
    for row in data["scan"]:
        finite_ok &= _finite(row.get("min_range"))
    boundary = data.get("safety_boundary") or {}
    return {
        "run": path.parent.name,
        "status": data.get("status"),
        "counts": data.get("counts"),
        "odom": data["odom"],
        "scan": data["scan"],
        "cloud": data["cloud"],
        "schedule": data["schedule"],
        "safety_boundary": boundary,
        "finite_fields_ok": bool(finite_ok),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", default="n2_4c4_results/experiment_d")
    parser.add_argument("--output", default="n2_4c4_results/experiment_d/experiment_d_summary.json")
    parser.add_argument("--pose-tol", type=float, default=POSE_TOL)
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    run_paths = sorted(results_dir.glob("run_*/replay.json"))
    if not run_paths:
        raise SystemExit(f"no replay.json under {results_dir}")
    runs = [analyze_run(path) for path in run_paths]

    schedule = runs[0]["schedule"]
    schedule_identical = all(run["schedule"] == schedule for run in runs)
    command_linear_min = min(row["linear_x"] for row in schedule)
    command_linear_nonnegative = command_linear_min >= 0.0

    initial = []
    for run in runs:
        first = run["odom"][0]
        initial.append({k: first[k] for k in ("x", "y", "yaw")})
    initial_spread = {
        key: max(v[key] for v in initial) - min(v[key] for v in initial)
        for key in ("x", "y", "yaw")
    }
    initial_pose_equal = all(abs(v) <= args.pose_tol for v in initial_spread.values())

    per_cycle = []
    first_pose_divergence: dict[str, Any] | None = None
    for idx, cmd in enumerate(schedule):
        t = cmd["t_sec"]
        poses = []
        for run in runs:
            pose = _interp_pose(run["odom"], t)
            if pose is None:
                poses = []
                break
            poses.append(pose)
        row: dict[str, Any] = {"cycle": idx + 1, "t_sec": t}
        if not poses:
            row["pose_available"] = False
            per_cycle.append(row)
            continue
        spread = {
            key: max(p[key] for p in poses) - min(p[key] for p in poses)
            for key in ("x", "y", "yaw")
        }
        row["pose_available"] = True
        row["spread"] = spread
        row["poses"] = [{k: p[k] for k in ("x", "y", "yaw")} for p in poses]
        per_cycle.append(row)
        if first_pose_divergence is None:
            worst = max(spread.values())
            if worst > args.pose_tol:
                first_pose_divergence = {
                    "kind": "pose",
                    "cycle": idx + 1,
                    "t_sec": t,
                    "max_abs_diff": worst,
                    "spread": spread,
                }

    scan_slots = [(_slot_sensors(run["scan"], "digest")) for run in runs]
    cloud_slots = [(_slot_sensors(run["cloud"], "data_digest")) for run in runs]
    common_scan_slots = sorted(set.intersection(*(set(s) for s in scan_slots)))
    common_cloud_slots = sorted(set.intersection(*(set(s) for s in cloud_slots)))

    scan_rows = []
    first_scan_divergence: dict[str, Any] | None = None
    for slot in common_scan_slots:
        frames = [slots[slot] for slots in scan_slots]
        min_range = [f["min_range"] for f in frames]
        digests = {f["digest"] for f in frames}
        row = {
            "slot": slot,
            "t_sec": [f["t_sec"] for f in frames],
            "min_range": min_range,
            "min_range_spread": max(min_range) - min(min_range),
            "digests_unique": len(digests),
        }
        scan_rows.append(row)
        if first_scan_divergence is None and (len(digests) > 1 or row["min_range_spread"] > args.pose_tol):
            first_scan_divergence = {
                "kind": "scan",
                "slot": slot,
                "t_sec": [f["t_sec"] for f in frames],
                "min_range": min_range,
                "min_range_spread": row["min_range_spread"],
                "digests_unique": len(digests),
            }

    cloud_rows = []
    first_cloud_divergence: dict[str, Any] | None = None
    for slot in common_cloud_slots:
        frames = [slots[slot] for slots in cloud_slots]
        digests = {f["data_digest"] for f in frames}
        row = {"slot": slot, "t_sec": [f["t_sec"] for f in frames], "digests_unique": len(digests)}
        cloud_rows.append(row)
        if first_cloud_divergence is None and len(digests) > 1:
            first_cloud_divergence = dict(row)

    safety = {
        "all_runs_completed": all(run["status"] == "completed" for run in runs),
        "command_linear_min": command_linear_min,
        "command_linear_nonnegative": command_linear_nonnegative,
        "contact_zero": all((run["safety_boundary"] or {}).get("contact_zero") is True for run in runs),
        "contact_total": sum(run["counts"].get("contacts", 0) for run in runs),
        "finite_fields_ok": all(run["finite_fields_ok"] for run in runs),
        "worker_failures": sum(1 for run in runs if run["status"] != "completed"),
    }

    pose_divergence_confirmed = bool(
        initial_pose_equal
        and first_pose_divergence is not None
        and first_pose_divergence["max_abs_diff"] > args.pose_tol
    )
    candidates = [c for c in (first_pose_divergence, first_scan_divergence, first_cloud_divergence) if c]
    first_external_divergence = (
        min(candidates, key=lambda c: c.get("t_sec", c.get("cycle", 0)) if isinstance(c.get("t_sec"), float) else c.get("cycle", 0))
        if candidates
        else None
    )

    pose_spread_max = {
        key: max((row["spread"][key] for row in per_cycle if row.get("pose_available")), default=0.0)
        for key in ("x", "y", "yaw")
    }
    reference_cycles = {}
    for cycle in (10, 20, 30, 40, 45, 46, 47):
        if cycle <= len(per_cycle):
            row = per_cycle[cycle - 1]
            reference_cycles[str(cycle)] = row.get("spread") if row.get("pose_available") else None

    summary = {
        "schema": "r1_n2_4c4_experiment_d_v1",
        "status": "EXECUTED",
        "runs": len(runs),
        "horizon_sec": max(cmd["t_sec"] for cmd in schedule),
        "schedule_identical": schedule_identical,
        "initial_pose_equal": initial_pose_equal,
        "initial_pose_spread": initial_spread,
        "pose_divergence_confirmed": pose_divergence_confirmed,
        "first_external_divergence": first_external_divergence,
        "first_pose_divergence": first_pose_divergence,
        "first_scan_divergence": first_scan_divergence,
        "first_cloud_divergence": first_cloud_divergence,
        "pose_spread_max": pose_spread_max,
        "reference_cycle_spread": reference_cycles,
        "scan_slots_compared": len(scan_rows),
        "scan_digest_divergent_slots": sum(1 for row in scan_rows if row["digests_unique"] > 1),
        "scan_min_range_spread_max": max((row["min_range_spread"] for row in scan_rows), default=0.0),
        "cloud_slots_compared": len(cloud_rows),
        "cloud_digest_divergent_slots": sum(1 for row in cloud_rows if row["digests_unique"] > 1),
        "safety": safety,
        "checkpoints": [
            f"runs={len(runs)} horizon={max(cmd['t_sec'] for cmd in schedule):.3f}s schedule_identical={schedule_identical}",
            f"initial_pose_equal={initial_pose_equal} spread<= {args.pose_tol:g}",
            f"first divergence: {json.dumps(first_external_divergence) if first_external_divergence else 'none'}",
            f"pose spread max x/y/yaw = {pose_spread_max['x']:.6f}/{pose_spread_max['y']:.6f}/{pose_spread_max['yaw']:.6f}",
            f"scan divergent slots {sum(1 for row in scan_rows if row['digests_unique'] > 1)}/{len(scan_rows)}; cloud divergent slots {sum(1 for row in cloud_rows if row['digests_unique'] > 1)}/{len(cloud_rows)}",
            f"safety: linear>=0={command_linear_nonnegative} contacts={safety['contact_total']} finite={safety['finite_fields_ok']} worker_failures={safety['worker_failures']}",
        ],
        "per_cycle_pose_spread": [
            {"cycle": row["cycle"], "t_sec": row["t_sec"], **row.get("spread", {})}
            for row in per_cycle
            if row.get("pose_available")
        ],
        "scan_slot_rows": scan_rows,
        "cloud_slot_rows": cloud_rows,
    }

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}")
    for line in summary["checkpoints"]:
        print("-", line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
