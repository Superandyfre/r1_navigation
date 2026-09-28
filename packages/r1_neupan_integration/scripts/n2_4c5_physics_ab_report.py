#!/usr/bin/env python3
"""N2.4C5-4: aggregate the physics A/B fixed-command matrix into a report.

Reads n2_4c5_results/physics_ab/{BASE,S1A,S1B}/run_*/ and computes, per arm:
run health, final-pose spread, distinct odom trajectory digests, distinct
scan digest sequences, and per-slot min_range spread.  Then classifies:

  seed effect   -> S1A vs BASE   (physics-only variance)
  noise effect  -> S1B vs S1A    (sensor-noise-only variance)
  PROFILE A     -> S1B deterministic when every run shares one odom digest
                   and one scan sequence digest
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MATRIX_DIR = ROOT / "n2_4c5_results" / "physics_ab"
ARMS = ("BASE", "S1A", "S1B")
ODOM_QUANT = 4  # decimals for trajectory digest


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def odom_digest(odom: list[dict]) -> str:
    rows = [f"{round(r['x'], ODOM_QUANT)},{round(r['y'], ODOM_QUANT)},{round(r['yaw'], ODOM_QUANT)}" for r in odom]
    return hashlib.sha256("\n".join(rows).encode()).hexdigest()[:16]


def scan_sequence_digest(scan: list[dict]) -> str:
    return hashlib.sha256("\n".join(r["digest"] for r in scan).encode()).hexdigest()[:16]


def max_pairwise(values: list[tuple[float, float]]) -> float:
    return max((math.dist(a, b) for a, b in itertools.combinations(values, 2)), default=0.0)


def max_pairwise_yaw(yaws: list[float]) -> float:
    return max((abs(wrap_angle(a - b)) for a, b in itertools.combinations(yaws, 2)), default=0.0)


def load_run(run_dir: Path) -> dict:
    row: dict = {"run": run_dir.name, "dir": str(run_dir)}
    replay_path = run_dir / "replay.json"
    health_path = run_dir / "health.json"
    try:
        health = json.loads(health_path.read_text())
        row["replay_status"] = health.get("replay_status")
        row["recorder_exit_code"] = health.get("recorder_exit_code")
    except Exception as exc:
        row["replay_status"] = f"health_unreadable: {exc}"
    if not replay_path.exists():
        row["status"] = "missing_replay_json"
        return row
    try:
        replay = json.loads(replay_path.read_text())
    except Exception as exc:
        row["status"] = f"replay_unreadable: {exc}"
        return row
    odom = replay.get("odom") or []
    scan = replay.get("scan") or []
    final = odom[-1] if odom else None
    row.update({
        "status": replay.get("status"),
        "n_odom": len(odom),
        "n_scan": len(scan),
        "contacts_max": max((c.get("count", 0) for c in replay.get("contacts", [])), default=0),
        "final_x": final["x"] if final else None,
        "final_y": final["y"] if final else None,
        "final_yaw": final["yaw"] if final else None,
        "path_len": math.fsum(
            math.dist((a["x"], a["y"]), (b["x"], b["y"])) for a, b in zip(odom, odom[1:])
        ) if len(odom) > 1 else 0.0,
        "odom_digest": odom_digest(odom) if odom else None,
        "scan_seq_digest": scan_sequence_digest(scan) if scan else None,
        "min_ranges": [r["min_range"] for r in scan],
    })
    row["completed"] = row["status"] == "completed" and row["n_odom"] > 0 and row["n_scan"] > 0
    return row


def arm_metrics(arm: str, runs: list[dict]) -> dict:
    done = [r for r in runs if r.get("completed")]
    poses = [(r["final_x"], r["final_y"]) for r in done]
    yaws = [r["final_yaw"] for r in done]
    per_slot_min = {}
    for r in done:
        for i, value in enumerate(r["min_ranges"]):
            lo, hi = per_slot_min.get(i, (math.inf, -math.inf))
            per_slot_min[i] = (min(lo, value), max(hi, value))
    slot_spread = {i: {"min": lo, "max": hi, "spread": hi - lo} for i, (lo, hi) in sorted(per_slot_min.items())}
    return {
        "n_runs": len(runs),
        "n_completed": len(done),
        "statuses": sorted({str(r.get("status")) for r in runs}),
        "final_pose_spread_m": round(max_pairwise(poses), 6) if done else None,
        "final_yaw_spread_rad": round(max_pairwise_yaw(yaws), 6) if done else None,
        "distinct_odom_digests": len({r["odom_digest"] for r in done}),
        "distinct_scan_seq_digests": len({r["scan_seq_digest"] for r in done}),
        "min_range_slot_spread": {str(k): round(v["spread"], 6) for k, v in list(slot_spread.items())[:5]},
        "min_range_slot0_spread_max": round(max((v["spread"] for v in slot_spread.values()), default=0.0), 6),
        "deterministic": bool(done) and len(done) == len(runs) and len({r["odom_digest"] for r in done}) == 1 and len({r["scan_seq_digest"] for r in done}) == 1,
    }


def main() -> int:
    arms: dict[str, dict] = {}
    for arm in ARMS:
        run_dirs = sorted((MATRIX_DIR / arm).glob("run_*")) if (MATRIX_DIR / arm).exists() else []
        runs = [load_run(d) for d in run_dirs if d.is_dir()]
        arms[arm] = {"metrics": arm_metrics(arm, runs), "runs": runs}

    base, s1a, s1b = (arms[a]["metrics"] for a in ARMS)

    def ratio(delta: float | None, ref: float | None) -> float | None:
        if delta is None or ref in (None, 0.0):
            return None
        return round(delta / ref, 3)

    comparisons = {
        "seed_effect_S1A_vs_BASE": {
            "pose_spread": {"BASE": base["final_pose_spread_m"], "S1A": s1a["final_pose_spread_m"], "ratio": ratio(s1a["final_pose_spread_m"], base["final_pose_spread_m"])},
            "distinct_odom_digests": {"BASE": base["distinct_odom_digests"], "S1A": s1a["distinct_odom_digests"]},
            "distinct_scan_seq": {"BASE": base["distinct_scan_seq_digests"], "S1A": s1a["distinct_scan_seq_digests"]},
        },
        "noise_effect_S1B_vs_S1A": {
            "distinct_scan_seq": {"S1A": s1a["distinct_scan_seq_digests"], "S1B": s1b["distinct_scan_seq_digests"]},
            "distinct_odom_digests": {"S1A": s1a["distinct_odom_digests"], "S1B": s1b["distinct_odom_digests"]},
        },
    }
    profile_a = {
        "candidate_arm": "S1B",
        "complete": s1b["n_completed"] == s1b["n_runs"] and s1b["n_runs"] > 0,
        "odom_identical": s1b["distinct_odom_digests"] == 1,
        "scan_identical": s1b["distinct_scan_seq_digests"] == 1,
        "verdict": "PASS" if (s1b["n_runs"] > 0 and s1b["n_completed"] == s1b["n_runs"] and s1b["deterministic"]) else "FAIL",
    }
    summary = {
        "task": "N2.4C5-4 physics A/B fixed-command matrix",
        "arms": {arm: {"config": cfg, **arms[arm]["metrics"]} for arm, cfg in
                 (("BASE", "noise on, unseeded"), ("S1A", "noise on, gz seed 42"), ("S1B", "noise off, gz seed 42"))},
        "comparisons": comparisons,
        "profile_a": profile_a,
    }
    (MATRIX_DIR / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))

    lines = ["# N2.4C5-4 physics A/B report", ""]
    for arm in ARMS:
        m = arms[arm]["metrics"]
        lines.append(f"## {arm} — {dict(BASE='noise on, unseeded', S1A='noise on, seed 42', S1B='noise off, seed 42')[arm]}")
        lines.append(f"- completed: {m['n_completed']}/{m['n_runs']} statuses={m['statuses']}")
        lines.append(f"- final pose spread: {m['final_pose_spread_m']} m, yaw spread: {m['final_yaw_spread_rad']} rad")
        lines.append(f"- distinct odom digests: {m['distinct_odom_digests']}, distinct scan sequences: {m['distinct_scan_seq_digests']}")
        lines.append(f"- min_range slot-0 spread: {m['min_range_slot_spread'].get('0')} (max over slots {m['min_range_slot0_spread_max']})")
        lines.append("")
    lines.append(f"- seed effect (S1A/BASE pose spread ratio): {comparisons['seed_effect_S1A_vs_BASE']['pose_spread']['ratio']}")
    lines.append(f"- PROFILE A verdict: {profile_a['verdict']} (odom_identical={profile_a['odom_identical']}, scan_identical={profile_a['scan_identical']})")
    (MATRIX_DIR / "physics_ab_report.md").write_text("\n".join(lines) + "\n")
    print(json.dumps({"profile_a": profile_a, "BASE": base, "S1A": s1a, "S1B": s1b}, indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
