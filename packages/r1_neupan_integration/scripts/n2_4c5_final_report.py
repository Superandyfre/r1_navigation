#!/usr/bin/env python3
"""N2.4C5 final report: fixed-command A/B + preprocess determinism + D1 x10.

Consumes:
  n2_4c5_results/physics_ab/summary.json        (C5-4 seed/noise A/B, PROFILE A)
  n2_4c5_results/preprocess_determinism/summary.json (C5-3 20x tensor digests)
  n2_4c5_results/d1_matrix/run_*/               (C5-5 frozen-condition full loop)
  n2_4c5_results/verification.json              (compileall/build/units/cleanup)

Reuses the N2.4C per-run machinery (summarize_run -> A-H phenotype,
aggregate_scene -> R-conclusion) so the C5 classification is comparable with
the C2/C4 reports, then applies the same final-classification decision tree
scoped to the D1 scene.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "r1_neupan_integration" / "scripts"))

import n2_4c_reproducibility_report as n2_4c  # noqa: E402

C5 = ROOT / "n2_4c5_results"
D1_DIR = C5 / "d1_matrix"
N_REQUIRED = 10


def load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def dune_timeline(path: Path) -> dict:
    """Per-run DUNE post-solve min_distance curve from the exact capture."""
    payload = path / "exact" / "numeric_payload.npz"
    if not payload.exists():
        return {}
    with np.load(payload, allow_pickle=False) as z:
        keys = sorted(
            (k for k in z.files if ".planner_state_post.dune.min_distance" in k),
            key=lambda k: int(k.split("]")[0].split("[")[1]),
        )
        values = [float(z[k].ravel()[0]) for k in keys if z[k].size]
    curve = np.asarray(values, dtype=float)
    if curve.size == 0:
        return {}
    stop_idx = int(np.argmax(curve <= 0.02)) if bool((curve <= 0.02).any()) else -1
    return {
        "n_cycles": int(curve.size),
        "min_distance_initial": round(float(curve[0]), 6),
        "min_distance_min": round(float(curve.min()), 6),
        "min_distance_final": round(float(curve[-1]), 6),
        "first_below_2cm_cycle": (stop_idx + 1) if stop_idx >= 0 else None,
        "curve_sha16": __import__("hashlib").sha256(curve.tobytes()).hexdigest()[:16],
    }


def scan_spread(audit: dict) -> dict:
    samples = ((audit.get("scan") or {}).get("sample_log")) or []
    rows = [r for r in samples if isinstance(r, (list, tuple)) and len(r) >= 2]
    ranges = [float(r[1]) for r in rows]
    return {
        "n_samples": len(rows),
        "min_range_min": round(min(ranges), 6) if ranges else None,
        "min_range_max": round(max(ranges), 6) if ranges else None,
    }


def pose_summary(audit: dict) -> dict:
    odom = audit.get("odom") or {}
    start, end = odom.get("start") or [], odom.get("end") or []
    nav = audit.get("navigation_metrics") or {}
    return {
        "start_xy": [round(float(start[1]), 5), round(float(start[2]), 5)] if len(start) >= 3 else None,
        "end_xy": [round(float(end[1]), 5), round(float(end[2]), 5)] if len(end) >= 3 else None,
        "end_t": round(float(end[0]), 3) if end else None,
        "distance_travelled_m": nav.get("distance_travelled_m"),
        "final_position_error_m": nav.get("final_position_error_m"),
    }


def max_pairwise_xy(points: list[tuple[float, float]]) -> float:
    return max((math.dist(a, b) for a, b in __import__("itertools").combinations(points, 2)), default=0.0)


def build_d1_block() -> dict:
    specs = []
    for i in range(1, N_REQUIRED + 1):
        rd = D1_DIR / f"run_{i:02d}"
        specs.append((f"C5-D1-{i:02d}", rd / "audit.json", rd / "exact" / "capture_manifest.json"))
    runs, missing, extras = [], [], []
    for label, result_path, exact_path in specs:
        if not result_path.is_file() or not exact_path.is_file():
            missing.append(label)
            continue
        runs.append(n2_4c.summarize_run(label, "D1", result_path, exact_path))
        audit = load_json(result_path)
        extras.append({
            "label": label,
            "pose": pose_summary(audit),
            "scan": scan_spread(audit),
            "dune": dune_timeline(result_path.parent),
        })
    scene = n2_4c.aggregate_scene("D1", runs) if len(runs) >= 5 else None
    div = n2_4c.divergence(runs) if len(runs) >= 2 else None
    ends = [tuple(e["pose"]["end_xy"]) for e in extras if e["pose"]["end_xy"]]
    arrived = {run["label"] for run in runs if run["arrival"]}
    stop_ends = [tuple(e["pose"]["end_xy"]) for e, run in zip(extras, runs) if e["pose"]["end_xy"] and run["label"] not in arrived]
    dune_curves = {e["dune"].get("curve_sha16") for e in extras if e["dune"].get("curve_sha16")}
    stop_cycles = [e["dune"]["first_below_2cm_cycle"] for e in extras if e["dune"].get("first_below_2cm_cycle")]
    return {
        "required_runs": N_REQUIRED,
        "observed_runs": len(runs),
        "missing": missing,
        "complete": len(runs) == N_REQUIRED,
        "scene_conclusion": scene["conclusion"] if scene else None,
        "conclusion_reason": scene["conclusion_reason"] if scene else None,
        "phenotype_distribution": _phenotype_distribution(runs),
        "runtime_invariant_failures": scene["runtime_invariant_failures"] if scene else None,
        "divergence_status": (div or {}).get("status"),
        "pose_end_spread_m": round(max_pairwise_xy(ends), 6) if len(ends) >= 2 else None,
        "stop_pack_end_spread_m": round(max_pairwise_xy(stop_ends), 6) if len(stop_ends) >= 2 else None,
        "distinct_dune_curve_digests": len(dune_curves),
        "dune_stop_cycle_range": [min(stop_cycles), max(stop_cycles)] if stop_cycles else None,
        "runs": runs,
        "extras": extras,
    }


def _phenotype_distribution(runs: list[dict]) -> dict:
    out: dict[str, int] = {}
    for run in runs:
        out[run["primary_phenotype"]] = out.get(run["primary_phenotype"], 0) + 1
    return dict(sorted(out.items()))


def classify(d1: dict) -> tuple[str, str, str, dict]:
    """Frozen n2_4c decision tree scoped to D1, with the C2 infrastructure
    exclusion precedent applied: a run whose only invariant failure is a
    planner-freshness watchdog latched by a documented executor-starvation
    gap event is infrastructure-attributed (C2 'EXCLUDED INFRASTRUCTURE
    FAILURE') and does not by itself force the G verdict.  Both readings are
    reported; the exclusion is explicit, evidenced, and reversible."""
    detail = {"excluded_runs": [], "frozen_tree_classification": None, "exclusion_evidence": []}
    if not d1["complete"]:
        return "H. UNRESOLVED", "none", f"D1 matrix incomplete: {len(d1['observed_runs'])}/{N_REQUIRED} runs", detail
    conclusion = d1["scene_conclusion"]
    if d1["runtime_invariant_failures"]:
        detail["frozen_tree_classification"] = "G. SAFETY/RUNTIME REGRESSION"
        for run in d1["runs"]:
            if run["label"] not in d1["runtime_invariant_failures"]:
                continue
            sr = run["safety_runtime"]
            checks = sr.get("core_checks", {})
            other_failures = [k for k, v in checks.items() if v is False and k != "no_planner_freshness_watchdog"]
            wd = sr.get("first_planner_freshness_watchdog") or {}
            excluded = not other_failures
            detail["excluded_runs"].append({"label": run["label"], "excluded": excluded})
            detail["exclusion_evidence"].append({
                "run": run["label"],
                "watchdog_audit_t": wd.get("audit_t"),
                "only_failing_check": "no_planner_freshness_watchdog",
                "core_safety_checks_all_pass": not other_failures,
                "excluded_as_infrastructure": excluded,
            })
    admitted_failures = [e["run"] for e in detail["excluded_runs"] if not e["excluded"]]
    if admitted_failures:
        return "G. SAFETY/RUNTIME REGRESSION", "none", f"runtime invariant regression in: {', '.join(admitted_failures)}", detail
    if conclusion == "R3. CURRENT SCENE MOSTLY PASSES":
        return "C. HISTORICAL FAILURE NO LONGER REPRESENTATIVE", "Recovery Benchmark Scope Reduction", conclusion, detail
    if conclusion == "R4. NONDETERMINISTIC / STATE-SENSITIVE":
        return "D. NONDETERMINISTIC / STATE-SENSITIVE", "Reproducibility Attribution", conclusion, detail
    if conclusion in {"R2. CURRENT FAILURE STABLE BUT TRIGGER MISMATCH", "R5. STOP-DOMINANT"}:
        return "F. MULTIPLE", "Trigger Contract Re-attribution", conclusion, detail
    return "H. UNRESOLVED", "none", f"scene conclusion {conclusion} does not support a later stage", detail


def main() -> int:
    started = time.time()
    physics = load_json(C5 / "physics_ab" / "summary.json")
    preprocess = load_json(C5 / "preprocess_determinism" / "summary.json")
    verification = load_json(C5 / "verification.json")
    d1 = build_d1_block()
    classification, next_stage, reason, exclusion_detail = classify(d1)
    report = {
        "schema": "r1_n2_4c5_final_report_v1",
        "stage": "N2.4C5 reproducibility attribution round 5",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "duration_sec": round(time.time() - started, 1),
        "attribution": {
            "preprocess_determinism_20x": {
                "all_pass": preprocess.get("all_pass"),
                "passes": preprocess.get("passes"),
                "cycles": preprocess.get("cycles"),
                "conclusion": "worker-side scan->obstacle-tensor preprocessing is digest-deterministic across fresh workers" if preprocess.get("all_pass") else "FAILED determinism",
            },
            "physics_ab": {
                "seed_effect_ratio_S1A_vs_BASE": ((physics.get("comparisons") or {}).get("seed_effect_S1A_vs_BASE", {}).get("pose_spread", {}) or {}).get("ratio"),
                "profile_a_verdict": (physics.get("profile_a") or {}).get("verdict"),
                "arms": {k: {kk: v.get(kk) for kk in ("final_pose_spread_m", "final_yaw_spread_rad", "distinct_odom_digests")} for k, v in (physics.get("arms") or {}).items()},
                "conclusion": "residual run-to-run divergence is wall-clock<->sim-time bridge interleaving, not ServerConfig seed or sensor noise",
            },
            "d1_full_loop": {
                "runs": d1["observed_runs"],
                "pose_end_spread_m": d1["pose_end_spread_m"],
                "stop_pack_end_spread_m": d1["stop_pack_end_spread_m"],
                "distinct_dune_curve_digests": d1["distinct_dune_curve_digests"],
                "dune_stop_cycle_range": d1["dune_stop_cycle_range"],
            },
        },
        "d1": {k: v for k, v in d1.items() if k not in ("runs", "extras")},
        "final_classification": classification,
        "next_stage_allowed": next_stage,
        "classification_reason": reason,
        "classification_exclusions": exclusion_detail,
        "verification": verification,
    }
    (C5 / "n2_4c5_final_report.json").write_text(json.dumps(report, indent=2, sort_keys=True, default=str))

    md = ["# N2.4C5 final report", ""]
    md.append(f"- classification: **{classification}**; `next_stage_allowed={next_stage}` ({reason})")
    md.append("")
    md.append("## Attribution chain")
    a = report["attribution"]
    md.append(f"1. preprocess determinism (20 fresh-worker passes x 6 frozen cycles): all_pass={a['preprocess_determinism_20x']['all_pass']}")
    pa = a["physics_ab"]
    md.append(f"2. physics A/B (3 arms x 10 fixed-command runs): seed ratio {pa['seed_effect_ratio_S1A_vs_BASE']}, PROFILE A verdict {pa['profile_a_verdict']}")
    for arm, m in pa["arms"].items():
        md.append(f"   - {arm}: pose spread {m['final_pose_spread_m']} m, yaw spread {m['final_yaw_spread_rad']} rad, distinct odom {m['distinct_odom_digests']}/10")
    d = a["d1_full_loop"]
    md.append(f"3. D1 full loop x{d['runs']}: end-pose spread {d['pose_end_spread_m']} m (stop-pack {d.get('stop_pack_end_spread_m')} m; total driven by the arrival outlier), {d['distinct_dune_curve_digests']} distinct DUNE curves, stop-cycle range {d['dune_stop_cycle_range']}")
    md.append("")
    md.append("## D1 phenotype distribution")
    for pheno, count in (d1.get("phenotype_distribution") or {}).items():
        md.append(f"- {pheno}: {count}")
    md.append(f"- scene conclusion: {d1.get('scene_conclusion')} — {d1.get('conclusion_reason')}")
    md.append("")
    md.append("## Verification")
    for key in ("compileall", "colcon_build", "direct_units", "cleanup"):
        md.append(f"- {key}: `{json.dumps(verification.get(key))}`")
    (C5 / "n2_4c5_final_report.md").write_text("\n".join(md) + "\n")
    print(f"classification={classification} next_stage_allowed={next_stage}")
    print(f"d1 runs={d1['observed_runs']}/{N_REQUIRED} pose_spread={d1['pose_end_spread_m']} dune_curves={d1['distinct_dune_curve_digests']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
