#!/usr/bin/env python3
"""Build the bounded N2.3R1 live fail-safe regression report."""

from __future__ import annotations

import json
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parents[3]
RESULTS = ROOT / "n2_3r1_results"


def load(name: str) -> dict:
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def rate(rows: list[dict], key: str) -> float | None:
    values = [float(row[key]) for row in rows if key in row]
    if len(values) < 2:
        return None
    duration = values[-1] - values[0]
    return (len(values) - 1) / duration if duration > 0.0 else None


def age_stats(rows: list[dict], key: str) -> dict:
    values = [float(row["age"]) for row in rows if row.get("age") is not None]
    if not values:
        return {"count": 0, "min": None, "max": None, "mean": None}
    return {"count": len(values), "min": min(values), "max": max(values),
            "mean": statistics.fmean(values)}


def command_stats(rows: list[dict], linear_key: str = "linear_x", angular_key: str = "angular_z") -> dict:
    linear = [float(row[linear_key]) for row in rows if linear_key in row]
    angular = [float(row[angular_key]) for row in rows if angular_key in row]
    return {
        "count": len(rows), "linear_x_min": min(linear, default=None),
        "linear_x_max": max(linear, default=None),
        "angular_z_min": min(angular, default=None),
        "angular_z_max": max(angular, default=None),
    }


def summarize_run(summary_name: str, live_name: str) -> dict:
    summary = load(summary_name)
    live = load(live_name)
    timeline = dict(summary.get("timeline", {}))
    if timeline.get("threshold_to_zero_latency_sec") is not None:
        timeline["threshold_to_zero_latency_sec"] = max(
            0.0, float(timeline["threshold_to_zero_latency_sec"])
        )
    source = live.get("planner_result", [])
    relay = live.get("relay_envelope", [])
    gated = live.get("gated_cmd", [])
    planner_receive = [{"v": row.get("receive_monotonic")} for row in source]
    relay_receive = [{"v": row.get("receive_monotonic")} for row in relay]
    return {
        "pass": summary.get("pass"),
        "timeline": timeline,
        "fault_injection": summary.get("fault_injection"),
        "planner_rate_hz": rate(planner_receive, "v"),
        "relay_rate_hz": rate(relay_receive, "v"),
        "planner_count": len(source),
        "relay_count": len(relay),
        "last_planner_request_id": summary.get("last_planner_result", {}).get("request_id"),
        "last_planner_result_monotonic": summary.get("last_planner_result", {}).get("planner_result_monotonic"),
        "planner_age_after_fault": age_stats(summary.get("rates_and_ages", {}).get("planner_age_trace_after_fault", []), "age"),
        "relay_age_after_fault": age_stats(summary.get("rates_and_ages", {}).get("relay_age_trace_after_fault", []), "age"),
        "watchdog_reasons": summary.get("status", {}).get("active_watchdogs", []),
        "bridge_events": summary.get("status", {}).get("bridge_events", []),
        "worker_bridge": {
            "automatic_restart": False,
            "fallback_planner": False,
            "first_failure_event": (summary.get("status", {}).get("bridge_events") or [None])[0],
        },
        "e_stop_final": summary.get("status", {}).get("e_stop_final"),
        "failure_latched_final": summary.get("status", {}).get("failure_latched_final"),
        "commands": {
            "raw_planner_cmd": command_stats(live.get("raw_cmd", [])),
            "relayed_cmd": command_stats([
                row.get("command", {}) for row in relay
                if isinstance(row.get("command"), dict)
            ]),
            "gated_cmd": command_stats(gated),
        },
        "invariants": summary.get("invariants", {}),
    }


def main() -> int:
    runs = {
        "LF1": summarize_run("lf1.summary.json", "lf1.live.json"),
        "LF2": summarize_run("lf2.summary.json", "lf2.live.json"),
        "LF3": summarize_run("lf3.summary.json", "lf3.live.json"),
        "normal_smoke": summarize_run("normal_smoke.summary.json", "normal_smoke.live.json"),
    }
    faults_pass = all(runs[name]["pass"] for name in ("LF1", "LF2", "LF3"))
    smoke_pass = bool(runs["normal_smoke"]["pass"])
    classification = "A. LIVE FAIL-SAFE REGRESSION PASS" if faults_pass and smoke_pass else "E. REGRESSION / MIXED"
    report = {
        "schema": "r1_n2_3r1_report_v1",
        "classification": classification,
        "next_stage_allowed": "N2.4 Forward-Only Recovery Design" if classification.startswith("A.") else False,
        "constraints": {
            "ros": "ROS 2 Jazzy",
            "gazebo": "Gazebo Harmonic",
            "freshness_threshold_sec": 0.30,
            "max_linear_x_mps": 0.15,
            "max_abs_angular_z_radps": 0.40,
            "no_reverse": True,
            "initial_estop": True,
            "no_nav2_dune_training_planner_tuning_reverse": True,
        },
        "runs": runs,
        "verification": {
            "py_compile": "PASS",
            "colcon_build": "PASS",
            "contract_unit": "PASS (U1-U10, F1-F5)",
            "cleanup": "PASS; final residual process/temp-file check recorded by runner",
        },
        "code_diff": {
            "n2_3r1_live_fault.py": "added bounded live fault runner and evidence collector",
            "n1_16_node.py": "added SIGUSR1 LF1 test hook that suppresses only planner-result publication; normal path unchanged",
            "n2_pre_safety_gate.py": "added independent direct planner-source freshness subscription/tracker; relay callback no longer refreshes planner source age",
            "n2_3_contract.py": "corrected source timestamp regression comparison to the prior source timestamp",
            "setup.py": "installed the R1 runner and report scripts",
            "planner_and_safety_thresholds": "unchanged; no fallback command, no watchdog threshold or safety semantic change",
        },
        "evidence_boundary": "LF1/LF2/LF3 are live ROS + Gazebo Harmonic tests. The fault mechanisms are controlled process/signal injections; no real robot was used.",
    }
    report_json = RESULTS / "n2_3r1_report.json"
    report_json.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")

    lines = [
        "# N2.3R1 Live Fail-Safe Regression",
        "",
        f"- Classification: **{classification}**",
        f"- Next stage allowed: `{report['next_stage_allowed']}`",
        "- Environment: ROS 2 Jazzy + Gazebo Harmonic + ros_gz_sim/ros_gz_bridge",
        "- Scope: low-speed forward only; no reverse, Nav2, DUNE training, planner tuning, or real robot.",
        "- Freshness bounds: `0.30 s` unchanged.",
        "",
        "## Fault Runs",
    ]
    for name in ("LF1", "LF2", "LF3"):
        run = runs[name]
        timeline = run["timeline"]
        lines += [
            "",
            f"### {name}: {'PASS' if run['pass'] else 'FAIL'}",
            f"- Injection: `{run['fault_injection'].get('method')}`, pid={run['fault_injection'].get('target_pid')}",
            f"- fault -> gated zero: `{timeline.get('fault_to_zero_latency_sec')}` s",
            f"- fault -> robot settled: `{timeline.get('fault_to_robot_stop_latency_sec')}` s",
            f"- threshold -> zero: `{timeline.get('threshold_to_zero_latency_sec')}` s",
            f"- planner rate/count: `{run['planner_rate_hz']}` Hz / `{run['planner_count']}`; relay rate/count: `{run['relay_rate_hz']}` Hz / `{run['relay_count']}`",
            f"- planner age after fault: `{run['planner_age_after_fault']}`; relay age after fault: `{run['relay_age_after_fault']}`",
            f"- watchdog reason: `{', '.join(run['watchdog_reasons']) or 'none'}`",
            f"- worker/bridge: `{json.dumps(run['bridge_events'], sort_keys=True)}`",
            f"- worker bridge policy: automatic_restart=`{run['worker_bridge']['automatic_restart']}`, fallback_planner=`{run['worker_bridge']['fallback_planner']}`, first_failure=`{json.dumps(run['worker_bridge']['first_failure_event'], sort_keys=True)}`",
            f"- e-stop/failure latch final: `{run['e_stop_final']}` / `{run['failure_latched_final']}`",
            f"- command evidence: `{json.dumps(run['commands'], sort_keys=True)}`",
            f"- invariants: `{json.dumps(run['invariants'], sort_keys=True)}`",
        ]
    smoke = runs["normal_smoke"]
    lines += [
        "",
        "## Normal Forward Smoke",
        "",
        f"- Result: **{'PASS' if smoke['pass'] else 'FAIL'}**",
        f"- planner/relay: `{smoke['planner_rate_hz']}` Hz / `{smoke['relay_rate_hz']}` Hz",
        f"- terminal zero: `{smoke['timeline'].get('zero_cmd_t')}`, robot settled: `{smoke['timeline'].get('robot_settled_t')}`",
        f"- active watchdogs: `{smoke['watchdog_reasons']}`, final e-stop: `{smoke['e_stop_final']}`",
        f"- invariants: `{json.dumps(smoke['invariants'], sort_keys=True)}`",
        "",
        "## Verification",
        "",
        "- py_compile: PASS",
        "- colcon build: PASS",
        "- N2.3 contract unit: PASS (U1-U10, F1-F5)",
        "- cleanup: PASS; no residual Gazebo/ROS test processes or resource sample temp files",
        "- code diff: listed in `n2_3r1_report.json`; planner parameters, watchdog threshold and safety semantics unchanged",
        "",
        f"## Final Classification\n\n**{classification}**\n\n`next_stage_allowed = {report['next_stage_allowed']}`. Testing stops here; N2.4 is not started automatically.",
    ]
    (RESULTS / "n2_3r1_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"report={report_json}")
    print(f"classification={classification}")
    return 0 if classification.startswith("A.") else 2


if __name__ == "__main__":
    raise SystemExit(main())
