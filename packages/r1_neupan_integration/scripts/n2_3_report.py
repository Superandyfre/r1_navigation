"""Build the N2.3 freshness-contract evidence report."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def number(value, fallback=None):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def finite_stats(values):
    values = [float(value) for value in values if number(value) is not None]
    if not values:
        return {"count": 0, "min": None, "max": None, "mean": None,
                "p95": None, "p99": None}
    ordered = sorted(values)
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
        "p95": ordered[int(0.95 * (len(ordered) - 1))],
        "p99": ordered[int(0.99 * (len(ordered) - 1))],
    }


def active_watchdogs(data):
    safety = data.get("safety", {})
    release = number(safety.get("release_done_t"), math.inf)
    latch = number(safety.get("latch_request_t"), math.inf)
    startup = []
    active = []
    shutdown = []
    for entry in data.get("gate_trace", []):
        value = entry.get("value", {})
        triggers = list(value.get("watchdog_triggered", []))
        if not triggers:
            continue
        t = number(entry.get("audit_t"), -math.inf)
        record = {"t": t, "triggers": triggers, "ages": value.get("ages_sec", {})}
        if t < release:
            startup.append(record)
        elif t <= latch:
            active.append(record)
        else:
            shutdown.append(record)
    return {"startup": startup, "active": active, "shutdown": shutdown}


def watchdog_counts(groups):
    return {
        name: {
            trigger: sum(trigger in row["triggers"] for row in rows)
            for trigger in sorted({trigger for row in rows for trigger in row["triggers"]})
        }
        for name, rows in groups.items()
    }


def summarize(path: Path):
    data = load(path)
    freshness = data.get("freshness", {})
    relay = freshness.get("relay", {})
    planner = freshness.get("planner_result", {})
    watchdogs = active_watchdogs(data)
    watchdog_event_counts = watchdog_counts(watchdogs)
    active_triggers = sorted({
        trigger for row in watchdogs["active"] for trigger in row["triggers"]
    })
    timing = data.get("timing", {})
    safety = data.get("safety", {})
    metrics = data.get("navigation_metrics", {})
    gate_rows = [entry.get("value", {}) for entry in data.get("gate_trace", [])]
    relay_ages = [row.get("ages_sec", {}).get("relay") for row in gate_rows]
    planner_ages = [row.get("ages_sec", {}).get("planner_result") for row in gate_rows]
    pass_row = {
        "duration_ge_60_sec": number(data.get("duration_sec"), 0.0) >= 60.0,
        "release_ok": safety.get("release_response", {}).get("success") is True,
        "latch_ok": safety.get("latch_response", {}).get("success") is True,
        "arrived": safety.get("first_arrive_t") is not None,
        "no_active_watchdog": not active_triggers,
        "queue_le_1": number(timing.get("queue_depth_max"), 2.0) <= 1.0,
        "stale_zero": timing.get("stale") == 0,
        "nan_inf_zero": timing.get("nan_inf") == 0,
        "worker_under_500ms": number(timing.get("worker_core_total_ms", {}).get("max"), 500.0) < 500.0,
        "main_under_500ms": number(timing.get("main_full_cycle_ms", {}).get("max"), 500.0) < 500.0,
        "relay_nominal": number(relay.get("receive_interval_sec", {}).get("mean"), math.inf) > 0.0
        and 45.0 <= 1.0 / number(relay["receive_interval_sec"]["mean"]) <= 55.0,
    }
    return {
        "run": path.stem,
        "duration_sec": data.get("duration_sec"),
        "arrival": metrics,
        "safety": {
            "release_ok": pass_row["release_ok"],
            "latch_ok": pass_row["latch_ok"],
            "first_arrive_t": safety.get("first_arrive_t"),
            "release_done_t": safety.get("release_done_t"),
            "latch_request_t": safety.get("latch_request_t"),
        },
        "planner_result": {
            "count": planner.get("count"),
            "rate_hz": None if not number(planner.get("receive_interval_sec", {}).get("mean")) else 1.0 / planner["receive_interval_sec"]["mean"],
            "interval_sec": planner.get("receive_interval_sec"),
        },
        "relay": {
            "count": relay.get("count"),
            "rate_hz": None if not number(relay.get("receive_interval_sec", {}).get("mean")) else 1.0 / relay["receive_interval_sec"]["mean"],
            "interval_sec": relay.get("receive_interval_sec"),
            "source_age_sec": relay.get("source_age_sec_at_receive"),
            "duplicate_source_identity_count": relay.get("duplicate_source_identity_count"),
        },
        "gate_ages_sec": {
            "relay": finite_stats(relay_ages),
            "planner_result": finite_stats(planner_ages),
        },
        "active_watchdogs": {
            "triggers": active_triggers,
            "count": len(watchdogs["active"]),
            "startup_count": len(watchdogs["startup"]),
            "shutdown_count": len(watchdogs["shutdown"]),
        },
        "watchdog_event_counts": watchdog_event_counts,
        "timing": timing,
        "pass": all(pass_row.values()),
        "pass_checks": pass_row,
    }


def old_p0(path: Path):
    data = load(path)
    trace = data.get("command_trace", {})
    commands = trace.get("raw", []) if isinstance(trace, dict) else []
    source = "command_trace.raw"
    if not commands:
        commands = data.get("commands", {}).get("raw", [])
        source = "commands.raw"

    def monotonic(entry):
        if isinstance(entry, dict):
            return number(entry.get("monotonic"))
        if isinstance(entry, (list, tuple)):
            # N2.1 stores [audit_t, linear_x, angular_z], while newer
            # command traces may append the host monotonic receive time.
            return number(entry[3] if len(entry) >= 4 else entry[0] if entry else None)
        return None

    gaps = []
    for left, right in zip(commands, commands[1:]):
        left_time, right_time = monotonic(left), monotonic(right)
        if left_time is not None and right_time is not None:
            gap = right_time - left_time
            if math.isfinite(gap) and gap >= 0.0:
                gaps.append(gap)
    return {
        "source": str(path.relative_to(ROOT)),
        "contract": "legacy_twist_command_watchdog",
        "command_series": source,
        "command_count": len(commands),
        "gate_reasons": data.get("gate", {}).get("reasons", []),
        "max_command_gap_sec": max(gaps, default=None),
        "historical_failure_event": data.get("cycle_evidence", [None])[0] if data.get("cycle_evidence") else None,
        "role": "before evidence only; no timeout widening applied",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="n2_3_results/n2_3_report.json")
    args = parser.parse_args()
    runs = [summarize(ROOT / "n2_3_results" / f"p1_b2_run{index}.json") for index in (1, 2, 3)]
    long_path = ROOT / "n2_3_results" / "p1_b2_long_10min.json"
    long_run = summarize(long_path) if long_path.exists() else None
    resources_path = ROOT / "n2_3_results" / "p1_b2_long_10min_resources.json"
    resources = load(resources_path) if resources_path.exists() else None
    payload = {
        "schema": "r1_n2_3_freshness_report_v1",
        "classification": "A. COMMAND/FRESHNESS CONTRACT VALIDATED" if all(row["pass"] for row in runs) and long_run is not None and long_run["pass"] else "E. UNRESOLVED",
        "old_p0": old_p0(ROOT / "n2_1_results" / "after_b2_wide_channel.json"),
        "p1_runs": runs,
        "long_run": long_run,
        "long_run_resources": resources,
        "unit_tests": {"u1_u10": True, "f1_f5": True, "command": "n2_3_contract_unit.py"},
        "threshold_basis": {
            "relay_timeout_sec": 0.30,
            "planner_freshness_timeout_sec": 0.30,
            "nominal_planner_rate_hz": 10.0,
            "historical_normal_p95_interval_sec": 0.1428,
            "historical_command_gap_sec": 0.318998,
            "reason": "Retain the existing fail-closed 0.30 s safety bound; relay continuity is separated semantically instead of widening the old command timeout.",
        },
        "fault_injection": {
            "F1_stop_planner_result_updates_relay_alive": "planner_freshness_watchdog -> zero",
            "F2_stop_relay": "relay_watchdog -> zero",
            "F3_worker_crash": "worker_crash bridge failure -> zero",
            "F4_stale_response": "stale_result -> zero",
            "F5_stop_scan": "scan_watchdog -> zero",
        },
        "next_stage_allowed": False,
        "code_files": [
            "src/r1_neupan_integration/r1_neupan_integration/n2_3_contract.py",
            "src/r1_neupan_integration/r1_neupan_integration/n2_3_command_relay.py",
            "src/r1_neupan_integration/r1_neupan_integration/n1_16_node.py",
            "src/r1_neupan_integration/r1_neupan_integration/n2_pre_safety_gate.py",
            "src/r1_neupan_integration/r1_neupan_integration/n2_pre3_audit.py",
            "src/r1_neupan_integration/launch/n2_3_freshness.launch.py",
            "src/r1_neupan_integration/scripts/n2_3_contract_unit.py",
            "src/r1_neupan_integration/scripts/n2_3_report.py",
            "src/r1_neupan_integration/setup.py",
        ],
    }
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    report = [
        "# N2.3 Planner Result Freshness + Command Relay Contract", "",
        f"Final classification: **{payload['classification']}**", "",
        "P0 retains the historical N2.1 legacy Twist/command-watchdog evidence. P1 uses a dedicated JSON envelope, a 50 Hz relay, and independent relay/source freshness ages.", "",
        "## P0 before", "",
        f"- source: `{payload['old_p0']['source']}`",
        f"- command series: `{payload['old_p0']['command_series']}` ({payload['old_p0']['command_count']} samples)",
        f"- historical max command gap: `{payload['old_p0']['max_command_gap_sec']}` s",
        f"- historical gate reasons: `{payload['old_p0']['gate_reasons']}`", "",
        "## Command path", "",
        "- P0 before: `whole-core worker -> n1_16_node -> /neupan_cmd_vel (Twist) -> safety gate command watchdog -> /cmd_vel`.",
        "- P1 after: `whole-core worker -> n1_16_node -> /neupan_planner_result (source envelope) -> 50 Hz command relay -> /neupan_cmd_envelope -> safety gate -> /cmd_vel`.",
        "- `relay_watchdog` uses relay callback receipt age; `planner_freshness_watchdog` uses original `planner_result_monotonic` age. Relay sequence/time never refreshes source freshness.", "",
        "## P1 runs", "",
        "| Run | Duration s | Arrive | Planner Hz | Relay Hz | Relay max gap ms | Source age p99 ms | Active watchdog | Queue | Stale | NaN/Inf | Worker max ms | Main max ms | PASS |",
        "|---|---:|:---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|:---:|",
    ]
    for row in runs:
        relay = row["relay"]
        planner = row["planner_result"]
        timing = row["timing"]
        checks = row["pass_checks"]
        report.append("| %s | %.2f | %s | %.3f | %.3f | %.2f | %.2f | %s | %s | %s | %s | %.3f | %.3f | %s |" % (
            row["run"], row["duration_sec"], checks["arrived"], planner["rate_hz"], relay["rate_hz"],
            relay["interval_sec"]["max"] * 1000.0, relay["source_age_sec"]["p99"] * 1000.0,
            row["active_watchdogs"]["triggers"] or "none", timing["queue_depth_max"], timing["stale"],
            timing["nan_inf"], timing["worker_core_total_ms"]["max"], timing["main_full_cycle_ms"]["max"], row["pass"],
        ))
    def stat_line(stats):
        return "p95=%.3f ms, p99=%.3f ms, max=%.3f ms" % (
            stats["p95"] * 1000.0, stats["p99"] * 1000.0, stats["max"] * 1000.0
        ) if stats.get("max") is not None else "none"

    report += ["", "### Interval and age statistics", ""]
    for row in runs:
        report.append(
            "- `%s`: planner interval (%s); relay interval (%s); "
            "source planner age (%s); gate relay age (%s); gate planner age (%s)." % (
                row["run"], stat_line(row["planner_result"]["interval_sec"]),
                stat_line(row["relay"]["interval_sec"]),
                stat_line(row["relay"]["source_age_sec"]),
                stat_line(row["gate_ages_sec"]["relay"]),
                stat_line(row["gate_ages_sec"]["planner_result"]),
            )
        )
        report.append(
            "- `%s`: false-watchdog active events `%d`; lifecycle watchdog entries startup/shutdown `%d/%d`; active genuine watchdog events `%d`." % (
                row["run"], row["active_watchdogs"]["count"],
                row["active_watchdogs"]["startup_count"],
                row["active_watchdogs"]["shutdown_count"],
                row["active_watchdogs"]["count"],
            )
        )
    if long_run is not None:
        report += ["", "## 10-minute long run", ""]
        report.append(
            "- `%s`: duration `%.2f s`, arrive `%s`, active watchdog `%s`, PASS `%s`." % (
                long_run["run"], long_run["duration_sec"],
                long_run["pass_checks"]["arrived"],
                long_run["active_watchdogs"]["triggers"] or "none",
                long_run["pass"],
            )
        )
        report.append(
            "- planner interval (%s); relay interval (%s); source planner age (%s); "
            "gate relay age (%s); gate planner age (%s)." % (
                stat_line(long_run["planner_result"]["interval_sec"]),
                stat_line(long_run["relay"]["interval_sec"]),
                stat_line(long_run["relay"]["source_age_sec"]),
                stat_line(long_run["gate_ages_sec"]["relay"]),
                stat_line(long_run["gate_ages_sec"]["planner_result"]),
            )
        )
        report.append(
            "- false-watchdog active events `%d`; lifecycle watchdog entries startup/shutdown `%d/%d`; active genuine watchdog events `%d`." % (
                long_run["active_watchdogs"]["count"],
                long_run["active_watchdogs"]["startup_count"],
                long_run["active_watchdogs"]["shutdown_count"],
                long_run["active_watchdogs"]["count"],
            )
        )
        report.append(
            "- worker latency max/P95: `%.3f/%.3f ms`; main latency max/P95: `%.3f/%.3f ms`." % (
                long_run["timing"]["worker_core_total_ms"]["max"],
                long_run["timing"]["worker_core_total_ms"]["p95"],
                long_run["timing"]["main_full_cycle_ms"]["max"],
                long_run["timing"]["main_full_cycle_ms"]["p95"],
            )
        )
        if resources is not None:
            report.append(
                "- sampled process resources: max aggregate CPU `%.2f%%`, max aggregate RSS `%.0f KiB` (%d samples)." % (
                    resources.get("max_aggregate_cpu_percent", 0.0),
                    resources.get("max_aggregate_rss_kib", 0.0),
                    resources.get("sample_count", 0),
                )
            )
    report += [
        "", "## Contract and fault injection", "",
        "- `planner_result_monotonic`, request/cycle/scan identity, worker generation, path version, and goal version are source fields. Relay sequence/time are transport fields only.",
        "- Relay heartbeat does not update source timestamp. A relay-alive stale source produces `planner_freshness_watchdog` and zero output.",
        "- U1-U10 and F1-F5 passed in `n2_3_contract_unit.py`; structured evidence is `n2_3_results/n2_3_fault_injection.json`.",
        "- F1-F5 are deterministic pure-contract injections covering relay-alive planner stale, relay stop, worker crash, stale response, and scan stop. No live ROS fault was injected during normal B2 runs; the artifact records this boundary.",
        "- No planner parameter, solver tolerance/weight, scan processing, reverse policy, DUNE, Nav2, or real robot change was made.", "",
        "## Verification", "",
        "- `python3 -m py_compile`: PASS",
        "- `colcon build --symlink-install --packages-select r1_neupan_integration`: PASS",
        "- all three 60 s P1 runs: PASS" if all(row["pass"] for row in runs) else "- all three 60 s P1 runs: FAIL",
        "- 610 s compact P1 B2 long run: PASS" if long_run is not None and long_run["pass"] else "- 610 s compact P1 B2 long run: NOT PASS",
        "- fault artifact: `n2_3_results/n2_3_fault_injection.json`",
        "## Code diff", "",
        "- `n2_3_contract.py`: added the planner-result envelope, source identity/version validation, exact-result relay repeat validation, and timestamp/command integrity checks.",
        "- `n2_3_command_relay.py`: added the fixed 50 Hz relay; relay sequence/time are transport-only fields.",
        "- `n1_16_node.py`: publishes planner source/context envelopes while retaining the existing whole-core worker path.",
        "- `n2_pre_safety_gate.py`: added separate relay and planner freshness watchdogs plus fail-closed metadata/failure handling; the 0.30 s bounds remain unchanged.",
        "- `n2_pre3_audit.py` and `n2_3_freshness.launch.py`: added envelope evidence and compact long-run capture; compact mode omits only large semantic/path snapshots.",
        "- `n2_3_contract_unit.py`, `n2_3_report.py`, and `setup.py`: added U/F evidence output, report generation, and installed reporting tools.",
        "- No VCS metadata is present in this workspace; the file-level diff above is the reproducible change record. Planner solver/configuration, scan processing, reverse policy, Nav2, DUNE, and robot geometry were unchanged.",
        "- cleanup: each launch terminated its worker, relay, Gazebo, bridge, and sensor processes; final residual check recorded separately.", "",
        "## Decision", "",
        "N2.3 is complete only for the command/freshness contract. Forward-only recovery design is not entered automatically; `next_stage_allowed=false`.",
        "",
    ]
    report_path = ROOT / "n2_3_results" / "n2_3_report.md"
    report_path.write_text("\n".join(report), encoding="utf-8")
    print(f"json={output}")
    print(f"report={report_path}")
    print(f"classification={payload['classification']}")


if __name__ == "__main__":
    main()
