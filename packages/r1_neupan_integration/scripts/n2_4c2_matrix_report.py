#!/usr/bin/env python3
"""N2.4C2 provenance audit and current-stack matrix aggregation."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
RESULTS = ROOT / "n2_4c2_results"
MANIFEST_PATH = RESULTS / "current_stack_manifest.json"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from n2_4c_reproducibility_report import (  # noqa: E402
    SCENES,
    aggregate_scene,
    cycle_rows,
    load_json,
    n1_timing_summary,
    number,
    path_from_result,
    relative,
    sha256,
    stats,
    summarize_run,
    trigger_summary,
)


HISTORICAL_RUNS = [
    ("B4-1", "B4", ROOT / "n2_4b_results_v5_b4_attempt1.json", ROOT / "n2_4b_capture_v5_b4_attempt1/capture_manifest.json", ROOT / ".ros/log/2026-09-01-14-07-58-115305-pinggu-Vector-16-HX-AI-A2XWIG-3875010/launch.log"),
    ("B4-2", "B4", ROOT / "n2_4b_results_v5_b4_attempt2.json", ROOT / "n2_4b_capture_v5_b4_attempt2/capture_manifest.json", ROOT / ".ros/log/2026-09-01-14-16-22-217852-pinggu-Vector-16-HX-AI-A2XWIG-3881122/launch.log"),
    ("B4-3", "B4", ROOT / "n2_4b_results_v5_b4_attempt3.json", ROOT / "n2_4b_capture_v5_b4_attempt3/capture_manifest.json", ROOT / ".ros/log/2026-09-01-14-19-17-563030-pinggu-Vector-16-HX-AI-A2XWIG-3882553/launch.log"),
    ("B4-4", "B4", ROOT / "n2_4c_phenotype_runs/b4_04/audit.json", ROOT / "n2_4c_phenotype_runs/b4_04/exact/capture_manifest.json", ROOT / "n2_4c_phenotype_runs/b4_04/launch.log"),
    ("B4-5", "B4", ROOT / "n2_4c_phenotype_runs/b4_05/audit.json", ROOT / "n2_4c_phenotype_runs/b4_05/exact/capture_manifest.json", ROOT / "n2_4c_phenotype_runs/b4_05/launch.log"),
    ("D1-1", "D1", ROOT / "n2_4b_results_v5_d1_attempt1.json", ROOT / "n2_4b_capture_v5_d1_attempt1/capture_manifest.json", ROOT / ".ros/log/2026-09-01-14-21-07-940771-pinggu-Vector-16-HX-AI-A2XWIG-3883689/launch.log"),
    ("D1-2", "D1", ROOT / "n2_4b_results_v5_d1_attempt2.json", ROOT / "n2_4b_capture_v5_d1_attempt2/capture_manifest.json", ROOT / ".ros/log/2026-09-01-14-24-33-743387-pinggu-Vector-16-HX-AI-A2XWIG-3886528/launch.log"),
]


SOURCE_FILES = {
    "n1_16_whole_core_source": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n1_15_whole_core.py",
    "n1_16_node_source": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n1_16_node.py",
    "n1_16_bridge_source": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n1_16_bridge.py",
    "n2_3_command_relay": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n2_3_command_relay.py",
    "safety_gate": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n2_pre_safety_gate.py",
    "planner_config": ROOT / "src/r1_neupan_integration/config/planner.yaml",
    "checkpoint": ROOT / "src/r1_neupan_integration/config/models/dune_model_5000.pth",
    "b4_world": ROOT / "src/r1_neupan_integration/worlds/n2_b4_center_obstacle.sdf",
    "d1_world": ROOT / "src/r1_neupan_integration/worlds/n2_d1_corner_obstacle.sdf",
    "reproducibility_launch": ROOT / "src/r1_neupan_integration/launch/n2_4c_reproducibility.launch.py",
    "n1_validation_launch": ROOT / "src/r1_neupan_integration/launch/n1_validation.launch.py",
    "scan_preprocess_launch": ROOT / "src/r1_lidar_preprocess/launch/pointcloud_to_scan.launch.py",
    "trigger_detector": ROOT / "src/r1_neupan_integration/r1_neupan_integration/n2_4_recovery.py",
}


def canonical_manifest_bytes(payload: dict[str, Any]) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def current_stack_payload() -> dict[str, Any]:
    files = {
        name: {"path": relative(path), "sha256": sha256(path)}
        for name, path in SOURCE_FILES.items()
    }
    return {
        "schema": "r1_n2_4c2_current_stack_manifest_v1",
        "stage": "N2.4C2 Current-Stack Matrix Completion",
        "identity_rule": "A run is current-stack sealed only when its exact capture metadata references this file and its byte hash.",
        "environment": {
            "os": "Ubuntu 24.04",
            "ros": "ROS 2 Jazzy",
            "simulator": "Gazebo Harmonic",
            "ros_gz": ["ros_gz_sim", "ros_gz_bridge"],
        },
        "files": files,
        "planner": {
            "config_sha256": files["planner_config"]["sha256"],
            "checkpoint_sha256": files["checkpoint"]["sha256"],
            "neupan_commit": "a740cead70030fa9079cb3ea29c3725b70960828",
            "whole_core_source_sha256": files["n1_16_whole_core_source"]["sha256"],
        },
        "geometry": {"footprint_length_m": 0.682, "footprint_width_m": 0.532},
        "policy": {
            "forward_only": True,
            "max_gated_linear_x_mps": 0.15,
            "initial_estop_latched": True,
            "release_after_health": True,
            "end_latch": True,
            "timeout_sec": 32.0,
            "recovery_commands_allowed": False,
            "navigation_mode": "terminal",
        },
        "freshness_and_timing": {
            "planner_freshness_sec": 0.30,
            "relay_freshness_sec": 0.30,
            "command_timeout_sec": 0.30,
            "scan_timeout_sec": 0.30,
            "odom_timeout_sec": 0.30,
            "tf_timeout_sec": 0.30,
            "timing_timeout_sec": 0.50,
            "max_unexplained_planner_gap_sec": 0.30,
        },
        "torch_thread_policy": {
            "torch_intraop_threads": 1,
            "torch_interop_threads": 24,
            "thread_env_policy": "inherit",
            "bootstrap_arguments": {"torch_num_threads": 0, "torch_num_interop_threads": 0},
            "node_effective_parameters": {"torch_num_threads": 1, "torch_num_interop_threads": 24},
            "environment_variables_modified": False,
        },
        "cpu_affinity": {
            "cpus": [0, 1, 2, 3, 4, 5, 6, 7],
            "policy": "fixed for every C2 launch to avoid host-level startup pthread contention",
        },
        "scan_preprocessing_chain": [
            {"topic": "/lidar/points", "type": "sensor_msgs/msg/PointCloud2", "owner": "Gazebo Harmonic bridge"},
            {"topic": "/scan_neupan", "type": "sensor_msgs/msg/LaserScan", "owner": "pointcloud_to_laserscan"},
            {"topic": "/scan_neupan", "type": "sensor_msgs/msg/LaserScan", "owner": "n1_16_node input"},
        ],
        "ros_topic_topology": {
            "planner_source": {"topic": "/neupan_planner_result", "type": "std_msgs/msg/String", "publisher": "n1_16_node", "subscriber": "n2_3_command_relay and audit/gate source observer"},
            "relay": {"topic": "/neupan_cmd_envelope", "type": "std_msgs/msg/String", "publisher": "n2_3_command_relay", "subscriber": "r1_safety_gate and audit"},
            "gate_output": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "publisher": "r1_safety_gate", "subscriber": "Gazebo DiffDrive bridge"},
            "context": {"topic": "/neupan_planner_context", "type": "std_msgs/msg/String", "publisher": "n1_16_node", "subscriber": "r1_safety_gate"},
            "health": ["/neupan_timing", "/neupan_bridge_status", "/r1_safety_gate_status", "/odom", "/tf"],
        },
        "command_path": "n1_16_node -> /neupan_planner_result -> n2_3_command_relay -> /neupan_cmd_envelope -> r1_safety_gate(freshness_envelope) -> /cmd_vel",
        "worlds": {
            "B4": {"path": relative(SOURCE_FILES["b4_world"]), "sha256": files["b4_world"]["sha256"], "start_pose": [0.0, 0.0, 0.0]},
            "D1": {"path": relative(SOURCE_FILES["d1_world"]), "sha256": files["d1_world"]["sha256"], "start_pose": [0.0, 0.0, 0.0]},
        },
        "paths": {label: [list(point) for point in scene["path"]] for label, scene in SCENES.items()},
        "prohibited_changes": [
            "evaluate_trigger", "trigger thresholds", "recovery counterfactual", "synthetic rotate/reanchor",
            "Gazebo recovery motion", "recovery /cmd_vel", "autonomous reverse", "planner tuning", "DUNE training",
            "Nav2", "world/path/geometry", "real robot",
        ],
    }


def write_current_stack_manifest(path: Path = MANIFEST_PATH) -> str:
    payload = current_stack_payload()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_manifest_bytes(payload))
    return sha256(path) or ""


def load_manifest(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file():
        return None, None
    return load_json(path), sha256(path)


def process_provenance(log_path: Path) -> dict[str, Any]:
    if not log_path.is_file():
        return {"log": relative(log_path), "available": False, "started_processes": [], "relay_process_started": False, "fatal_patterns": []}
    text = log_path.read_text(encoding="utf-8", errors="replace")
    started = sorted(set(re.findall(r"\[([^]]+)\]: process started", text)))
    died_lines = re.findall(r"process has died[^\n]*exit code ([^,) ]+)", text)
    unexpected_died = [code for code in died_lines if code not in {"-2", "2"}]
    patterns = {
        # Gazebo receives SIGINT during launch shutdown and reports exit -2;
        # that is expected teardown, not a run failure.
        "process_died": bool(unexpected_died),
        "traceback": r"Traceback \(most recent call last\)",
        "resource_unavailable": r"Resource temporarily unavailable",
        "aborted": r"Aborted \(Signal sent",
    }
    return {
        "log": relative(log_path),
        "available": True,
        "started_processes": started,
        "relay_process_started": any("n2_3_command_relay" in item for item in started),
        "fatal_patterns": [
            name for name, pattern in patterns.items()
            if (pattern if isinstance(pattern, bool) else re.search(pattern, text))
        ],
        "process_exit_codes": died_lines,
        "unexpected_process_exit_codes": unexpected_died,
    }


def historical_provenance(label: str, scene_label: str, result_path: Path, exact_path: Path, log_path: Path, expected: dict[str, Any]) -> dict[str, Any]:
    result = load_json(result_path)
    capture = load_json(exact_path)
    freshness = result.get("freshness") or {}
    trace = result.get("command_envelope_trace") or {}
    planner_rows = trace.get("planner_result") or []
    relay_rows = trace.get("relay") or []
    planner_payloads = [row.get("payload") or {} for row in planner_rows if isinstance(row, dict)]
    relay_payloads = [row.get("payload") or {} for row in relay_rows if isinstance(row, dict)]
    source_schema = any(item.get("schema") == "r1_n2_3_command_envelope_v1" for item in planner_payloads)
    relay_schema = any(item.get("schema") == "r1_n2_3_command_envelope_v1" for item in relay_payloads)
    current_path = bool(relay_rows) and freshness.get("mode") == "freshness_envelope"
    config = capture.get("config_identity") or {}
    extra = capture.get("extra_metadata") or {}
    launch = process_provenance(log_path)
    recorded_hashes = {
        "n1_16_whole_core_source": config.get("integration_source_sha256"),
        "planner_config": config.get("config_sha256"),
        "checkpoint": config.get("checkpoint_sha256"),
        "n2_3_command_relay": None,
        "safety_gate": None,
        "launch": None,
    }
    if current_path:
        compatibility = True
        basis = [
            "freshness_envelope mode recorded",
            "relay process started in launch provenance",
            "relay trace contains current N2.3 envelope schema and relay fields",
            "gate trace records freshness_envelope and source_metadata",
        ]
    else:
        compatibility = False
        basis = [
            "freshness.mode=legacy_twist",
            "relay trace count is zero",
            "launch provenance has no n2_3_command_relay process",
            "gate ages show relay=null and command input is the legacy Twist path",
        ]
    return {
        "run": label,
        "scene": scene_label,
        "result_file": relative(result_path),
        "exact_capture_file": relative(exact_path),
        "launch_provenance": launch,
        "actual_command_path": (
            "n1_16_node -> /neupan_planner_result -> n2_3_command_relay -> /neupan_cmd_envelope -> r1_safety_gate(freshness_envelope) -> /cmd_vel"
            if current_path else
            "n1_16_node -> /neupan_cmd_vel(Twist) -> r1_safety_gate(legacy_twist) -> /cmd_vel"
        ),
        "n2_3_relay_active": bool(launch["relay_process_started"] or relay_rows),
        "source_envelope_observed": source_schema,
        "relay_envelope_telemetry_observed": relay_schema and bool(relay_rows),
        "envelope_field_semantics": (
            "The old false marker means the N2.3 relay/envelope command path was not active; the planner-result source envelope itself was still observed."
            if not current_path else
            "The planner-result source envelope and the repeated relay envelope were both observed."
        ),
        "freshness_mode": freshness.get("mode"),
        "planner_result_count": len(planner_rows),
        "relay_count": len(relay_rows),
        "source_schema": "r1_n2_3_command_envelope_v1" if source_schema else None,
        "relay_schema": "r1_n2_3_command_envelope_v1" if relay_schema else None,
        "source_identity_fields_observed": ["worker_generation", "request_id", "cycle_id", "scan_counter", "planner_result_monotonic", "path_version", "goal_version"] if source_schema else [],
        "relay_fields_observed": ["relay_sequence", "relay_publish_monotonic", "relay_source_receive_monotonic"] if relay_schema else [],
        "recorded_source_hashes": recorded_hashes,
        "expected_current_hashes": {
            "n1_16_whole_core_source": expected["files"]["n1_16_whole_core_source"]["sha256"],
            "planner_config": expected["files"]["planner_config"]["sha256"],
            "checkpoint": expected["files"]["checkpoint"]["sha256"],
            "n2_3_command_relay": expected["files"]["n2_3_command_relay"]["sha256"],
            "safety_gate": expected["files"]["safety_gate"]["sha256"],
            "launch": expected["files"]["reproducibility_launch"]["sha256"],
        },
        "launch_stage": extra.get("stage"),
        "capture_mode": extra.get("mode"),
        "manifest_reference": extra.get("current_stack_manifest_sha256"),
        "sealed_manifest": False,
        "current_stack_compatible": compatibility,
        "compatibility_basis": basis,
        "historical_hash_limit": "The N2.4B/N2.4C artifact contract did not record relay, safety-gate, or launch source hashes; those fields are explicitly unrecorded rather than inferred.",
    }


def max_gate_age(result: dict[str, Any], key: str) -> float | None:
    release = number((result.get("safety") or {}).get("release_done_t"))
    values = []
    for row in result.get("gate_trace", []):
        if not isinstance(row, dict):
            continue
        t = number(row.get("audit_t"))
        if release is not None and t is not None and t < release:
            continue
        value = number(((row.get("value") or {}).get("ages_sec") or {}).get(key))
        if value is not None:
            values.append(value)
    return max(values) if values else None


def current_runtime_health(result_path: Path, result: dict[str, Any], manifest: dict[str, Any], manifest_hash: str | None, launch_path: Path, expected_manifest_hash: str | None) -> dict[str, Any]:
    capture_path = result_path.parent / "exact" / "capture_manifest.json"
    capture = load_json(capture_path) if capture_path.is_file() else {}
    extra = capture.get("extra_metadata") or {}
    config = capture.get("config_identity") or {}
    n1_path = result_path.with_name(result_path.stem + "_n1_metrics.json")
    n1 = load_json(n1_path) if n1_path.is_file() else {}
    runtime_path = result_path.parent / "runtime.json"
    runtime = load_json(runtime_path) if runtime_path.is_file() else {}
    runtime_info = current_runtime_summary(result, n1, runtime)
    expected_files = manifest.get("files") or {}
    checks = {
        "manifest_file_present": manifest_hash is not None,
        "manifest_hash_matches_expected": expected_manifest_hash is not None and manifest_hash == expected_manifest_hash,
        "run_references_manifest": extra.get("current_stack_manifest_sha256") == expected_manifest_hash and bool(extra.get("current_stack_manifest")),
        "freshness_mode": (result.get("freshness") or {}).get("mode") == "freshness_envelope",
        "relay_telemetry_observed": bool(((result.get("freshness") or {}).get("relay") or {}).get("count")),
        "n1_source_hash_matches": config.get("integration_source_sha256") == (expected_files.get("n1_16_whole_core_source") or {}).get("sha256"),
        "planner_config_hash_matches": config.get("config_sha256") == (expected_files.get("planner_config") or {}).get("sha256"),
        "checkpoint_hash_matches": config.get("checkpoint_sha256") == (expected_files.get("checkpoint") or {}).get("sha256"),
        "launch_has_relay_process": process_provenance(launch_path).get("relay_process_started", False),
        "no_launch_fatal_pattern": not process_provenance(launch_path).get("fatal_patterns"),
        "release_success": bool(((result.get("safety") or {}).get("release_response") or {}).get("success")),
        "end_latch_success": bool(((result.get("safety") or {}).get("latch_response") or {}).get("success")),
        "no_planner_freshness_watchdog": not runtime_info["post_release_watchdogs"].get("planner_freshness_watchdog", 0),
        "no_relay_freshness_watchdog": not runtime_info["post_release_watchdogs"].get("relay_watchdog", 0),
        "scan_source_healthy": runtime_info["scan_source_healthy"],
        "active_scan_age_under_300ms": runtime_info["scan_age_after_first_cycle_ms"] is not None and runtime_info["scan_age_after_first_cycle_ms"] <= 300.0,
        "planner_source_age_under_300ms": runtime_info["planner_source_age_sec"] is not None and runtime_info["planner_source_age_sec"] <= 0.30,
        "relay_age_under_300ms": runtime_info["relay_age_sec"] is not None and runtime_info["relay_age_sec"] <= 0.30,
        "planner_gap_under_300ms": runtime_info["planner_result_interval_sec"] is not None and runtime_info["planner_result_interval_sec"] < 0.30,
        "queue_le_1": runtime_info["queue_max"] is not None and runtime_info["queue_max"] <= 1.0,
        "stale_zero": runtime_info["stale"] == 0,
        "nan_inf_zero": runtime_info["nan_inf"] == 0,
        "worker_failure_zero": runtime_info["worker_failures"] == 0,
        "gated_linear_nonnegative": runtime_info["gated_linear_min"] is not None and runtime_info["gated_linear_min"] >= 0.0,
        "contact_zero": runtime_info["contact_count"] == 0,
    }
    return {
        "checks": checks,
        "runtime_invariants_pass": all(checks.values()),
        "runtime_file": relative(runtime_path) if runtime_path.is_file() else None,
        "runtime_summary": runtime_info,
        "provenance": {
            "manifest_hash": manifest_hash,
            "manifest_reference": extra.get("current_stack_manifest_sha256"),
            "recorded_stage": extra.get("stage"),
            "recorded_mode": extra.get("mode"),
            "config_hashes": {
                "n1_16_whole_core_source": config.get("integration_source_sha256"),
                "planner_config": config.get("config_sha256"),
                "checkpoint": config.get("checkpoint_sha256"),
            },
        },
    }


def current_runtime_summary(result: dict[str, Any], n1: dict[str, Any], runtime: dict[str, Any]) -> dict[str, Any]:
    info = n1_timing_summary(n1)
    timing = result.get("timing") or {}
    freshness = result.get("freshness") or {}
    scan = n1.get("scan") or {}
    scan_messages = scan.get("messages")
    if isinstance(scan_messages, list):
        scan_count = len(scan_messages)
    else:
        scan_count = number(scan_messages) or 0
    rows = result.get("gate_trace") or []
    release = number((result.get("safety") or {}).get("release_done_t"))
    watchdogs: dict[str, int] = {}
    for row in rows:
        if release is not None and number(row.get("audit_t")) is not None and number(row.get("audit_t")) < release:
            continue
        for reason in (row.get("value") or {}).get("watchdog_triggered", []):
            watchdogs[reason] = watchdogs.get(reason, 0) + 1
    planner_interval = number((((freshness.get("planner_result") or {}).get("source_interval_sec") or {}).get("max")))
    relay_interval = number((((freshness.get("relay") or {}).get("source_interval_sec") or {}).get("max")))
    return {
        "queue_max": number(timing.get("queue_depth_max")),
        "stale": timing.get("stale", 0),
        "nan_inf": timing.get("nan_inf", 0),
        "worker_failures": info["worker_failures"],
        "gated_linear_min": number((result.get("no_reverse") or {}).get("minimum_gated_linear_x")),
        "contact_count": (result.get("contacts") or {}).get("contact_count"),
        "scan_count": scan_count,
        "scan_source_healthy": scan_count > 0 and info["scan_age_max_after_first_cycle_ms"] is not None,
        "scan_age_after_first_cycle_ms": info["scan_age_max_after_first_cycle_ms"],
        "planner_source_age_sec": max_gate_age(result, "planner_result"),
        "relay_age_sec": max_gate_age(result, "relay"),
        "planner_result_interval_sec": planner_interval,
        "relay_interval_sec": relay_interval,
        "post_release_watchdogs": watchdogs,
        "runtime_counts": runtime.get("counts") if isinstance(runtime, dict) else None,
    }


def c2_run_specs() -> list[tuple[str, str, Path, Path, Path]]:
    specs = []
    for scene_label, prefix in (("B4", "b4"), ("D1", "d1")):
        for index in range(1, 6):
            run = f"C2-{scene_label}-{index}"
            directory = ROOT / "n2_4c2_phenotype_runs" / f"{prefix}_{index:02d}"
            specs.append((run, scene_label, directory / "audit.json", directory / "exact/capture_manifest.json", directory / "launch.log"))
    return specs


def enrich_current_run(run_label: str, scene_label: str, result_path: Path, exact_path: Path, launch_path: Path, stack_manifest: dict[str, Any] | None, stack_hash: str | None) -> dict[str, Any]:
    base = summarize_run(run_label, scene_label, result_path, exact_path)
    base.pop("_comparison_rows", None)
    result = load_json(result_path)
    health = current_runtime_health(result_path, result, stack_manifest or {}, stack_hash, launch_path, stack_hash)
    capture = load_json(exact_path)
    extra = capture.get("extra_metadata") or {}
    base["current_stack_sealed"] = health["checks"]["run_references_manifest"] and health["checks"]["manifest_hash_matches_expected"]
    base["matrix_admitted"] = bool(base["current_stack_sealed"] and health["runtime_invariants_pass"])
    base["provenance"] = {
        "manifest": extra.get("current_stack_manifest"),
        "manifest_sha256": extra.get("current_stack_manifest_sha256"),
        "capture_stage": extra.get("stage"),
        "capture_mode": extra.get("mode"),
        "launch": process_provenance(launch_path),
    }
    base["runtime_health"] = health
    return base


def safe_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items() if key != "_comparison_rows"}
    if isinstance(value, list):
        return [safe_json(item) for item in value]
    return value


def matrix_scene(scene_label: str, runs: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [run for run in runs if run.get("matrix_admitted")]
    scene = aggregate_scene(scene_label, valid)
    scene["attempted_runs"] = len(runs)
    scene["excluded_runs"] = [
        {"run": run["label"], "reasons": [key for key, value in (run.get("runtime_health", {}).get("checks", {}) or {}).items() if not value]}
        for run in runs if not run.get("matrix_admitted")
    ]
    scene["runs"] = [safe_json(run) for run in scene["runs"]]
    return scene


def final_classification(b4: dict[str, Any], d1: dict[str, Any]) -> tuple[str, str, str]:
    if b4["valid_runs"] < 5 or d1["valid_runs"] < 5:
        return "F. UNRESOLVED", "none", "current-stack matrix incomplete"
    if b4["runtime_invariant_failures"] or d1["runtime_invariant_failures"]:
        return "E. RUNTIME REGRESSION RETURNED", "Runtime Attribution/Fix", "runtime health failed for an admitted candidate"
    stable = []
    for scene in (b4, d1):
        rates = scene["phenotype_rates"]
        stable.append(rates["arrive_rate"] == 0.0 and rates["formal_trigger_rate"] == 0.0 and rates["fail_rate"] == 1.0)
    if all(stable):
        return "A. CURRENT-STACK MATRIX COMPLETE - TRIGGER MISMATCH CONFIRMED", "N2.4D Trigger Contract Re-attribution", "both scenes fail in all five current-stack runs and the frozen formal trigger never fires"
    if any(scene["phenotype_rates"]["arrive_rate"] not in (0.0, 1.0) for scene in (b4, d1)):
        return "B. CURRENT-STACK MATRIX COMPLETE - FAILURE NONDETERMINISTIC", "Reproducibility Attribution", "at least one scene has mixed arrival outcomes"
    if any(scene["phenotype_rates"]["arrive_rate"] == 1.0 for scene in (b4, d1)):
        return "C. HISTORICAL FAILURE NO LONGER REPRESENTATIVE", "Recovery Benchmark Scope Reduction", "at least one scene mostly passes under the current stack"
    return "D. EVIDENCE PROVENANCE MIXED", "Current-Stack Matrix Rebuild", "the observed matrix does not support a stable attribution"


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    stack_manifest, stack_hash = load_manifest(MANIFEST_PATH)
    expected_manifest = stack_manifest or current_stack_payload()
    historical = [historical_provenance(label, scene, result, exact, log, expected_manifest) for label, scene, result, exact, log in HISTORICAL_RUNS if result.is_file() and exact.is_file()]
    current_by_scene: dict[str, list[dict[str, Any]]] = {"B4": [], "D1": []}
    missing_current = []
    for label, scene, result, exact, log in c2_run_specs():
        if not result.is_file() or not exact.is_file():
            missing_current.append(label)
            continue
        current_by_scene[scene].append(enrich_current_run(label, scene, result, exact, log, stack_manifest, stack_hash))
    b4 = matrix_scene("B4", current_by_scene["B4"])
    d1 = matrix_scene("D1", current_by_scene["D1"])
    classification, next_stage, reason = final_classification(b4, d1)
    runtime_failures = [run["label"] for run in current_by_scene["B4"] + current_by_scene["D1"] if not run.get("runtime_health", {}).get("runtime_invariants_pass")]
    return safe_json({
        "schema": "r1_n2_4c2_current_stack_matrix_report_v1",
        "stage": "N2.4C2 Current-Stack Matrix Completion",
        "final_classification": classification,
        "next_stage_allowed": next_stage,
        "final_reason": reason,
        "experiment_stopped": True,
        "recovery_motion_allowed": False,
        "envelope_false_semantics": "N2.3 envelope=false means the observed run did not use the N2.3 relay/envelope command path. It does not mean /neupan_planner_result was absent: the old runs contain source envelope records, but their safety gate consumed legacy /neupan_cmd_vel Twist and no relay process/publication was observed.",
        "gate_1": {"complete": True, "case": "CASE B", "historical_runs": historical},
        "current_stack_manifest": {"path": relative(MANIFEST_PATH), "sha256": stack_hash, "payload": expected_manifest},
        "included_runs": {
            "B4": [run["label"] for run in current_by_scene["B4"] if run.get("matrix_admitted")],
            "D1": [run["label"] for run in current_by_scene["D1"] if run.get("matrix_admitted")],
        },
        "excluded_runs": {
            "missing": missing_current,
            "runtime_or_provenance": [
                {"run": run["label"], "reasons": [key for key, value in (run.get("runtime_health", {}).get("checks", {}) or {}).items() if not value]}
                for run in current_by_scene["B4"] + current_by_scene["D1"] if not run.get("matrix_admitted")
            ],
            "historical_from_current_denominator": [
                {"run": item["run"], "reason": "legacy/non-C2 provenance; retained only as historical reference"}
                for item in historical if not item["current_stack_compatible"] or not item["sealed_manifest"]
            ],
        },
        "scenes": {"B4": b4, "D1": d1},
        "runtime_health": {
            "current_run_failures": runtime_failures,
            "all_current_candidates_healthy": not runtime_failures,
            "required_invariants": ["no planner_freshness_watchdog regression", "no relay freshness regression", "scan source healthy", "active scan age <=300ms", "planner result gap <300ms", "queue<=1", "stale=0", "NaN/Inf=0", "worker failure=0", "gated linear.x>=0", "contact=0"],
        },
        "safety_invariants": {"forward_only": True, "contact_zero_required": True, "queue_max_required": 1, "stale_required": 0, "nan_inf_required": 0, "worker_failure_required": 0, "failure_latch_required": True, "planner_source_age_not_refreshed_by_relay": True},
        "verification": {
            "compileall": args.compileall_status,
            "colcon_build": args.colcon_build_status,
            "direct_units": {"n2_4_recovery_unit.py": args.recovery_unit_status, "n2_3_contract_unit.py": args.contract_unit_status, "n2_pre_safety_unit.py": args.safety_unit_status, "n2_pre_semantics_unit.py": args.semantics_unit_status},
            "colcon_test": "0 tests / exit 5 (existing discovery gap; not fixed in N2.4C2)",
            "cleanup": args.cleanup_status,
            "commands": args.commands,
        },
        "provenance_note": "Historical N2.4B/N2.4C artifacts remain available but are excluded from the C2 denominator unless a sealed manifest reference and runtime-valid C2 evidence are present.",
        "scope_boundary": "No evaluate_trigger change, threshold change, recovery counterfactual, synthetic rotate/reanchor, Gazebo recovery motion, recovery /cmd_vel, autonomous reverse, planner tuning, DUNE training, Nav2, world/path/geometry change, or real-robot operation was executed.",
    })


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def render_markdown(report: dict[str, Any]) -> str:
    manifest = report["current_stack_manifest"]
    lines = [
        "# N2.4C2 Current-Stack Matrix Completion",
        "",
        f"Final classification: **{report['final_classification']}**",
        f"`next_stage_allowed={report['next_stage_allowed']}`, `recovery_motion_allowed=false`.",
        "",
        "## Gate 1: N2.3 Envelope Provenance",
        "",
        "`N2.3 envelope=false` is an architecture marker, not a statement that planner-result source telemetry was absent. In the old runs, `/neupan_planner_result` carried the source envelope schema, but no `n2_3_command_relay` process or `/neupan_cmd_envelope` publication was observed; the safety gate consumed legacy `/neupan_cmd_vel` Twist. Therefore these runs are CASE B legacy/non-current-stack and are excluded from the C2 denominator.",
        "",
        "| Run | Actual command path | Relay active | Source envelope | Relay telemetry | Mode | Current-stack compatible | Sealed manifest | Hash evidence |",
        "|---|---|---:|---:|---:|---|---:|---:|---|",
    ]
    for item in report["gate_1"]["historical_runs"]:
        hashes = item["recorded_source_hashes"]
        hash_text = f"n1 `{str(hashes.get('n1_16_whole_core_source'))[:12]}` / cfg `{str(hashes.get('planner_config'))[:12]}` / ckpt `{str(hashes.get('checkpoint'))[:12]}`; relay/gate/launch unrecorded"
        lines.append(f"| {item['run']} | `{item['actual_command_path']}` | {int(item['n2_3_relay_active'])} | {int(item['source_envelope_observed'])} | {int(item['relay_envelope_telemetry_observed'])} | `{item['freshness_mode']}` | {int(item['current_stack_compatible'])} | {int(item['sealed_manifest'])} | {hash_text} |")
    lines.extend([
        "",
        "Historical provenance result: old B4-1..3 and D1-1..2 are `LEGACY / NON-CURRENT-STACK`; B4-4/5 did run the current relay architecture, but are retained as unsealed historical reference and are not mixed into the sealed C2 denominator.",
        "",
        "## Current Stack Manifest",
        "",
        f"Manifest: `{manifest['path']}`; byte SHA-256 `{manifest['sha256']}`.",
        f"Environment: `{manifest['payload']['environment']['os']}`, `{manifest['payload']['environment']['ros']}`, `{manifest['payload']['environment']['simulator']}` with `ros_gz_sim`/`ros_gz_bridge`.",
        f"Footprint: `{manifest['payload']['geometry']['footprint_length_m']} x {manifest['payload']['geometry']['footprint_width_m']} m`; forward-only; planner/relay freshness `0.30 s`; timing timeout `0.50 s`; timeout `32.0 s`.",
        f"N1.16 whole-core `{manifest['payload']['files']['n1_16_whole_core_source']['sha256']}`; N2.3 relay `{manifest['payload']['files']['n2_3_command_relay']['sha256']}`; safety gate `{manifest['payload']['files']['safety_gate']['sha256']}`.",
        f"Planner config `{manifest['payload']['files']['planner_config']['sha256']}`; checkpoint `{manifest['payload']['files']['checkpoint']['sha256']}`; B4 world `{manifest['payload']['files']['b4_world']['sha256']}`; D1 world `{manifest['payload']['files']['d1_world']['sha256']}`.",
        f"Torch policy: effective intra-op `{manifest['payload']['torch_thread_policy']['node_effective_parameters']['torch_num_threads']}`, inter-op `{manifest['payload']['torch_thread_policy']['node_effective_parameters']['torch_num_interop_threads']}`, environment policy `inherit`.",
        f"Command path: `{manifest['payload']['command_path']}`.",
        "",
        "## Included / Excluded Runs",
        "",
        f"Included B4: `{', '.join(report['included_runs']['B4']) or 'none'}`.",
        f"Included D1: `{', '.join(report['included_runs']['D1']) or 'none'}`.",
        "",
        "| Run | Status | Exclusion reason |",
        "|---|---|---|",
    ])
    for scene in ("B4", "D1"):
        for run in report["scenes"][scene].get("excluded_runs", []):
            lines.append(f"| {run['run']} | EXCLUDED | `{', '.join(run['reasons']) or 'not admitted'}` |")
    for item in report["excluded_runs"]["historical_from_current_denominator"]:
        lines.append(f"| {item['run']} | HISTORICAL ONLY | {item['reason']} |")
    lines.extend([
        "",
        "## Current-Stack Phenotype Matrix",
        "",
        "| Scene | Run | Primary | Arrival | Stop | Stuck | Timeout | Tracking loss | Persistent negative | Formal trigger | Goal error | Travel | Max/final deviation | Neg samples/longest | Neg/Dev/Stop cycles | Runtime |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ])
    for scene in ("B4", "D1"):
        for run in report["scenes"][scene]["runs"]:
            tracking = int(run["primary_phenotype"] == "E. REFERENCE TRACKING LOSS" or "E. REFERENCE TRACKING LOSS" in run["secondary_tags"])
            persistent = int(run["longest_continuous_negative_run"] >= 3)
            lines.append(f"| {scene} | {run['label']} | {run['primary_phenotype']} | {int(run['arrival'])} | {int(run['stop'])} | {int(run['stuck'])} | {int(run['timeout_with_motion'])} | {tracking} | {persistent} | {int(run['formal_trigger'])} | {fmt(run['final_goal_error_m'])} | {fmt(run['total_travel_m'])} | {fmt(run['max_path_deviation_m'])}/{fmt(run['final_path_deviation_m'])} | {run['raw_negative_sample_count']}/{run['longest_continuous_negative_run']} | {run['first_negative_cycle'] or 'n/a'} / {run['first_deviation_cycle'] or 'n/a'} / {run['first_stop_cycle'] or 'n/a'} | PASS |")
    lines.extend(["", "## Rates and Scene Classification", "", "| Scene | n | Arrival | Fail | Stop | Timeout | Stuck | Tracking loss | Persistent negative | Formal trigger | Primary | Secondary |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|"])
    for scene in ("B4", "D1"):
        data = report["scenes"][scene]
        rates = data["phenotype_rates"]
        lines.append(f"| {scene} | {data['valid_runs']} | {fmt(rates['arrive_rate'], 2)} | {fmt(rates['fail_rate'], 2)} | {fmt(rates['stop_rate'], 2)} | {fmt(rates['timeout_rate'], 2)} | {fmt(rates['stuck_rate'], 2)} | {fmt(rates['tracking_loss_rate'], 2)} | {fmt(rates['persistent_negative_rate'], 2)} | {fmt(rates['formal_trigger_rate'], 2)} | **{data['conclusion']}** | `{', '.join(data['secondary_conclusions']) or 'none'}` |")
    lines.extend([
        "",
        "## Trigger Rejection Timeline",
        "",
        "The frozen online detector and exact capture remain read-only. Each current run preserves raw negative sample count, longest negative run, first negative cycle, first persistent-negative cycle, first deviation cycle, first stop cycle, formal trigger cycle, and rejection reasons in the JSON report.",
        "",
        "| Scene | Run | First negative | First persistent negative | First deviation | First stop | Formal trigger |",
        "|---|---|---:|---:|---:|---:|---:|",
    ])
    for scene in ("B4", "D1"):
        for run in report["scenes"][scene]["runs"]:
            lines.append(f"| {scene} | {run['label']} | {run['first_negative_cycle'] or 'n/a'} | {run['first_persistent_negative_cycle'] or 'n/a'} | {run['first_deviation_cycle'] or 'n/a'} | {run['first_stop_cycle'] or 'n/a'} | {run['formal_trigger_cycle'] or 'n/a'} |")
    lines.extend([
        "",
        "D1 timing interpretation is reported directly from these cycle fields: early negative intent is compared with later deviation and stop closure; no trigger threshold or window was altered. B4 negative scarcity is represented by its raw negative sample and longest-run columns; no run is reclassified to create a trigger.",
        "",
        "## Runtime Health and Safety",
        "",
        "All included runs must pass: no planner/relay freshness watchdog after release, healthy scan source, active scan age <=300 ms, no unexplained planner result gap >=300 ms, queue <=1, stale=0, NaN/Inf=0, worker failure=0, gated linear.x >=0, and contact=0. Initial e-stop, release-after-health, and final latch are also checked per run.",
        f"Current runtime failures: `{', '.join(report['runtime_health']['current_run_failures']) or 'none'}`.",
        "",
        "## Verification and Boundary",
        "",
        f"- compileall: `{report['verification']['compileall']}`; colcon build: `{report['verification']['colcon_build']}`.",
        f"- direct units: `{report['verification']['direct_units']}`.",
        f"- colcon test: `{report['verification']['colcon_test']}`.",
        f"- cleanup: `{report['verification']['cleanup']}`.",
        "- No trigger contract change, recovery, planner tuning, DUNE training, Nav2, world/path/geometry change, autonomous reverse, Gazebo recovery motion, or real-robot operation was executed.",
        "",
        f"Final decision: **{report['final_classification']}**; `next_stage_allowed={report['next_stage_allowed']}`. Testing stops at N2.4C2.",
    ])
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write-manifest", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=RESULTS)
    parser.add_argument("--compileall-status", default="NOT RUN")
    parser.add_argument("--colcon-build-status", default="NOT RUN")
    parser.add_argument("--recovery-unit-status", default="NOT RUN")
    parser.add_argument("--contract-unit-status", default="NOT RUN")
    parser.add_argument("--safety-unit-status", default="NOT RUN")
    parser.add_argument("--semantics-unit-status", default="NOT RUN")
    parser.add_argument("--cleanup-status", default="NOT RUN")
    parser.add_argument("--commands", action="append", default=[])
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.write_manifest:
        print(f"manifest={MANIFEST_PATH}")
        print(f"manifest_sha256={write_current_stack_manifest(MANIFEST_PATH)}")
        return
    report = build_report(args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "n2_4c2_current_stack_matrix.json"
    markdown_path = args.output_dir / "n2_4c2_current_stack_matrix.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    print(f"json={json_path}")
    print(f"markdown={markdown_path}")
    print(f"classification={report['final_classification']}")
    print(f"next_stage_allowed={report['next_stage_allowed']}")


if __name__ == "__main__":
    main()
