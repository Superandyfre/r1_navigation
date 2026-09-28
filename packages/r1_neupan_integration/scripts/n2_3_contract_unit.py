"""Deterministic N2.3 freshness, relay, and fail-safe contract tests."""

from __future__ import annotations

import copy
import argparse
import json
import math
from pathlib import Path

from r1_neupan_integration.n2_3_contract import (
    ContractError,
    FreshnessTracker,
    make_context,
    make_planner_envelope,
    relay_envelope,
    validate_envelope,
)
from r1_neupan_integration.n2_pre_safety_gate import SafetyGateCore


def envelope(*, request_id=1, generation=7, cycle_id=None, scan_counter=None,
             path_version=3, goal_version=4, source_time=99.90, linear=0.10):
    return make_planner_envelope(
        linear_x=linear, angular_z=0.05, worker_generation=generation,
        request_id=request_id, cycle_id=request_id if cycle_id is None else cycle_id,
        scan_counter=request_id if scan_counter is None else scan_counter,
        planner_result_monotonic=source_time, path_version=path_version,
        goal_version=goal_version,
    )


def healthy_step(core, now=100.0, relay_seen=99.99, source_seen=99.90,
                 command_seen=None, **kwargs):
    if command_seen is None:
        command_seen = relay_seen
    sensor_seen = now - 0.01
    return core.step(
        now, 0.10, 0.05, command_seen=command_seen, relay_seen=relay_seen,
        planner_result_seen=source_seen, scan_seen=sensor_seen, odom_seen=sensor_seen,
        tf_seen=sensor_seen, timing_seen=sensor_seen, **kwargs,
    )


def make_core():
    core = SafetyGateCore(command_mode="freshness_envelope")
    core.release()
    return core


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    tracker = FreshnessTracker()
    tracker.set_context(make_context(3, 4))
    first = envelope()
    ok, reason = tracker.accept(first)
    assert ok and reason == "new_source_result"  # U1
    repeated = relay_envelope(first, 1, 99.92)
    ok, reason = tracker.accept(repeated)
    assert ok and reason == "duplicate_source_result"  # U2 identity stable
    assert all(repeated[field] == first[field] for field in (
        "worker_generation", "request_id", "cycle_id", "scan_counter",
        "planner_result_monotonic", "path_version", "goal_version",
    ))
    assert repeated["relay_sequence"] != first["relay_sequence"] or repeated["relay_publish_monotonic"] != first["relay_publish_monotonic"]
    tampered = relay_envelope(first, 2, 99.93)
    tampered["command"]["linear_x"] = 0.11
    ok, reason = tracker.accept(tampered)
    assert not ok and reason == "duplicate_command_mismatch"  # source integrity
    timestamp_changed = relay_envelope(first, 3, 99.94)
    timestamp_changed["planner_result_monotonic"] = 99.91
    ok, reason = tracker.accept(timestamp_changed)
    assert not ok and reason == "source_identity_mismatch"  # source timestamp identity

    core = make_core()
    decision = healthy_step(core, source_seen=99.90)
    assert decision.linear > 0.0 and not decision.zeroed  # U1
    stale = healthy_step(core, now=100.21, relay_seen=100.20, source_seen=99.90)
    assert stale.zeroed and "planner_freshness_watchdog" in stale.reasons  # U3
    gap_tolerated = healthy_step(
        make_core(), now=100.50, relay_seen=100.49, source_seen=100.45,
        command_seen=99.80,
    )
    assert not gap_tolerated.zeroed and "command_watchdog" not in gap_tolerated.reasons  # U4
    relay_stale = healthy_step(make_core(), now=100.40, relay_seen=100.00, source_seen=100.35)
    assert relay_stale.zeroed and "relay_watchdog" in relay_stale.reasons  # U5
    crashed = healthy_step(make_core(), bridge_failure=True, bridge_failure_reason="worker_crash")
    assert crashed.zeroed and "worker_crash" in crashed.reasons  # U6

    mismatch = FreshnessTracker()
    mismatch.accept(first)
    bad_generation = envelope(request_id=2, generation=8, source_time=100.0)
    ok, reason = mismatch.accept(bad_generation)
    assert not ok and reason == "generation_mismatch"  # U7
    generation_gate = healthy_step(make_core(), metadata_fault="generation_mismatch")
    assert generation_gate.zeroed and "generation_mismatch" in generation_gate.reasons

    version = FreshnessTracker()
    version.set_context(make_context(3, 4))
    version.accept(first)
    version.set_context(make_context(4, 4))
    ok, reason = version.accept(first)
    assert not ok and reason == "path_version_mismatch"  # U8
    version_gate = healthy_step(make_core(), metadata_fault="path_version_mismatch")
    assert version_gate.zeroed and "path_version_mismatch" in version_gate.reasons

    bad = copy.deepcopy(first)
    bad["command"]["linear_x"] = math.nan
    try:
        validate_envelope(bad)
    except ContractError:
        pass
    else:
        raise AssertionError("U9 accepted NaN")

    estop = make_core()
    estop.engage("operator_emergency_stop")
    decision = healthy_step(estop)
    assert decision.zeroed and "emergency_stop" in decision.reasons  # U10

    # F1-F5: each fault is checked against the independent fail-safe reason.
    f1 = healthy_step(make_core(), now=100.40, relay_seen=100.39, source_seen=100.00)
    f2 = healthy_step(make_core(), now=100.40, relay_seen=100.00, source_seen=100.39)
    f3 = healthy_step(make_core(), bridge_failure=True, bridge_failure_reason="worker_crash")
    f4_tracker = FreshnessTracker()
    f4_tracker.accept(first)
    ok, reason = f4_tracker.accept(envelope(request_id=0, cycle_id=0, scan_counter=0, source_time=100.0))
    assert not ok and reason == "stale_result"
    f4 = healthy_step(make_core(), metadata_fault="stale_result")
    f5 = make_core().step(
        100.40, 0.10, 0.05, command_seen=100.39, relay_seen=100.39,
        planner_result_seen=100.39, scan_seen=100.00, odom_seen=100.39,
        tf_seen=100.39, timing_seen=100.39,
    )
    assert f1.zeroed and "planner_freshness_watchdog" in f1.reasons
    assert f2.zeroed and "relay_watchdog" in f2.reasons
    assert f3.zeroed and "worker_crash" in f3.reasons
    assert f4.zeroed and "stale_result" in f4.reasons
    assert f5.zeroed and "scan_watchdog" in f5.reasons
    result = {
        "schema": "r1_n2_3_fault_injection_unit_v1",
        "unit_tests": {"U1": "PASS", "U2": "PASS", "U2_source_integrity": "PASS", "U3": "PASS",
                       "U4": "PASS", "U5": "PASS", "U6": "PASS",
                       "U7": "PASS", "U8": "PASS", "U9": "PASS",
                       "U10": "PASS"},
        "fault_injection": {
            "F1_stop_planner_result_updates_relay_alive": {
                "expected": "planner_freshness_watchdog -> zero", "observed": "PASS"},
            "F2_stop_relay": {
                "expected": "relay_watchdog -> zero", "observed": "PASS"},
            "F3_worker_crash": {
                "expected": "worker_crash bridge failure -> zero", "observed": "PASS"},
            "F4_stale_response": {
                "expected": "stale_result -> zero", "observed": "PASS"},
            "F5_stop_scan": {
                "expected": "scan_watchdog -> zero", "observed": "PASS"},
        },
        "live_ros_fault_injection": False,
        "note": "Deterministic pure-contract evidence; no live fault was injected into Gazebo.",
    }
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("N2.3 contract unit PASS: U1-U10, F1-F5")


if __name__ == "__main__":
    main()
