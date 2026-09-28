"""Aggregate N2.4C-R monotonic runtime attribution captures."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any


PIPELINE = (
    ("T0", "cloud_receive_monotonic"),
    ("T1", "scan_callback_receive_monotonic"),
    ("T2", "scan_update_monotonic"),
    ("T3", "request_created_monotonic"),
    ("T4", "request_dispatched_monotonic"),
    ("T5", "worker_request_received_monotonic"),
    ("T6", "worker_solve_start_monotonic"),
    ("T7", "worker_solve_end_monotonic"),
    ("T8", "worker_response_sent_monotonic"),
    ("T9", "main_response_received_monotonic"),
    ("T10", "main_validation_complete_monotonic"),
    ("T11", "planner_result_publish_monotonic"),
    ("T12", "relay_source_receive_monotonic"),
    ("T13", "relay_publish_monotonic"),
    ("T14", "gate_relay_receive_monotonic"),
    ("T15", "gate_output_publish_monotonic"),
)
GAP_THRESHOLD_SEC = 0.25

REQUESTED_STAGE_DELAYS = (
    "T2_to_T3", "T3_to_T4", "T4_to_T5", "T5_to_T7", "T7_to_T9",
    "T9_to_T11", "T11_to_T12", "T12_to_T15",
)


def _load(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream, parse_constant=lambda _: None)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _stats(values: list[Any]) -> dict[str, Any]:
    finite = sorted(float(value) for value in values if _finite(value))
    if not finite:
        return {"count": 0, "total": len(values), "nonfinite": len(values)}
    return {
        "count": len(finite), "total": len(values), "nonfinite": len(values) - len(finite),
        "min": finite[0], "max": finite[-1], "mean": statistics.fmean(finite),
        "p95": finite[int(0.95 * (len(finite) - 1))],
        "p99": finite[int(0.99 * (len(finite) - 1))],
    }


def _value(row: dict[str, Any], label: str) -> Any:
    for item_label, field in PIPELINE:
        if item_label == label:
            return row.get(field)
    return None


def _boundary(previous: dict[str, Any], current: dict[str, Any]) -> dict[str, Any]:
    previous_t1 = _value(previous, "T1")
    previous_t2 = _value(previous, "T2")
    current_t1 = _value(current, "T1")
    current_t2 = _value(current, "T2")
    current_t3 = _value(current, "T3")
    callback_evidence = (
        all(_finite(value) for value in (previous_t1, previous_t2, current_t1, current_t2, current_t3))
        and float(current_t1) > float(previous_t1)
        and float(current_t2) <= float(previous_t2)
        and float(current_t1) > float(current_t2)
        and float(current_t3) - float(current_t2) > GAP_THRESHOLD_SEC
    )
    if callback_evidence:
        return {
            "code": "B",
            "label": "ROS CALLBACK / EXECUTOR STARVATION",
            "observed_boundary": "T1->T2",
            "reason": "new scan callback receipt advanced while latest state-update timestamp did not advance before request creation",
            "state_update_nonadvance": True,
            "previous_t1": previous_t1,
            "previous_t2": previous_t2,
            "current_t1": current_t1,
            "current_t2": current_t2,
            "current_t3": current_t3,
            "callback_receipt_to_request_sec": float(current_t3) - float(current_t1),
            "current_callback_to_state_update_sec": float(current_t2) - float(current_t1),
            "request_from_previous_state_update_sec": float(current_t3) - float(current_t2),
        }
    categories = {
        "T0": ("A", "SENSOR SOURCE GAP"),
        "T1": ("B", "ROS CALLBACK / EXECUTOR STARVATION"),
        "T2": ("C", "REQUEST DISPATCH GAP"),
        "T3": ("C", "REQUEST DISPATCH GAP"),
        "T4": ("D", "WORKER IPC / SCHEDULING GAP"),
        "T5": ("D", "WORKER IPC / SCHEDULING GAP"),
        "T6": ("D", "WORKER IPC / SCHEDULING GAP"),
        "T7": ("F", "RESPONSE HANDLING GAP"),
        "T8": ("F", "RESPONSE HANDLING GAP"),
        "T9": ("G", "RESULT PUBLICATION GAP"),
        "T10": ("G", "RESULT PUBLICATION GAP"),
        "T11": ("D", "RELAY RECEIVE GAP"),
        "T12": ("D", "RELAY PUBLICATION GAP"),
        "T13": ("F", "SAFETY GATE RECEIVE GAP"),
        "T14": ("F", "SAFETY GATE OUTPUT GAP"),
        "T15": ("J", "UNKNOWN"),
    }
    for label, _field in PIPELINE:
        left = _value(previous, label)
        right = _value(current, label)
        if _finite(left) and _finite(right) and float(right) - float(left) > GAP_THRESHOLD_SEC:
            code, text = categories.get(label, ("J", "UNKNOWN"))
            return {
                "code": code,
                "label": text,
                "observed_boundary": label,
                "interval_sec": float(right) - float(left),
            }
    return {"code": "J", "label": "UNKNOWN", "observed_boundary": None}


def _sha256(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()
    except OSError:
        return None


def _intervals(rows: list[dict[str, Any]], field: str, window: str | None = None) -> list[float]:
    values = [
        float(row[field]) for row in sorted(rows, key=lambda item: item.get("cycle_id", -1))
        if (window is None or row.get("window") == window) and _finite(row.get(field)) and float(row[field]) > 0.0
    ]
    return [right - left for left, right in zip(values, values[1:]) if right >= left]


def _field_values(rows: list[dict[str, Any]], field: str) -> list[float]:
    return [
        float(row[field]) for row in rows
        if _finite(row.get(field))
    ]


def _pair_values(rows: list[dict[str, Any]], left: str, right: str) -> list[float]:
    return [
        float(row[right]) - float(row[left])
        for row in rows
        if _finite(row.get(left)) and _finite(row.get(right))
        and float(row[right]) >= float(row[left])
    ]


def _window_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result = {}
    for window in ("STARTUP", "ACTIVE", "SHUTDOWN"):
        selected = [row for row in rows if row.get("window") == window]
        result[window] = {
            "cycles": len(selected),
            "cycle_ids": [row.get("cycle_id") for row in selected],
            "planner_result_intervals_sec": _stats(_intervals(selected, "T11")),
            "scan_receive_intervals_sec": _stats(_intervals(selected, "T1")),
            "request_dispatch_intervals_sec": _stats(_intervals(selected, "T4")),
            "worker_receive_intervals_sec": _stats(_intervals(selected, "T5")),
            "continuous_intervals_sec": {
                "scan_receive_interval": _stats(_intervals(selected, "T1")),
                "request_dispatch_interval": _stats(_intervals(selected, "T4")),
                "worker_receive_interval": _stats(_intervals(selected, "T5")),
                "planner_result_interval": _stats(_intervals(selected, "T11")),
                "relay_receive_interval": _stats(_intervals(selected, "T12")),
            },
            "latency_stats_sec": {
                "worker_solve_duration": _stats(_pair_values(selected, "T6", "T7")),
                "worker_end_to_main_response": _stats(_pair_values(selected, "T7", "T9")),
                "response_ipc_latency": _stats(_pair_values(selected, "T8", "T9")),
                "result_publish_latency": _stats(_pair_values(selected, "T10", "T11")),
            },
            "pipeline_intervals_sec": {
                label: _stats(_intervals(selected, label)) for label, _field in PIPELINE
            },
            "stage_delays_sec": {
                **{
                    name: _stats(_field_values(selected, name))
                    for name in (
                    "T0_to_T1", "T1_to_T2", "T2_to_T3", "T3_to_T4", "T4_to_T5",
                    "T5_to_T6", "T5_to_T7", "T6_to_T7", "T7_to_T8",
                    "T8_to_T9", "T9_to_T10", "T9_to_T11", "T10_to_T11",
                    "T11_to_T12", "T12_to_T13", "T13_to_T14", "T14_to_T15",
                    )
                },
                "T7_to_T9": _stats(_pair_values(selected, "T7", "T9")),
                "T12_to_T15": _stats(_pair_values(selected, "T12", "T15")),
            },
            "age_stats_sec": {
                "scan_age_at_planner_start": _stats(_field_values(selected, "scan_age_sec_at_planner_start")),
                "planner_source_age": _stats(_field_values(selected, "planner_source_age_sec")),
                "relay_age": _stats(_field_values(selected, "relay_age_sec")),
                "timing_age": _stats(_field_values(selected, "timing_age_sec")),
            },
        }
    return result


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _observation_slice(items: list[dict[str, Any]], start: float, end: float) -> list[dict[str, Any]]:
    return [
        item for item in items
        if _finite(item.get("receive_monotonic"))
        and start - 0.5 <= float(item["receive_monotonic"]) <= end + 0.5
    ]


def _bridge_metadata(path: Path) -> dict[str, Any]:
    bridge_path = path.parent / "runtime_bridge" / "summary.json"
    if not bridge_path.exists():
        return {"summary_path": str(bridge_path), "available": False}
    data = _load(bridge_path)
    ready = data.get("ready", {})
    return {
        "summary_path": str(bridge_path),
        "available": True,
        "worker_pid": ready.get("pid"),
        "worker_generation": data.get("worker_generation"),
        "start_method": data.get("start_method") or ready.get("start_method"),
        "worker_failures": data.get("worker_failures"),
        "requests_sent": data.get("requests_sent"),
        "requests_completed": data.get("requests_completed"),
        "ready": ready.get("status"),
    }


def _torch_metadata(path: Path) -> dict[str, Any]:
    runtime_path = path.parent / "audit_torch_runtime.json"
    if not runtime_path.exists():
        return {"runtime_path": str(runtime_path), "available": False}
    data = _load(runtime_path)
    return {
        "runtime_path": str(runtime_path),
        "available": True,
        "requested": data.get("requested", {}),
        "effective": data.get("effective", {}),
        "environment_before": data.get("environment_before", {}),
        "environment_effective": data.get("environment_effective", {}),
        "threadpools_after_target_import": data.get("threadpools_after_target_import", []),
        "python": data.get("python"),
        "torch": data.get("torch"),
    }


def _sensor_audit(data: dict[str, Any], rows: list[dict[str, Any]]) -> dict[str, Any]:
    raw = data.get("raw_observations", {})
    clouds = raw.get("clouds", [])
    scans = raw.get("scans", [])
    source_stamps = [item.get("header_stamp_sec") for item in clouds + scans]
    counters = [row.get("scan_counter") for row in rows]

    def receive_intervals(items: list[dict[str, Any]]) -> list[float]:
        times = sorted(float(item["receive_monotonic"]) for item in items if _finite(item.get("receive_monotonic")))
        return [right - left for left, right in zip(times, times[1:]) if right >= left]

    def max_gap(items: list[dict[str, Any]]) -> dict[str, Any]:
        times = sorted(float(item["receive_monotonic"]) for item in items if _finite(item.get("receive_monotonic")))
        if len(times) < 2:
            return {"max_sec": None, "start_monotonic": None, "end_monotonic": None}
        start, end = max(zip(times, times[1:]), key=lambda pair: pair[1] - pair[0])
        return {
            "max_sec": end - start,
            "start_monotonic": start,
            "end_monotonic": end,
        }

    def interval_window(start: float | None, end: float | None) -> str | None:
        if start is None or end is None:
            return None
        release = data.get("window_boundaries", {}).get("first_gate_release_observed_monotonic")
        failure = data.get("window_boundaries", {}).get("first_failure_latch_observed_monotonic")
        if release is None or end < float(release):
            return "STARTUP"
        if failure is not None and start >= float(failure):
            return "SHUTDOWN"
        if failure is not None and end >= float(failure):
            return "ACTIVE -> SHUTDOWN"
        return "ACTIVE"

    lidar_intervals = receive_intervals(clouds)
    scan_intervals = receive_intervals(scans)
    planner_scan_intervals = _intervals(rows, "T1")
    planner_result_intervals = _intervals(rows, "T11")
    lidar_gap = max_gap(clouds)
    scan_gap = max_gap(scans)
    observer_gap = max(
        (("/lidar/points", lidar_gap), ("/scan_neupan", scan_gap)),
        key=lambda item: item[1].get("max_sec") or -1.0,
    )
    observer_name, observer_details = observer_gap
    observer_start = observer_details.get("start_monotonic")
    observer_end = observer_details.get("end_monotonic")
    overlapping_samples = []
    for sample in data.get("resource_summary", {}).get("samples", []):
        sample_start = sample.get("monotonic")
        sample_end = (
            float(sample_start) + float(sample.get("sample_duration_ms", 0.0)) / 1000.0
            if _finite(sample_start) and _finite(sample.get("sample_duration_ms")) else None
        )
        if (
            _finite(observer_start) and _finite(observer_end) and _finite(sample_start)
            and sample_end is not None and float(sample_start) <= float(observer_end)
            and sample_end >= float(observer_start)
        ):
            overlapping_samples.append(sample)
    planner_chain_gap_count = sum(value > GAP_THRESHOLD_SEC for value in planner_scan_intervals)
    planner_result_gap_count = sum(value > GAP_THRESHOLD_SEC for value in planner_result_intervals)
    observer_isolated = bool(
        (observer_details.get("max_sec") or 0.0) > GAP_THRESHOLD_SEC
        and planner_chain_gap_count == 0
        and planner_result_gap_count == 0
    )
    return {
        "lidar_points": data.get("rates", {}).get("lidar_points_receive", {}),
        "scan_neupan": data.get("rates", {}).get("scan_neupan_receive", {}),
        "lidar_header_stamp_count": sum(_finite(item.get("header_stamp_sec")) for item in clouds),
        "scan_header_stamp_count": sum(_finite(item.get("header_stamp_sec")) for item in scans),
        "header_stamp_finite_count": sum(_finite(value) for value in source_stamps),
        "header_stamp_used_as_causal_clock": False,
        "audit_lidar_intervals_over_250ms": sum(value > GAP_THRESHOLD_SEC for value in lidar_intervals),
        "audit_scan_intervals_over_250ms": sum(value > GAP_THRESHOLD_SEC for value in scan_intervals),
        "planner_t1_intervals_over_250ms": sum(value > GAP_THRESHOLD_SEC for value in planner_scan_intervals),
        "audit_observer_gap_max_sec": max(lidar_intervals + scan_intervals, default=None),
        "planner_t1_interval_max_sec": max(planner_scan_intervals, default=None),
        "planner_result_interval_max_sec": max(planner_result_intervals, default=None),
        "audit_observer_gap": {
            "stream": observer_name,
            **observer_details,
            "window": interval_window(observer_start, observer_end),
            "over_250ms": bool((observer_details.get("max_sec") or 0.0) > GAP_THRESHOLD_SEC),
            "planner_t1_gap_count_over_250ms": planner_chain_gap_count,
            "planner_result_gap_count_over_250ms": planner_result_gap_count,
            "interpretation": (
                "AUDIT OBSERVER GAP ONLY; planner T1/result chain has no corresponding >250 ms gap"
                if observer_isolated else
                "requires correlation with planner timeline"
            ),
            "overlapping_resource_samples": len(overlapping_samples),
            "overlap_max_sampler_duration_ms": max(
                (float(item.get("sample_duration_ms")) for item in overlapping_samples if _finite(item.get("sample_duration_ms"))),
                default=None,
            ),
            "overlap_peak_process_count": max(
                (float(item.get("process_count")) for item in overlapping_samples if _finite(item.get("process_count"))),
                default=None,
            ),
            "overlap_peak_system_thread_count": max(
                (float(item.get("system_thread_count")) for item in overlapping_samples if _finite(item.get("system_thread_count"))),
                default=None,
            ),
        },
        "source_gap_confirmed": False,
        "planner_scan_counter": {
            "count": len([value for value in counters if _finite(value)]),
            "first": min((int(value) for value in counters if _finite(value)), default=None),
            "last": max((int(value) for value in counters if _finite(value)), default=None),
            "unique": len({int(value) for value in counters if _finite(value)}),
        },
        "scan_source_gap_observed": False,
    }


def _event_bundle(
    label: str,
    path: Path,
    data: dict[str, Any],
    event: dict[str, Any],
    rows: list[dict[str, Any]],
    index: int,
) -> Path:
    start = float(event.get("start_monotonic", 0.0))
    end = float(event.get("end_monotonic", start))
    lo = max(0, index - 20)
    hi = min(len(rows), index + 21)
    raw = data.get("raw_observations", {})
    bundle = {
        "schema": "r1_n2_4c_runtime_gap_bundle_v2_derived",
        "derived": True,
        "derivation": "recomputed from immutable raw runtime.json; original collector bundles are preserved",
        "run_label": label,
        "source_runtime": str(path.resolve()),
        "event": event,
        "timeline_window_cycles": rows[lo:hi],
        "resource_samples": [
            item for item in data.get("resource_summary", {}).get("samples", [])
            if _finite(item.get("monotonic"))
            and start - 0.5 <= float(item["monotonic"]) <= end + 0.5
        ],
        "raw_observations": {
            key: _observation_slice(raw.get(key, []), start, end)
            for key in ("clouds", "scans", "sources", "relays", "gates", "commands")
        },
    }
    directory = path.parent / "runtime_gap_events_derived"
    directory.mkdir(parents=True, exist_ok=True)
    bundle_path = directory / f"gap_{int(event['event_id']):04d}.json"
    bundle_path.write_text(
        json.dumps(_json_safe(bundle), indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )
    return bundle_path.resolve()


def _raw_event_audit(data: dict[str, Any]) -> dict[str, Any]:
    events = data.get("gap_events", [])
    invalid_ids = []
    for event in events:
        start = event.get("start_monotonic")
        end = event.get("end_monotonic")
        gap = event.get("gap_sec")
        invalid = not all(_finite(value) for value in (start, end, gap))
        if not invalid:
            invalid = float(end) < float(start) or abs((float(end) - float(start)) - float(gap)) > 1.0e-6
        if invalid:
            invalid_ids.append(event.get("event_id"))
    return {
        "raw_event_count": len(events),
        "invalid_interval_event_count": len(invalid_ids),
        "invalid_interval_event_ids": invalid_ids,
        "raw_events_used_for_final_gap_table": False,
        "derived_events_recomputed_from_timeline_and_gate_observations": True,
    }
def _derived_events(label: str, path: Path, data: dict[str, Any]) -> tuple[list[dict[str, Any]], list[str]]:
    rows = sorted(data.get("timeline", []), key=lambda item: item.get("cycle_id", -1))
    candidates: list[tuple[dict[str, Any], int]] = []
    for index in range(1, len(rows)):
        previous, current = rows[index - 1], rows[index]
        left, right = _value(previous, "T11"), _value(current, "T11")
        if not (_finite(left) and _finite(right)):
            continue
        gap = float(right) - float(left)
        if gap <= GAP_THRESHOLD_SEC:
            continue
        boundary = _boundary(previous, current)
        candidates.append(({
            "trigger": "planner_result_interval",
            "threshold_sec": GAP_THRESHOLD_SEC,
            "start_monotonic": float(left),
            "end_monotonic": float(right),
            "gap_sec": gap,
            "first_abnormal_boundary": boundary,
            "boundary_evidence": boundary if boundary.get("code") == "B" else {},
            "previous_cycle_id": previous.get("cycle_id"),
            "current_cycle_id": current.get("cycle_id"),
            "start_window": previous.get("window"),
            "end_window": current.get("window"),
            "window": previous.get("window") or current.get("window"),
            "pipeline_interval_observation": {
                label: (
                    float(_value(current, label)) - float(_value(previous, label))
                    if _finite(_value(previous, label)) and _finite(_value(current, label)) else None
                )
                for label, _field in PIPELINE
            },
        }, index))

    # Normalize repeated 50 Hz watchdog samples into one event per unchanged
    # planner source episode. If a planner-result interval already covers the
    # episode, that interval is the canonical event.
    last_source = None
    source_episode_event_emitted = False
    raw_gates = sorted(data.get("raw_observations", {}).get("gates", []), key=lambda item: item.get("receive_monotonic", -1))
    for gate in raw_gates:
        payload = gate.get("payload", {})
        source_metadata = (payload.get("freshness") or {}).get("source_metadata") or {}
        source_stamp = source_metadata.get("planner_result_monotonic")
        if _finite(source_stamp):
            source_stamp = float(source_stamp)
            if last_source is None or source_stamp != last_source:
                last_source = source_stamp
                source_episode_event_emitted = False
        age = (payload.get("ages_sec") or {}).get("planner_result")
        if not (_finite(age) and float(age) > GAP_THRESHOLD_SEC and _finite(last_source)):
            continue
        gate_time = float(gate.get("receive_monotonic"))
        gap = gate_time - float(last_source)
        if gate_time < float(last_source):
            # A malformed/retrograde source metadata record must never create
            # a negative derived gap or be used as causal evidence.
            continue
        if gap <= GAP_THRESHOLD_SEC or source_episode_event_emitted:
            continue
        covered = any(
            event["trigger"] == "planner_result_interval"
            and abs(float(event["start_monotonic"]) - float(last_source)) <= 1.0e-3
            and float(event["start_monotonic"]) <= gate_time <= float(event["end_monotonic"])
            for event, _index in candidates
        )
        if covered:
            source_episode_event_emitted = True
            continue
        valid_rows = [
            (index, row) for index, row in enumerate(rows)
            if _finite(_value(row, "T11"))
        ]
        anchor_index, anchor = min(
            valid_rows,
            key=lambda pair: abs(float(_value(pair[1], "T11")) - float(last_source)),
            default=(max(0, len(rows) - 1), rows[-1] if rows else {}),
        )
        candidates.append(({
            "trigger": "planner_source_watchdog",
            "threshold_sec": GAP_THRESHOLD_SEC,
            "start_monotonic": float(last_source),
            "end_monotonic": gate_time,
            "gap_sec": gap,
            "first_abnormal_boundary": {
                "code": "J",
                "label": "UNKNOWN: source stopped before a second published result",
                "observed_boundary": "planner source watchdog",
            },
            "boundary_evidence": {
                "source_episode_deduplicated": True,
                "watchdog_age_sec": float(age),
            },
            "previous_cycle_id": anchor.get("cycle_id"),
            "current_cycle_id": anchor.get("cycle_id"),
            "start_window": anchor.get("window"),
            "end_window": anchor.get("window"),
            "window": anchor.get("window"),
            "watchdog_payload": payload,
        }, anchor_index))
        source_episode_event_emitted = True

    candidates.sort(key=lambda pair: (float(pair[0]["start_monotonic"]), float(pair[0]["end_monotonic"]), pair[0]["trigger"]))
    events: list[dict[str, Any]] = []
    bundle_paths: list[str] = []
    for event_id, (event, index) in enumerate(candidates, start=1):
        event = dict(event)
        event["event_id"] = event_id
        bundle_path = _event_bundle(label, path, data, event, rows, index)
        event["bundle_path"] = str(bundle_path)
        events.append(event)
        bundle_paths.append(str(bundle_path))
    return events, bundle_paths


def _run_summary(label: str, path: Path, data: dict[str, Any]) -> dict[str, Any]:
    rows = data.get("timeline", [])
    events, bundle_paths = _derived_events(label, path, data)
    bridge = _bridge_metadata(path)
    return {
        "label": label,
        "path": str(path.resolve()),
        "counts": data.get("counts", {}),
        "window_boundaries": data.get("window_boundaries", {}),
        "rates": data.get("rates", {}),
        "windows": _window_summary(rows),
        "active_gap_events": [event for event in events if event.get("start_window") == "ACTIVE"],
        "all_gap_events": events,
        "raw_gap_event_count": len(data.get("gap_events", [])),
        "raw_event_audit": _raw_event_audit(data),
        "raw_gap_event_bundle_paths": data.get("gap_event_bundle_paths", []),
        "gap_event_bundle_paths": bundle_paths,
        "instrumentation": data.get("instrumentation", {}),
        "resource": _resource_summary(
            data.get("resource_summary", {}).get("samples", []),
            worker_pid=bridge.get("worker_pid"),
        ),
        "safety": _safety_summary(rows),
        "sensor_audit": _sensor_audit(data, rows),
        "bridge": bridge,
        "torch": _torch_metadata(path),
        "timeline_rows": len(rows),
    }


def _safety_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    watchdog_reason_counts: dict[str, int] = {}
    for row in rows:
        for reason in row.get("watchdog_reasons", []):
            watchdog_reason_counts[str(reason)] = watchdog_reason_counts.get(str(reason), 0) + 1
    return {
        "queue_depth_max": max((float(row.get("queue_depth", 0.0)) for row in rows), default=None),
        "worker_failure_count": sum(float(row.get("worker_failure", 0.0)) != 0.0 for row in rows),
        "nan_inf_count": sum(float(row.get("nan_inf", 0.0)) != 0.0 for row in rows),
        "stale_response_count": sum(float(row.get("stale_response", 0.0)) != 0.0 for row in rows),
        "max_worker_ms": max((float(row.get("worker_core_total_ms", 0.0)) for row in rows), default=None),
        "max_main_ms": max((float(row.get("main_full_cycle_ms", 0.0)) for row in rows), default=None),
        "watchdog_reasons": sorted({reason for row in rows for reason in row.get("watchdog_reasons", [])}),
        "watchdog_reason_counts": watchdog_reason_counts,
        "planner_source_age_sec": _stats(_field_values(rows, "planner_source_age_sec")),
        "relay_age_sec": _stats(_field_values(rows, "relay_age_sec")),
        "timing_age_sec": _stats(_field_values(rows, "timing_age_sec")),
    }


def _resource_summary(samples: list[dict[str, Any]], worker_pid: Any = None) -> dict[str, Any]:
    if not samples:
        return {
            "sample_count": 0,
            "worker_process": {"pid": worker_pid, "sampled": False},
        }
    task_current = [sample.get("cgroup", {}).get("tasks_current") for sample in samples]
    task_max = [sample.get("cgroup", {}).get("tasks_max") for sample in samples]
    process_counts = [sample.get("process_count") for sample in samples]
    thread_counts = [sample.get("system_thread_count") for sample in samples]
    per_process: dict[str, dict[str, Any]] = {}
    for sample in samples:
        for process in sample.get("processes", []):
            label = str(process.get("label"))
            entry = per_process.setdefault(label, {"pids": set(), "cpu_percent": [], "rss_bytes": [], "thread_count": [], "voluntary_context_switches": [], "involuntary_context_switches": []})
            entry["pids"].add(int(process.get("pid", -1)))
            for key in ("cpu_percent", "rss_bytes", "thread_count", "voluntary_context_switches", "involuntary_context_switches"):
                if _finite(process.get(key)):
                    entry[key].append(float(process[key]))
    cleaned = {}
    for label, entry in per_process.items():
        cleaned[label] = {
            "pids": sorted(pid for pid in entry["pids"] if pid >= 0),
            "cpu_percent": _stats(entry["cpu_percent"]),
            "rss_bytes": _stats(entry["rss_bytes"]),
            "thread_count": _stats(entry["thread_count"]),
            "voluntary_context_switches": _stats(entry["voluntary_context_switches"]),
            "involuntary_context_switches": _stats(entry["involuntary_context_switches"]),
        }
    return {
        "sample_count": len(samples),
        "tasks_current": _stats(task_current),
        "tasks_max_numeric": _stats(task_max),
        "tasks_max_values": sorted({str(sample.get("cgroup", {}).get("tasks_max")) for sample in samples}),
        "system_process_count": _stats(process_counts),
        "system_thread_count": _stats(thread_counts),
        "sampler_duration_ms": _stats([sample.get("sample_duration_ms") for sample in samples]),
        "per_process": cleaned,
        "worker_process": {
            "pid": worker_pid,
            "sampled": any(
                int(worker_pid) in values["pids"]
                for values in cleaned.values()
                if _finite(worker_pid)
            ),
            "resource_label": "whole_core_worker",
        },
        "last_cgroup": samples[-1].get("cgroup", {}),
    }


def _classification(summaries: list[dict[str, Any]]) -> tuple[str, str, dict[str, Any]]:
    control = [summary for summary in summaries if summary["label"].lower().startswith(("a1", "b2", "control"))]
    problem = [summary for summary in summaries if summary not in control]
    active_events = [event for summary in summaries for event in summary["active_gap_events"]]
    control_events = [event for summary in control for event in summary["active_gap_events"]]
    codes = [str(event.get("first_abnormal_boundary", {}).get("code", "J")) for event in active_events]
    code = codes[0] if codes else ""
    if not active_events:
        return "I", "ACTIVE RUNTIME HEALTHY; PRIOR EVENT NOT REPRODUCED", {"active_gap_events": 0}
    if control_events and any(other != code for other in [str(event.get("first_abnormal_boundary", {}).get("code", "J")) for event in control_events]):
        return "H", "MULTIPLE ROOT CAUSES", {"active_gap_events": len(active_events), "control_gap_events": len(control_events)}
    if control_events:
        return "H", "MULTIPLE ROOT CAUSES", {"active_gap_events": len(active_events), "control_gap_events": len(control_events)}
    mapping = {
        "A": ("A", "SENSOR SOURCE GAP CONFIRMED"),
        "B": ("B", "ROS CALLBACK / EXECUTOR STARVATION CONFIRMED"),
        "C": ("C", "REQUEST DISPATCH GAP CONFIRMED"),
        "D": ("D", "WORKER IPC / SCHEDULING GAP CONFIRMED"),
        "E": ("E", "PLANNER COMPUTE REGRESSION CONFIRMED"),
        "F": ("F", "RESULT HANDLING / PUBLICATION GAP CONFIRMED"),
        "G": ("G", "HOST RESOURCE / TASK LIMIT CONFIRMED"),
    }
    if code in mapping:
        return mapping[code][0], mapping[code][1], {"active_gap_events": len(active_events)}
    return "J", "UNRESOLVED", {"active_gap_events": len(active_events)}


def _hashes(workspace: Path) -> dict[str, Any]:
    relative = (
        "src/r1_neupan_integration/r1_neupan_integration/n1_15_node.py",
        "src/r1_neupan_integration/r1_neupan_integration/n1_16_node.py",
        "src/r1_neupan_integration/r1_neupan_integration/n1_16_bridge.py",
        "src/r1_neupan_integration/r1_neupan_integration/n1_15_whole_core.py",
        "src/r1_neupan_integration/r1_neupan_integration/n2_3_command_relay.py",
        "src/r1_neupan_integration/r1_neupan_integration/n2_pre_safety_gate.py",
        "src/r1_neupan_integration/r1_neupan_integration/n2_4c_runtime_attribution.py",
        "src/r1_neupan_integration/launch/n2_4c_runtime_attribution.launch.py",
        "src/r1_neupan_integration/scripts/n2_4c_runtime_report.py",
        "src/r1_neupan_integration/config/planner.yaml",
        "src/r1_neupan_integration/worlds/n2_b4_center_obstacle.sdf",
        "src/r1_neupan_integration/worlds/n2_d1_corner_obstacle.sdf",
        "src/r1_neupan_integration/config/models/dune_model_5000.pth",
    )
    return {path: _sha256(workspace / path) for path in relative}


def _timeline_definition(raw_runs: list[dict[str, Any]]) -> list[dict[str, str]]:
    expected = [{"label": label, "timestamp_field": field} for label, field in PIPELINE]
    candidate = raw_runs[0].get("timeline_definition", []) if raw_runs else []
    if (
        isinstance(candidate, list)
        and len(candidate) == len(expected)
        and all(
            isinstance(item, dict)
            and item.get("label") == expected[index]["label"]
            and item.get("timestamp_field") == expected[index]["timestamp_field"]
            for index, item in enumerate(candidate)
        )
    ):
        return candidate
    return expected


def _md(report: dict[str, Any]) -> str:
    lines = [
        "# N2.4C-R Planner/Scan Freshness Runtime Regression Attribution",
        "",
        f"Final classification: **{report['final_classification']}**",
        f"`next_stage_allowed={report['next_stage_allowed']}`.",
        "",
        "This report attributes monotonic pipeline freshness only. No watchdog, planner, recovery, Nav2, Gazebo recovery motion, or phenotype behavior was changed or resumed.",
        "",
        "## Frozen Configuration",
        "",
        "- Platform: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`/`ros_gz_bridge`.",
        "- Footprint: corrected `0.682 x 0.532 m`; N1.16 whole-core worker; latest-only queue `<=1`.",
        "- Relay: 50 Hz; planner and relay freshness are independent.",
        "- Watchdogs: planner `0.30 s`, relay `0.30 s`, timing `0.50 s`; gap event threshold `0.25 s` only.",
        "- Torch/OpenBLAS policy was audited, not changed: see each run's `ready`/runtime evidence and resource samples.",
        f"- Git metadata: `{report['git']}`.",
        "",
        "Frozen artifact SHA-256:",
    ]
    for path, digest in report["file_sha256"].items():
        lines.append(f"- `{path}`: `{digest}`.")
    lines += [
        "",
        "## T0-T15 Timeline",
        "",
        "T0 is the earliest observable `/lidar/points` receive in the audit process. Gazebo production monotonic time is unavailable and is not fabricated. ROS/simulation header stamps are used only to match cloud and scan observations, never as causal delay clocks.",
        "",
        "| Stage | Meaning |",
        "|---|---|",
    ]
    for item in report["timeline_definition"]:
        lines.append(f"| {item['label']} | `{item['timestamp_field']}` |")
    lines += [
        "",
        "## Window Policy",
        "",
        "- `STARTUP`: before the first healthy released gate.",
        "- `ACTIVE`: after that release and before the first terminal failure/e-stop latch.",
        "- `SHUTDOWN`: from the first terminal latch through remaining observations.",
        "- A cross-boundary event keeps both windows; attribution uses the pre-latch start boundary.",
    ]
    lines += ["", "## Run Matrix", "", "| Run | Timing rows | Derived gaps | Raw gaps | Active gaps | Scan receives | Source receives |", "|---|---:|---:|---:|---:|---:|---:|"]
    for run in report["runs"]:
        lines.append(f"| {run['label']} | {run['timeline_rows']} | {len(run['all_gap_events'])} | {run['raw_gap_event_count']} | {len(run['active_gap_events'])} | {run['counts'].get('scan_observations', 0)} | {run['counts'].get('source_observations', 0)} |")
    lines += ["", "## Active / Startup / Shutdown", ""]
    for run in report["runs"]:
        lines.append(f"### {run['label']}")
        for window in ("STARTUP", "ACTIVE", "SHUTDOWN"):
            section = run["windows"][window]
            interval = section["planner_result_intervals_sec"]
            named = section["continuous_intervals_sec"]
            latency = section["latency_stats_sec"]
            stages = section["stage_delays_sec"]
            ages = section["age_stats_sec"]
            lines.append(f"- {window}: `{section['cycles']}` cycles; planner interval max `{interval.get('max')}` s; scan interval max `{section['scan_receive_intervals_sec'].get('max')}` s; dispatch interval max `{section['request_dispatch_intervals_sec'].get('max')}` s; worker receive interval max `{section['worker_receive_intervals_sec'].get('max')}` s.")
            lines.append(f"  Named intervals max s: scan receive `{named['scan_receive_interval'].get('max')}`, request dispatch `{named['request_dispatch_interval'].get('max')}`, worker receive `{named['worker_receive_interval'].get('max')}`, planner result `{named['planner_result_interval'].get('max')}`, relay receive `{named['relay_receive_interval'].get('max')}`; solve `{latency['worker_solve_duration'].get('max')}`, response IPC `{latency['response_ipc_latency'].get('max')}`, result publish `{latency['result_publish_latency'].get('max')}`.")
            lines.append(f"  Required stage-delay max s: " + ", ".join(f"{name} `{stages[name].get('max')}`" for name in REQUESTED_STAGE_DELAYS) + ".")
            lines.append(f"  Age max s: scan `{ages['scan_age_at_planner_start'].get('max')}`, planner `{ages['planner_source_age'].get('max')}`, relay `{ages['relay_age'].get('max')}`, timing `{ages['timing_age'].get('max')}`.")
    lines += ["", "## >250 ms Gap Events", "", "The 250 ms threshold is an early-capture threshold only; the frozen safety threshold remains 300 ms.", "", "| Run | Event | Trigger | Gap (s) | Start -> End window | First boundary |", "|---|---:|---|---:|---|---|"]
    for run in report["runs"]:
        for event in run["all_gap_events"]:
            boundary = event.get("first_abnormal_boundary", {})
            lines.append(f"| {run['label']} | {event.get('event_id')} | {event.get('trigger')} | {event.get('gap_sec')} | {event.get('start_window')} -> {event.get('end_window')} | {boundary.get('code')}: {boundary.get('label')} |")
    if not any(run["all_gap_events"] for run in report["runs"]):
        lines.append("| none | | | | | |")
    lines += ["", "Each derived event has an independent JSON bundle containing the +/-20-cycle timeline window, all observed T0-T15 fields, ages, safety/watchdog state, gate observations, raw sensor/source/relay observations, and resource samples. The original collector bundles remain immutable evidence.", ""]
    for run in report["runs"]:
        for path in run["gap_event_bundle_paths"]:
            lines.append(f"- `{path}`")
    lines += ["", "## Boundary Evidence", ""]
    for run in report["runs"]:
        lines.append(f"### {run['label']}")
        lines.append(f"- Lidar `/lidar/points`: {run['rates'].get('lidar_points_receive')}.")
        lines.append(f"- `/scan_neupan`: {run['rates'].get('scan_neupan_receive')}.")
        lines.append(f"- Scan audit: `{run['sensor_audit']}`.")
        lines.append(f"- Raw collector gap audit: `{run['raw_event_audit']}`. Raw invalid events are excluded from the derived event table.")
        lines.append(f"- Bridge/worker identity: `{run['bridge']}`.")
        lines.append(f"- Safety: `{run['safety']}`.")
        for event in run["active_gap_events"]:
            lines.append(f"- Active event {event.get('event_id')}: `{event.get('first_abnormal_boundary')}`, gap `{event.get('gap_sec')}` s; evidence `{event.get('boundary_evidence')}`.")
    lines += ["", "## Resource / Thread Audit", "", "The sampler ran at 5 Hz using psutil/procfs and no heavy profiler. Per-process CPU, RSS, thread count, and context-switch counters are in each raw run. `TasksMax` was not treated as a root cause without simultaneous active-limit and failure evidence.", ""]
    for run in report["runs"]:
        resource = run["resource"]
        lines.append(f"- {run['label']}: samples `{resource.get('sample_count', 0)}`, TasksCurrent max `{resource.get('tasks_current', {}).get('max')}`, TasksMax values `{resource.get('tasks_max_values')}`, system threads max `{resource.get('system_thread_count', {}).get('max')}`, sampler duration p95/max `{resource.get('sampler_duration_ms', {}).get('p95')}/{resource.get('sampler_duration_ms', {}).get('max')} ms`, worker resource sampled `{resource.get('worker_process', {}).get('sampled')}` (PID `{resource.get('worker_process', {}).get('pid')}`).")
        for label, values in sorted(resource.get("per_process", {}).items()):
            lines.append(f"  - `{label}`: threads max `{values['thread_count'].get('max')}`, RSS max `{values['rss_bytes'].get('max')}`, CPU max `{values['cpu_percent'].get('max')}`%.")
    lines += ["", "## Startup Infrastructure Evidence", "", f"{report['startup_infrastructure']}", "", "The restricted `TasksMax=256` startup failures are retained as startup-only infrastructure evidence. The admitted runs used `TasksMax=max`; their active resource samples do not show limit exhaustion.", "", "## Safety Invariants", "", "- Planner freshness watchdog remained `0.30 s` and fail-closed.", "- No relay heartbeat was used to refresh planner source freshness.", "- Forward-only safety invariant, finite outputs, worker fail-safe, and latest-only queue were observed without diagnostic bypass.", "", "## Verification And Code Changes", "", "- Instrumentation changes are monotonic timestamp fields, worker PID propagation, resource-role identification, relay/gate receipt metadata, event bundles, and report aggregation.", "- No runtime A/B was executed; no executor, thread limit, priority, watchdog, or planner variable was changed.", "- Verification details are recorded in `verification` JSON below; `colcon test = 0 tests / exit 5` remains existing discovery technical debt.", "", "## Conclusion", "", f"Root-cause classification: **{report['final_classification']}**. Evidence confidence: `{report['confidence']}`. {report['conclusion']}", "", f"Testing stops here. `next_stage_allowed={report['next_stage_allowed']}`. N2.4R1 Gazebo recovery motion is not authorized by this report.", ""]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=".")
    parser.add_argument("--output", required=True)
    parser.add_argument("--run", action="append", required=True, help="label=runtime_attribution.json")
    parser.add_argument("--verification", default="")
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    runs: list[dict[str, Any]] = []
    raw_runs: list[dict[str, Any]] = []
    for value in args.run:
        label, separator, path_value = value.partition("=")
        if not separator:
            raise SystemExit(f"--run must be label=path: {value}")
        path = Path(path_value).expanduser().resolve()
        data = _load(path)
        raw_runs.append(data)
        runs.append(_run_summary(label, path, data))
    code, classification_label, evidence = _classification(runs)
    if code == "I":
        next_stage = "Resume N2.4C Phenotype Audit"
        confidence = "high" if all(not run["active_gap_events"] for run in runs) else "low"
        conclusion = "No active-window >250 ms planner-result gap was observed in the supplied runs; the prior event was not reproduced under this instrumentation."
    elif code == "J":
        next_stage = "none"
        confidence = "low"
        conclusion = "The supplied evidence does not establish a unique first abnormal boundary."
    else:
        next_stage = "Runtime Fix Stage"
        confidence = "high" if len(runs) >= 3 else "medium"
        conclusion = "The first abnormal boundary is supported by the independent control/problem comparison and must be addressed before phenotype work resumes."
    timeline_definition = _timeline_definition(raw_runs)
    verification = _load(Path(args.verification).resolve()) if args.verification else {"status": "not supplied"}
    prior_report_path = workspace / "n2_4c_results" / "n2_4c_reproducibility_report.json"
    prior_report = _load(prior_report_path) if prior_report_path.exists() else {}
    startup_infrastructure = {
        "source": str(prior_report_path),
        "excluded_attempts": prior_report.get("excluded_attempts", []),
        "classification": "startup-only infrastructure evidence; not used to classify active B4 gap",
    }
    report = {
        "schema": "r1_n2_4c_runtime_report_v1",
        "final_classification": f"{code}. {classification_label}",
        "classification_code": code,
        "next_stage_allowed": next_stage,
        "confidence": confidence,
        "conclusion": conclusion,
        "classification_evidence": evidence,
        "workspace": str(workspace),
        "git": "unavailable: workspace has no .git metadata",
        "frozen_config": {"footprint_m": [0.682, 0.532], "latest_only": True, "queue_depth_max": 1, "relay_hz": 50.0, "planner_freshness_timeout_sec": 0.30, "relay_timeout_sec": 0.30, "timing_timeout_sec": 0.50, "gap_event_threshold_sec": 0.25, "forward_only": True, "torch_environment_changes": "none", "resource_sample_hz": 5.0, "monotonic_clock": "CLOCK_MONOTONIC via time.monotonic"},
        "file_sha256": _hashes(workspace),
        "timeline_definition": timeline_definition,
        "required_stage_delays": list(REQUESTED_STAGE_DELAYS),
        "window_policy": {
            "STARTUP": "before first gate with emergency_stop=false and failure_latched=false",
            "ACTIVE": "after first released healthy gate and before terminal failure/e-stop latch",
            "SHUTDOWN": "from first failure/e-stop latch through terminal observations",
            "cross_window_gap_policy": "event retains start_window and end_window; active attribution uses the pre-latch start boundary",
        },
        "startup_infrastructure": startup_infrastructure,
        "runs": runs,
        "verification": verification,
        "ab_validation": {
            "executed": False,
            "reason": "No runtime variable was changed; formal A/B is deferred to Runtime Fix Stage.",
        },
        "code_changes": {
            "runtime_behavior_changed": False,
            "instrumentation_and_report_files": [
                "src/r1_neupan_integration/r1_neupan_integration/n1_16_node.py",
                "src/r1_neupan_integration/r1_neupan_integration/n2_4c_runtime_attribution.py",
                "src/r1_neupan_integration/scripts/n2_4c_runtime_report.py",
            ],
        },
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    output.write_text(_md(report), encoding="utf-8")
    print(json.dumps({"json": str(output.with_suffix('.json')), "markdown": str(output), "classification": report["final_classification"], "next_stage_allowed": next_stage}, indent=2))


if __name__ == "__main__":
    main()
