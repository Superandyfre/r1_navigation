"""Pure N2.3 planner-result freshness and command-relay contract.

The envelope is carried on an integration-local ``std_msgs/String`` topic so
the standard ``geometry_msgs/Twist`` message is unchanged.  Relay fields may
change on every publication; source fields must remain byte-for-byte stable
for repeated publications of one planner result.
"""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Any


ENVELOPE_SCHEMA = "r1_n2_3_command_envelope_v1"
CONTEXT_SCHEMA = "r1_n2_3_planner_context_v1"
SOURCE_FIELDS = (
    "worker_generation", "request_id", "cycle_id", "scan_counter",
    "planner_result_monotonic", "path_version", "goal_version",
)
RELAY_FIELDS = ("relay_sequence", "relay_publish_monotonic")


class ContractError(ValueError):
    """Malformed or unsafe command contract payload."""


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def make_planner_envelope(
    *,
    linear_x: float,
    angular_z: float,
    worker_generation: int,
    request_id: int,
    cycle_id: int,
    scan_counter: int,
    planner_result_monotonic: float,
    path_version: int,
    goal_version: int,
) -> dict[str, Any]:
    payload = {
        "schema": ENVELOPE_SCHEMA,
        "command": {"linear_x": float(linear_x), "angular_z": float(angular_z)},
        "worker_generation": int(worker_generation),
        "request_id": int(request_id),
        "cycle_id": int(cycle_id),
        "scan_counter": int(scan_counter),
        "planner_result_monotonic": float(planner_result_monotonic),
        "path_version": int(path_version),
        "goal_version": int(goal_version),
        "finite": True,
        "relay_sequence": 0,
        "relay_publish_monotonic": 0.0,
    }
    validate_envelope(payload)
    return payload


def validate_envelope(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict) or payload.get("schema") != ENVELOPE_SCHEMA:
        raise ContractError("envelope_schema_mismatch")
    command = payload.get("command")
    if not isinstance(command, dict) or not _finite(command.get("linear_x")) or not _finite(command.get("angular_z")):
        raise ContractError("nonfinite_command")
    if payload.get("finite") is not True:
        raise ContractError("nonfinite_source")
    for field in SOURCE_FIELDS:
        if field not in payload:
            raise ContractError("missing_" + field)
    for field in ("worker_generation", "request_id", "cycle_id", "scan_counter", "path_version", "goal_version"):
        try:
            if int(payload[field]) != payload[field]:
                raise ContractError("noninteger_" + field)
        except (TypeError, ValueError):
            raise ContractError("noninteger_" + field) from None
    if not _finite(payload["planner_result_monotonic"]) or float(payload["planner_result_monotonic"]) <= 0.0:
        raise ContractError("invalid_planner_result_timestamp")
    return payload


def relay_envelope(source: dict[str, Any], sequence: int, publish_monotonic: float) -> dict[str, Any]:
    validate_envelope(source)
    if not _finite(publish_monotonic) or float(publish_monotonic) <= 0.0:
        raise ContractError("invalid_relay_timestamp")
    result = deepcopy(source)
    result["relay_sequence"] = int(sequence)
    result["relay_publish_monotonic"] = float(publish_monotonic)
    validate_envelope(result)
    return result


def make_context(path_version: int, goal_version: int) -> dict[str, Any]:
    return {
        "schema": CONTEXT_SCHEMA,
        "path_version": int(path_version),
        "goal_version": int(goal_version),
    }


class FreshnessTracker:
    """Validate source identity while allowing exact-result relay repeats."""

    def __init__(self) -> None:
        self.worker_generation: int | None = None
        self.path_version: int | None = None
        self.goal_version: int | None = None
        self.request_id: int | None = None
        self.cycle_id: int | None = None
        self.scan_counter: int | None = None
        self.planner_result_monotonic: float | None = None
        self.command: tuple[float, float] | None = None
        self.expected_path_version: int | None = None
        self.expected_goal_version: int | None = None
        self.fault: str | None = None

    def set_context(self, payload: Any) -> str:
        if not isinstance(payload, dict) or payload.get("schema") != CONTEXT_SCHEMA:
            self.fault = "context_schema_mismatch"
            return self.fault
        try:
            path_version = int(payload["path_version"])
            goal_version = int(payload["goal_version"])
        except (KeyError, TypeError, ValueError):
            self.fault = "context_version_malformed"
            return self.fault
        if self.expected_path_version is not None and path_version < self.expected_path_version:
            self.fault = "path_version_regressed"
        if self.expected_goal_version is not None and goal_version < self.expected_goal_version:
            self.fault = "goal_version_regressed"
        self.expected_path_version = path_version
        self.expected_goal_version = goal_version
        return "context_updated"

    def accept(self, payload: Any) -> tuple[bool, str]:
        try:
            validate_envelope(payload)
        except ContractError as exc:
            self.fault = str(exc)
            return False, self.fault
        source = payload
        generation = int(source["worker_generation"])
        path_version = int(source["path_version"])
        goal_version = int(source["goal_version"])
        request_id = int(source["request_id"])
        cycle_id = int(source["cycle_id"])
        scan_counter = int(source["scan_counter"])
        planner_result_monotonic = float(source["planner_result_monotonic"])
        command = (float(source["command"]["linear_x"]),
                   float(source["command"]["angular_z"]))
        if self.worker_generation is None:
            self.worker_generation = generation
        elif generation != self.worker_generation:
            self.fault = "generation_mismatch"
            return False, self.fault
        if self.expected_path_version is not None and path_version != self.expected_path_version:
            self.fault = "path_version_mismatch"
            return False, self.fault
        if self.expected_goal_version is not None and goal_version != self.expected_goal_version:
            self.fault = "goal_version_mismatch"
            return False, self.fault
        identity = (generation, request_id, cycle_id, scan_counter, planner_result_monotonic,
                    path_version, goal_version)
        previous = None if self.request_id is None else (
            self.worker_generation, self.request_id, self.cycle_id,
            self.scan_counter, self.planner_result_monotonic,
            self.path_version, self.goal_version,
        )
        if previous is not None:
            if identity == previous:
                if command != self.command:
                    self.fault = "duplicate_command_mismatch"
                    return False, self.fault
                return True, "duplicate_source_result"
            if generation != previous[0]:
                self.fault = "generation_mismatch"
                return False, self.fault
            if request_id < previous[1] or cycle_id < previous[2] or scan_counter < previous[3]:
                self.fault = "stale_result"
                return False, self.fault
            if request_id == previous[1] or cycle_id == previous[2] or scan_counter == previous[3]:
                self.fault = "source_identity_mismatch"
                return False, self.fault
            if planner_result_monotonic <= previous[4]:
                self.fault = "source_timestamp_regressed"
                return False, self.fault
        self.request_id = request_id
        self.cycle_id = cycle_id
        self.scan_counter = scan_counter
        self.planner_result_monotonic = planner_result_monotonic
        self.path_version = path_version
        self.goal_version = goal_version
        self.command = command
        self.fault = None
        return True, "new_source_result"
