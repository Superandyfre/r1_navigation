"""State and provenance for revalidating an existing NeuPAN control horizon.

No ROS and no publication: the caller owns worker isolation, fresh scan/pose
validation, command authority, cancellation and the final actuator boundary.
"""
from dataclasses import dataclass
import math
import time
import threading
import numpy as np

@dataclass(frozen=True)
class Continuation:
    context: tuple
    plan_request_id: int
    validation_scan_counter: int
    plan_started_monotonic: float
    scan_received_monotonic: float
    pose_received_monotonic: float
    validation_started_monotonic: float
    validated_monotonic: float
    linear_x: float
    angular_z: float
    minimum_distance_m: float | None

class TrajectoryContinuation:
    """Keep original plan time fixed and demand a strictly newer scan."""
    def __init__(self, planner, clock=time.monotonic):
        self.planner = planner
        self.clock = clock
        self._lock = threading.Lock()
        self.clear()

    def clear(self):
        with self._lock:
            self._plan = None
            self._last_scan = -1

    def install(self, *, context, request_id, scan_counter, started_monotonic, controls):
        a = np.asarray(controls, dtype=np.float64)
        if (not isinstance(context, tuple) or not context or
                type(request_id) is not int or request_id < 1 or
                type(scan_counter) is not int or scan_counter < 1 or
                not math.isfinite(started_monotonic) or started_monotonic <= 0 or
                a.ndim != 2 or a.shape[0] != 2 or a.shape[1] < 1 or
                not np.isfinite(a).all()):
            raise ValueError("invalid_continuation_plan")
        with self._lock:
            self._plan = (context, request_id, float(started_monotonic), a.copy())
            self._last_scan = scan_counter

    def validate(self, **kwargs):
        return self.validate_detailed(**kwargs)[0]

    def validate_detailed(self, *, context, scan_counter, scan_received_monotonic,
                          pose_received_monotonic, now, input_timeout, state, points):
        with self._lock:
            plan = self._plan
            if plan is None or context != plan[0]:
                return None, "unavailable"
            if (type(scan_counter) is not int or scan_counter <= self._last_scan or
                    not all(math.isfinite(x) for x in (scan_received_monotonic,
                        pose_received_monotonic, now, input_timeout)) or input_timeout <= 0 or
                    scan_received_monotonic <= plan[2] or pose_received_monotonic <= 0 or
                    scan_received_monotonic > now or pose_received_monotonic > now or
                    now - scan_received_monotonic > input_timeout or
                    now - pose_received_monotonic > input_timeout or now < plan[2]):
                return None, "unavailable"
            # Failed revalidation cannot be replayed under the same scan identity.
            self._last_scan = scan_counter
        started = self.clock()
        # A certificate cannot outlive its oldest observation. Checking past
        # that instant rejects future motion which the certificate can never
        # authorize and causes unnecessary stops while the solver replans.
        duration = min(scan_received_monotonic, pose_received_monotonic) + input_timeout - now
        if duration <= 0.:
            return None, "unavailable"
        result, reason = self.planner.validate_remaining_trajectory(
            state, plan[3], now - plan[2], points, with_reason=True, duration_sec=duration)
        finished = now + max(0.0, self.clock() - started)
        with self._lock:
            # Cancellation, replacement or a newer scan invalidates in-flight
            # results. The publisher must recheck authority at its own boundary.
            if self._plan is not plan or self._last_scan != scan_counter:
                return None, "unavailable"
        # Validation time cannot refresh input timestamps or extend the
        # original trajectory horizon; proofs cover their entire freshness period.
        if result is None:
            return None, reason
        if (finished - scan_received_monotonic > input_timeout or
                finished - pose_received_monotonic > input_timeout or
                finished - plan[2] >= plan[3].shape[1] * self.planner.dt):
            return None, "unavailable"
        action, minimum = result
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.size != 2 or not np.isfinite(action).all():
            raise ValueError("invalid_validated_continuation")
        if minimum is not None and not math.isfinite(minimum):
            raise ValueError("invalid_continuation_distance")
        return (Continuation(context, plan[1], scan_counter, plan[2],
                            float(scan_received_monotonic), float(pose_received_monotonic),
                            float(now), float(finished), float(action[0]), float(action[1]), minimum), "validated")


def make_trajectory_proof(parent_source, continuation, sequence, published_monotonic):
    """Describe a real, newly validated trajectory knot without changing plan time."""
    from .execution_protocol import validate_source
    if type(sequence) is not int or sequence < 1:
        raise ValueError("invalid_trajectory_proof_sequence")
    source = validate_source(parent_source, published_monotonic)
    fields = ("authority_boot_id", "controller_boot_id", "navigation_attempt_id",
              "worker_boot_id", "worker_generation", "path_version", "goal_version")
    context = tuple(source[k] for k in fields)
    if continuation.context != context or continuation.plan_request_id != source["request_id"]:
        raise ValueError("trajectory_proof_parent_mismatch")
    if (continuation.plan_started_monotonic != source["planner_result_monotonic"] or
            continuation.validation_scan_counter <= source["scan_counter"] or
            continuation.scan_received_monotonic <= continuation.plan_started_monotonic or
            not (continuation.scan_received_monotonic <= continuation.validation_started_monotonic <=
                 continuation.validated_monotonic <= published_monotonic) or
            continuation.pose_received_monotonic > continuation.validation_started_monotonic):
        raise ValueError("invalid_trajectory_proof_times")
    proof = {key:source[key] for key in fields}
    proof.update(schema="r1_neupan_trajectory_proof_v1", sequence=sequence,
        parent_request_id=source["request_id"], parent_cycle_id=source["cycle_id"],
        parent_scan_counter=source["scan_counter"], scan_counter=continuation.validation_scan_counter,
        plan_started_monotonic=continuation.plan_started_monotonic,
        scan_receive_monotonic=continuation.scan_received_monotonic,
        pose_receive_monotonic=continuation.pose_received_monotonic,
        validation_started_monotonic=continuation.validation_started_monotonic,
        validated_monotonic=continuation.validated_monotonic,
        published_monotonic=float(published_monotonic), valid=True,
        command=dict(linear_x=continuation.linear_x, angular_z=continuation.angular_z))
    return proof


def make_trajectory_rejection(parent_source, event, sequence, published_monotonic):
    """Revoke a previously validated continuation after a newer failed scan."""
    from .execution_protocol import validate_source
    source = validate_source(parent_source, published_monotonic)
    if (type(sequence) is not int or sequence < 1 or event.get("result") is not None or
            type(event.get("scan_counter")) is not int or event["scan_counter"] <= source["scan_counter"]):
        raise ValueError("invalid_trajectory_rejection")
    keys = ("scan_receive_monotonic", "pose_receive_monotonic",
            "validation_started_monotonic", "validated_monotonic")
    if any(type(event.get(k)) not in (int, float) or not math.isfinite(event[k]) for k in keys):
        raise ValueError("invalid_trajectory_rejection_times")
    scan, pose, started, done = (float(event[k]) for k in keys)
    if not (source["planner_result_monotonic"] < scan <= started <= done <= published_monotonic and
            0 < pose <= started):
        raise ValueError("invalid_trajectory_rejection_times")
    fields = ("authority_boot_id", "controller_boot_id", "navigation_attempt_id",
              "worker_boot_id", "worker_generation", "path_version", "goal_version")
    proof = {key:source[key] for key in fields}
    proof.update(schema="r1_neupan_trajectory_proof_v1", sequence=sequence,
        parent_request_id=source["request_id"], parent_cycle_id=source["cycle_id"],
        parent_scan_counter=source["scan_counter"], scan_counter=event["scan_counter"],
        plan_started_monotonic=source["planner_result_monotonic"],
        scan_receive_monotonic=scan, pose_receive_monotonic=pose,
        validation_started_monotonic=started, validated_monotonic=done,
        published_monotonic=float(published_monotonic), valid=False)
    return proof
