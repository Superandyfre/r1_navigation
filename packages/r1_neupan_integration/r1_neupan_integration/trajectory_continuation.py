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

    def validate(self, *, context, scan_counter, scan_received_monotonic,
                 pose_received_monotonic, now, input_timeout, state, points):
        with self._lock:
            plan = self._plan
            if plan is None or context != plan[0]:
                return None
            if (type(scan_counter) is not int or scan_counter <= self._last_scan or
                    not all(math.isfinite(x) for x in (scan_received_monotonic,
                        pose_received_monotonic, now, input_timeout)) or input_timeout <= 0 or
                    scan_received_monotonic <= plan[2] or pose_received_monotonic <= 0 or
                    scan_received_monotonic > now or pose_received_monotonic > now or
                    now - scan_received_monotonic > input_timeout or
                    now - pose_received_monotonic > input_timeout or now < plan[2]):
                return None
            # Failed revalidation cannot be replayed under the same scan identity.
            self._last_scan = scan_counter
        started = self.clock()
        result = self.planner.validate_remaining_trajectory(
            state, plan[3], now - plan[2], points)
        finished = now + max(0.0, self.clock() - started)
        with self._lock:
            # Cancellation, replacement or a newer scan invalidates in-flight
            # results. The publisher must recheck authority at its own boundary.
            if self._plan is not plan or self._last_scan != scan_counter:
                return None
        # Validation time cannot refresh input timestamps or silently select a
        # different trajectory knot. Recompute on the next scan instead.
        if (result is None or finished - scan_received_monotonic > input_timeout or
                finished - pose_received_monotonic > input_timeout or
                int((finished - plan[2]) / self.planner.dt) !=
                int((now - plan[2]) / self.planner.dt)):
            return None
        action, minimum = result
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.size != 2 or not np.isfinite(action).all():
            raise ValueError("invalid_validated_continuation")
        if minimum is not None and not math.isfinite(minimum):
            raise ValueError("invalid_continuation_distance")
        return Continuation(context, plan[1], scan_counter, plan[2],
                            float(scan_received_monotonic), float(finished),
                            float(action[0]), float(action[1]), minimum)
