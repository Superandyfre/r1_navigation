"""Predict the planner start pose from time-stamped TF and measured odometry.

This does not alter obstacle observations or issue actuator commands.
"""
from collections import deque
import math
import numpy as np


class SolveLatency:
    """Recent successful request-to-result durations, bounded by input validity."""
    def __init__(self, validity_sec, window=30):
        if not math.isfinite(validity_sec) or validity_sec <= 0:
            raise ValueError("invalid validity")
        self.validity_sec = float(validity_sec)
        self.samples = deque(maxlen=window)

    def observe(self, duration):
        if math.isfinite(duration) and duration >= 0:
            self.samples.append(min(float(duration), self.validity_sec))

    def estimate(self):
        # Until a request completes, do not invent a latency measurement.
        if not self.samples:
            return 0.0
        return min(float(np.quantile(self.samples, .75)), self.validity_sec)


def predict_pose(state, velocity, pose_age, solve_latency, validity_sec):
    """Constant measured twist in map frame; preserve obstacle points exactly.

    pose_age is derived from the TF header, not callback receive time. If TF
    or odometry is too old, callers leave the pose unpredicted and retain the
    existing input-freshness/authorization behavior.
    """
    pose = np.asarray(state, dtype=np.float64).reshape(3, 1).copy()
    if velocity is None:
        return pose, 0.0
    twist = np.asarray(velocity, dtype=np.float64).reshape(2)
    if (not np.isfinite(pose).all() or not np.isfinite(twist).all() or
            not all(math.isfinite(v) for v in (pose_age, solve_latency, validity_sec)) or
            validity_sec <= 0 or pose_age < 0 or pose_age > validity_sec or solve_latency < 0):
        return pose, 0.0
    dt = min(pose_age + solve_latency, validity_sec)
    v, w = twist
    yaw = float(pose[2, 0])
    if abs(w) < 1e-8:
        pose[0, 0] += v * math.cos(yaw) * dt
        pose[1, 0] += v * math.sin(yaw) * dt
    else:
        pose[0, 0] += v / w * (math.sin(yaw + w * dt) - math.sin(yaw))
        pose[1, 0] += v / w * (math.cos(yaw) - math.cos(yaw + w * dt))
    pose[2, 0] = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
    return pose, dt
