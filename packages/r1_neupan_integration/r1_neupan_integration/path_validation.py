"""Pure numeric checks for optimized NeuPAN paths."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


def max_path_deviation_m(opt_state: np.ndarray, path: np.ndarray) -> float:
    """Return the largest nearest-path distance over a finite optimized plan."""
    if opt_state.ndim != 2 or path.ndim != 2 or opt_state.shape[0] < 2 or path.shape[0] < 2:
        return float("inf")
    if opt_state.shape[1] == 0 or path.shape[1] == 0:
        return float("inf")
    optimized_xy = opt_state[:2, :].T
    path_xy = path[:2, :].T
    if not np.isfinite(optimized_xy).all() or not np.isfinite(path_xy).all():
        return float("inf")
    distances_sq = np.sum(
        (optimized_xy[:, None, :] - path_xy[None, :, :]) ** 2, axis=2
    )
    return float(np.sqrt(np.max(np.min(distances_sq, axis=1))))


def near_term_path_deviation_m(
    opt_state: np.ndarray, path: np.ndarray, state_count: int,
) -> float:
    """Return nearest-path error over only the command-reachable horizon start."""
    if state_count < 1:
        raise ValueError("state_count must be positive")
    if opt_state.ndim != 2 or path.ndim != 2 or opt_state.shape[0] < 2 or path.shape[0] < 2:
        return float("inf")
    if opt_state.shape[1] == 0 or path.shape[1] == 0:
        return float("inf")
    optimized_xy = opt_state[:2, :min(state_count, opt_state.shape[1])].T
    path_xy = path[:2, :].T
    if not np.isfinite(optimized_xy).all() or not np.isfinite(path_xy).all():
        return float("inf")
    distances_sq = np.sum(
        (optimized_xy[:, None, :] - path_xy[None, :, :]) ** 2, axis=2
    )
    return float(np.sqrt(np.max(np.min(distances_sq, axis=1))))


@dataclass(frozen=True)
class PathDeviationDecision:
    blocked: bool
    warning: bool
    violation_cycles: int
    reason: str


class TemporalPathDeviationGuard:
    """Stop only for sustained, near-term deviations from the route.

    The full prediction horizon is intentionally diagnostic-only: it often
    bends around an obstacle before the commanded base pose has left the path.
    """

    def __init__(self, warn_m: float, stop_m: float, stop_cycles: int) -> None:
        if not np.isfinite(warn_m) or not np.isfinite(stop_m) or warn_m <= 0.0:
            raise ValueError("path deviation thresholds must be finite and positive")
        if stop_m < warn_m:
            raise ValueError("path deviation stop threshold must be >= warning threshold")
        if stop_cycles < 1:
            raise ValueError("path deviation stop cycles must be positive")
        self.warn_m = float(warn_m)
        self.stop_m = float(stop_m)
        self.stop_cycles = int(stop_cycles)
        self.reset()

    def reset(self) -> None:
        self._violation_cycles = 0

    def evaluate(self, near_term_deviation_m: float) -> PathDeviationDecision:
        if not np.isfinite(near_term_deviation_m):
            self._violation_cycles = self.stop_cycles
            return PathDeviationDecision(True, True, self._violation_cycles, "invalid_near_term_path")
        warning = near_term_deviation_m > self.warn_m
        if near_term_deviation_m > self.stop_m:
            self._violation_cycles += 1
        else:
            self._violation_cycles = 0
        blocked = self._violation_cycles >= self.stop_cycles
        if blocked:
            reason = "sustained_near_term_deviation"
        elif warning:
            reason = "near_term_deviation_warning"
        else:
            reason = "within_near_term_limit"
        return PathDeviationDecision(blocked, warning, self._violation_cycles, reason)
