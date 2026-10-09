#!/usr/bin/env python3
"""Unit checks for the optimized-path deviation guard."""

import numpy as np

from r1_neupan_integration.path_validation import (
    TemporalPathDeviationGuard,
    max_path_deviation_m,
    near_term_path_deviation_m,
)


def main() -> None:
    path = np.asarray([[0.0, 0.5, 1.0], [0.0, 0.0, 0.0]])
    close_plan = np.asarray([[0.0, 0.5, 1.0], [0.02, -0.04, 0.01]])
    diverged_plan = np.asarray([[0.0, 0.5, 1.0], [0.02, 0.18, 0.01]])
    assert max_path_deviation_m(close_plan, path) < 0.15
    assert max_path_deviation_m(diverged_plan, path) > 0.15
    far_horizon_diverged = np.asarray([
        [0.0, 0.5, 1.0, 1.5],
        [0.01, -0.01, 0.02, 0.28],
    ])
    assert max_path_deviation_m(far_horizon_diverged, path) > 0.20
    assert near_term_path_deviation_m(far_horizon_diverged, path, 2) < 0.15
    guard = TemporalPathDeviationGuard(warn_m=0.15, stop_m=0.20, stop_cycles=3)
    far_only = guard.evaluate(near_term_path_deviation_m(far_horizon_diverged, path, 2))
    assert not far_only.blocked
    first = guard.evaluate(0.21)
    second = guard.evaluate(0.22)
    third = guard.evaluate(0.23)
    assert not first.blocked and not second.blocked and third.blocked
    assert third.reason == "sustained_near_term_deviation"
    assert np.isinf(max_path_deviation_m(np.empty((2, 0)), path))
    assert np.isinf(max_path_deviation_m(np.asarray([[0.0], [np.nan]]), path))
    print("path_validation_unit.py PASS")


if __name__ == "__main__":
    main()
