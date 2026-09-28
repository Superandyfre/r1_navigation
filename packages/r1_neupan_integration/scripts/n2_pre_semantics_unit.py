#!/usr/bin/env python3
"""Deterministic unit checks for the R1 N2-PRE semantic adapter."""

import numpy as np

from r1_neupan_integration.n2_pre_semantics import (
    apply_uniform_gear,
    collapse_invalid_stationary_prefix,
    navigation_loop,
    normalize_path_matrix,
    transition_requires_zero,
    validate_gear,
)


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    geometry = np.asarray([[0.0, 0.5, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    forward = normalize_path_matrix(geometry)
    reverse = apply_uniform_gear(geometry, -1)
    check(forward.shape == (4, 3), "path must be materialized as 4xN")
    check(np.all(forward[3] == 1.0), "default path gear must be +1")
    check(np.all(reverse[:3] == geometry) and np.all(reverse[3] == -1.0),
          "reverse gear must not change geometry")
    check(validate_gear(-1) == -1 and validate_gear(1) == 1, "gear validation")
    check(transition_requires_zero(1, -1), "gear changes require a zero command")
    check(not transition_requires_zero(-1, -1), "same gear must not transition")
    mixed = np.vstack([geometry, [1.0, 1.0, -1.0]])
    check(normalize_path_matrix(mixed)[3, -1] == -1.0,
          "one explicit gear transition is allowed")
    try:
        normalize_path_matrix(np.vstack([geometry, [1.0, -1.0, 1.0]]))
    except ValueError:
        pass
    else:
        raise AssertionError("multiple arbitrary gear transitions must be rejected")
    check(navigation_loop("cyclic") and not navigation_loop("terminal"),
          "navigation mode loop mapping")
    invalid_prefix = np.asarray([
        [3.125, 3.125, 3.125, 3.108, 3.060],
        [3.325, 3.325, 3.325, 3.370, 3.454],
        [0.0, 0.0, 0.0, 1.976, 2.264],
        [1.0, 1.0, 1.0, 1.0, 1.0],
    ])
    collapsed = collapse_invalid_stationary_prefix(invalid_prefix)
    check(collapsed.shape == (4, 3), "inconsistent stationary prefix must collapse")
    check(abs(collapsed[2, 0] - np.arctan2(0.045, -0.017)) < 1.0e-9,
          "collapsed first pose must use the first geometric tangent")
    in_place_rotation = invalid_prefix.copy()
    in_place_rotation[2, :3] = [0.0, 0.5, 1.0]
    check(collapse_invalid_stationary_prefix(in_place_rotation).shape == in_place_rotation.shape,
          "genuine in-place rotations must be retained")
    print("N2-PRE semantic unit PASS: explicit gear, transition policy, terminal/cyclic mode")


if __name__ == "__main__":
    main()
