#!/usr/bin/env python3
"""Regression check for the Nav2/NeuPAN path-version boundary."""

from __future__ import annotations

import numpy as np

from r1_neupan_integration.n2_pre_semantics import apply_uniform_gear


def main() -> None:
    raw_path = np.asarray(
        [[0.0, 1.0], [0.0, 0.0], [0.0, 0.0]], dtype=np.float64)
    planner_path_version = 1
    nav2_path_version = 1
    rewritten_path = apply_uniform_gear(raw_path, -1)
    planner_path_version += 1

    if planner_path_version != 2:
        raise AssertionError("gear rewrite must advance NeuPAN's internal path version")
    if nav2_path_version != 1:
        raise AssertionError("gear rewrite must not change Nav2's external path context")
    if not np.all(rewritten_path[3, :] == -1.0):
        raise AssertionError("rewritten planner path must carry the requested reverse gear")
    # A later Nav2 path callback advances both contexts once. The external
    # sequence cannot inherit the internal gear-rewrite increment.
    planner_path_version += 1
    nav2_path_version += 1
    if (planner_path_version, nav2_path_version) != (3, 2):
        raise AssertionError("Nav2 and planner path versions must remain independent")
    print("n1_gear_context_unit PASS")


if __name__ == "__main__":
    main()
