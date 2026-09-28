#!/usr/bin/env python3
"""Contract checks for the R1 base-frame LiDAR self-return filter."""

import numpy as np

from r1_neupan_integration.scan_self_filter import (
    outside_polygon_footprint,
    outside_rectangular_footprint,
)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    ranges = np.asarray([0.15, 0.40, 0.50, 0.45])
    angles = np.asarray([0.0, 0.0, np.pi / 2.0, np.pi])
    keep = outside_rectangular_footprint(
        ranges, angles, front_m=0.341, rear_m=0.341,
        left_m=0.266, right_m=0.266,
    )
    check(keep.tolist() == [False, True, True, True],
          "only the return inside the chassis footprint may be removed")
    try:
        outside_rectangular_footprint(
            ranges, angles, front_m=0.0, rear_m=0.341,
            left_m=0.266, right_m=0.266,
        )
    except ValueError:
        pass
    else:
        raise AssertionError("invalid footprint extents must be rejected")
    polygon = np.asarray([
        [-0.371, -0.296], [0.150, -0.296], [0.150, -0.340],
        [0.450, -0.360], [0.450, -0.296], [0.371, -0.296],
        [0.371, 0.296], [-0.371, 0.296],
    ])
    points = np.asarray([
        [0.218, 0.600, -0.200],
        [-0.299, -0.300, 0.000],
    ])
    keep = outside_polygon_footprint(points, polygon)
    check(keep.tolist() == [False, True, False],
          "the forward-right self return must be removed without masking an exterior point")
    print("N2 scan self-filter unit PASS")


if __name__ == "__main__":
    main()
