#!/usr/bin/env python3
"""Contract checks for the R1 base-frame LiDAR self-return filter."""

import numpy as np

from r1_neupan_integration.scan_self_filter import (
    inflate_obstacles_for_dynamic_footprint,
    inflate_obstacles_for_mesh_components,
    mesh_component_overlap_mask,
    mesh_components_from_flat,
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
    dynamic = np.asarray([
        [-0.341, -0.266], [0.341, -0.266], [0.800, 0.300],
        [0.341, 0.266], [-0.341, 0.266],
    ])
    adjusted, displacement = inflate_obstacles_for_dynamic_footprint(
        np.asarray([[1.20, 0.00], [0.00, 1.20]]), dynamic,
        base_front_m=0.341, base_rear_m=0.341,
        base_left_m=0.266, base_right_m=0.266,
        clearance_m=0.15,
    )
    check(displacement[0] > displacement[1],
          "an extended arm must enlarge forward obstacle clearance more than the base side")
    check(np.all(np.hypot(adjusted[0], adjusted[1]) < np.asarray([1.20, 1.20])),
          "configuration-space obstacle points must move toward the robot")

    # The arm projections are disjoint. Mesh-aware clearance must preserve a
    # wall's path-longitudinal position rather than radially pulling it into
    # the robot's forward centre.
    component_stream = [
        4, 0.65, 0.35, 0.85, 0.35, 0.85, 0.55, 0.65, 0.55,
        4, 0.65, -0.55, 0.85, -0.55, 0.85, -0.35, 0.65, -0.35,
    ]
    components = mesh_components_from_flat(component_stream)
    mesh_points = np.asarray([
        [0.26, 1.20, 0.75],
        [0.44, 0.00, 0.60],
    ])
    component_adjusted, component_displacement = inflate_obstacles_for_mesh_components(
        mesh_points, components,
        base_front_m=0.341, base_rear_m=0.341,
        base_left_m=0.266, base_right_m=0.266,
        clearance_m=0.15,
        path_tangent=np.asarray([1.0, 0.0]),
    )
    check(np.isclose(component_adjusted[0, 0], mesh_points[0, 0]),
          "a side wall must retain its longitudinal coordinate")
    check(component_adjusted[1, 0] > 0.10,
          "a side wall must not become a fictitious central near obstacle")
    check(np.isclose(component_displacement[1], 0.0),
          "a centreline obstacle must not inherit side-arm clearance")
    check(component_displacement[2] > 0.40 and component_adjusted[1, 2] < mesh_points[1, 2],
          "an obstacle beside a real arm projection must retain conservative lateral clearance")
    # A component that widens only distally must not pull a base-side wall
    # into the path, but its distal cross section must still be expanded.
    tapered_components = mesh_components_from_flat([
        4, 0.20, 0.30, 0.20, 0.40, 0.80, 1.00, 0.80, 0.40,
    ])
    tapered_points = np.asarray([[0.25, 0.75], [0.90, 1.10]])
    tapered_adjusted, tapered_displacement = inflate_obstacles_for_mesh_components(
        tapered_points, tapered_components,
        base_front_m=0.341, base_rear_m=0.341,
        base_left_m=0.266, base_right_m=0.266,
        clearance_m=0.15,
        path_tangent=np.asarray([1.0, 0.0]),
    )
    check(tapered_displacement[0] < 0.50 and tapered_adjusted[1, 0] > 0.40,
          "a distal-wide arm component must not pull a base-side wall into the path")
    check(tapered_displacement[1] > 0.70 and tapered_adjusted[1, 1] < 0.40,
          "a wall beside the real distal arm cross section must remain expanded")
    overlap = mesh_component_overlap_mask(
        np.asarray([[0.75, 0.20], [0.45, 0.00]]), components,
        margin_m=0.0,
    )
    check(overlap.tolist() == [True, False],
          "raw mesh overlap must be reported independently of planner remapping")
    try:
        inflate_obstacles_for_mesh_components(
            mesh_points, components,
            base_front_m=0.341, base_rear_m=0.341,
            base_left_m=0.266, base_right_m=0.266,
            clearance_m=0.15,
            path_tangent=np.asarray([0.0, 0.0]),
        )
    except ValueError:
        pass
    else:
        raise AssertionError("zero path tangent must be rejected")
    try:
        mesh_components_from_flat([4, 0.0, 0.0])
    except ValueError:
        pass
    else:
        raise AssertionError("truncated mesh component data must be rejected")
    print("N2 scan self-filter unit PASS")


if __name__ == "__main__":
    main()
