"""R1-specific removal of LiDAR returns from inside the base footprint."""

from __future__ import annotations

import numpy as np


def outside_rectangular_footprint(
    ranges: np.ndarray,
    angles: np.ndarray,
    *,
    front_m: float,
    rear_m: float,
    left_m: float,
    right_m: float,
) -> np.ndarray:
    """Return a mask retaining only points outside a base-frame footprint."""
    values = (front_m, rear_m, left_m, right_m)
    if not all(np.isfinite(values)) or min(values) <= 0.0:
        raise ValueError("footprint extents must be finite positive distances")
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    if ranges.shape != angles.shape:
        raise ValueError("ranges and angles must have matching shapes")
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    inside = (
        (x >= -rear_m) & (x <= front_m)
        & (y >= -right_m) & (y <= left_m)
    )
    return ~inside


def polygon_vertices_from_flat_xy(values) -> np.ndarray:
    """Validate a ROS double-array footprint parameter and return Nx2 vertices."""
    vertices = np.asarray(values, dtype=np.float64).reshape(-1)
    if vertices.size < 6 or vertices.size % 2:
        raise ValueError("self-filter polygon must contain at least three x/y pairs")
    vertices = vertices.reshape(-1, 2)
    if not np.isfinite(vertices).all():
        raise ValueError("self-filter polygon vertices must be finite")
    area_twice = np.dot(vertices[:, 0], np.roll(vertices[:, 1], -1)) - np.dot(
        vertices[:, 1], np.roll(vertices[:, 0], -1)
    )
    if abs(float(area_twice)) <= 1.0e-9:
        raise ValueError("self-filter polygon must enclose a nonzero area")
    return vertices


def outside_polygon_footprint(
    points: np.ndarray,
    vertices: np.ndarray,
    *,
    margin_m: float = 0.0,
) -> np.ndarray:
    """Retain base-frame points outside a polygon and its outward margin."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] != 2:
        raise ValueError("points must have shape (2, N)")
    vertices = polygon_vertices_from_flat_xy(vertices)
    margin_m = float(margin_m)
    if not np.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("self-filter polygon margin must be finite and nonnegative")
    if points.shape[1] == 0:
        return np.empty(0, dtype=bool)

    x, y = points[0], points[1]
    inside = np.zeros(points.shape[1], dtype=bool)
    distance_sq = np.full(points.shape[1], np.inf, dtype=np.float64)
    for start, end in zip(vertices, np.roll(vertices, -1, axis=0)):
        x0, y0 = start
        x1, y1 = end
        crosses = (y0 > y) != (y1 > y)
        x_at_y = (x1 - x0) * (y - y0) / ((y1 - y0) if y1 != y0 else 1.0) + x0
        inside ^= crosses & (x < x_at_y)

        edge = end - start
        edge_norm_sq = float(np.dot(edge, edge))
        projection = np.clip(
            ((x - x0) * edge[0] + (y - y0) * edge[1]) / edge_norm_sq,
            0.0,
            1.0,
        )
        nearest_x = x0 + projection * edge[0]
        nearest_y = y0 + projection * edge[1]
        distance_sq = np.minimum(distance_sq, (x - nearest_x) ** 2 + (y - nearest_y) ** 2)
    return ~(inside | (distance_sq <= margin_m ** 2))


def inflate_obstacles_for_dynamic_footprint(
    points: np.ndarray,
    footprint_vertices: np.ndarray,
    *,
    base_front_m: float,
    base_rear_m: float,
    base_left_m: float,
    base_right_m: float,
    clearance_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Map local obstacle points into a conservative live-arm configuration space.

    NeuPAN already models the fixed chassis rectangle.  Each scan point is
    therefore shifted only by the directional reach of the live footprint
    beyond that rectangle, plus the requested local clearance.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] != 2:
        raise ValueError("points must have shape (2, N)")
    vertices = polygon_vertices_from_flat_xy(footprint_vertices)
    extents = (base_front_m, base_rear_m, base_left_m, base_right_m)
    if not all(np.isfinite(extents)) or min(extents) <= 0.0:
        raise ValueError("base footprint extents must be finite positive distances")
    clearance_m = float(clearance_m)
    if not np.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("clearance_m must be finite and nonnegative")
    if points.shape[1] == 0:
        return points.copy(), np.empty(0, dtype=np.float64)

    distances = np.hypot(points[0], points[1])
    unit_x = np.divide(points[0], distances, out=np.zeros_like(distances), where=distances > 1.0e-9)
    unit_y = np.divide(points[1], distances, out=np.zeros_like(distances), where=distances > 1.0e-9)
    dynamic_support = np.max(
        vertices[:, 0:1] * unit_x[None, :] + vertices[:, 1:2] * unit_y[None, :], axis=0)
    base_x = np.where(unit_x >= 0.0, float(base_front_m), float(base_rear_m))
    base_y = np.where(unit_y >= 0.0, float(base_left_m), float(base_right_m))
    base_support = np.abs(unit_x) * base_x + np.abs(unit_y) * base_y
    displacement = np.maximum(0.0, dynamic_support - base_support) + clearance_m
    adjusted_range = np.maximum(0.01, distances - displacement)
    adjusted = np.vstack([unit_x * adjusted_range, unit_y * adjusted_range])
    return adjusted, displacement


def mesh_components_from_flat(values) -> list[np.ndarray]:
    """Decode the r1_arm_mesh_components_v1 variable-size polygon stream."""
    data = np.asarray(values, dtype=np.float64).reshape(-1)
    if data.size == 0 or not np.isfinite(data).all():
        raise ValueError("mesh component stream must be finite and nonempty")
    components: list[np.ndarray] = []
    offset = 0
    while offset < data.size:
        count_value = data[offset]
        count = int(round(float(count_value)))
        if count < 3 or abs(count_value - count) > 1.0e-9:
            raise ValueError("mesh component vertex count must be an integer of at least three")
        end = offset + 1 + 2 * count
        if end > data.size:
            raise ValueError("mesh component stream is truncated")
        components.append(polygon_vertices_from_flat_xy(data[offset + 1:end]))
        offset = end
    if offset != data.size:
        raise ValueError("mesh component stream has trailing data")
    return components


def _ray_exit_distance(vertices: np.ndarray, unit_x: np.ndarray,
                       unit_y: np.ndarray) -> np.ndarray:
    """Return each ray's far intersection with one convex mesh projection.

    A component that does not intersect a ray returns zero.  Mesh components
    are individually convex hulls, so crossing every edge yields the exact
    entry/exit interval for this radial projection.
    """
    distances = np.zeros(unit_x.shape, dtype=np.float64)
    for start, end in zip(vertices, np.roll(vertices, -1, axis=0)):
        edge_x = float(end[0] - start[0])
        edge_y = float(end[1] - start[1])
        denominator = unit_x * edge_y - unit_y * edge_x
        valid = np.abs(denominator) > 1.0e-9
        t = np.zeros_like(unit_x)
        cross_start_edge = float(start[0]) * edge_y - float(start[1]) * edge_x
        t[valid] = cross_start_edge / denominator[valid]
        segment = np.zeros_like(unit_x)
        segment[valid] = (
            float(start[0]) * unit_y[valid] - float(start[1]) * unit_x[valid]
        ) / denominator[valid]
        hit = valid & (t >= 0.0) & (segment >= -1.0e-9) & (segment <= 1.0 + 1.0e-9)
        distances = np.maximum(distances, np.where(hit, t, 0.0))
    return distances


def _lateral_support_in_longitudinal_strip(
    component_longitudinal: np.ndarray,
    component_lateral: np.ndarray,
    longitudinal: np.ndarray,
    strip_half_width_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return exact polygon lateral extrema in each path-frame strip.

    The extrema are at a vertex inside the strip or where an edge crosses a
    strip boundary.  This avoids applying a distal component's widest section
    along its entire longitudinal span.
    """
    lower = longitudinal - strip_half_width_m
    upper = longitudinal + strip_half_width_m
    positive = np.full(longitudinal.shape, -np.inf, dtype=np.float64)
    negative = np.full(longitudinal.shape, -np.inf, dtype=np.float64)

    def include(values: np.ndarray, valid: np.ndarray) -> None:
        nonlocal positive, negative
        positive = np.maximum(
            positive, np.max(np.where(valid, values, -np.inf), axis=0))
        negative = np.maximum(
            negative, np.max(np.where(valid, -values, -np.inf), axis=0))

    # Evaluate every vertex of this component in one NumPy operation.  The
    # live R1 projection currently has thousands of vertices; a Python loop
    # per vertex used enough callback time to make fresh scan/mesh data miss
    # the controller watchdog even though the planner solve was fast.
    vertex_longitudinal = component_longitudinal[:, None]
    vertex_lateral = component_lateral[:, None]
    vertex_inside = (
        (vertex_longitudinal >= lower[None, :])
        & (vertex_longitudinal <= upper[None, :])
    )
    include(vertex_lateral, vertex_inside)
    indices = np.arange(component_longitudinal.size)
    ends = np.roll(indices, -1)
    start_longitudinal = component_longitudinal[indices]
    delta_longitudinal = component_longitudinal[ends] - start_longitudinal
    start_lateral = component_lateral[indices]
    delta_lateral = component_lateral[ends] - start_lateral
    non_parallel = np.abs(delta_longitudinal) > 1.0e-12
    safe_delta = np.where(non_parallel, delta_longitudinal, 1.0)[:, None]
    edge_min = np.minimum(start_longitudinal, component_longitudinal[ends])[:, None] - 1.0e-12
    edge_max = np.maximum(start_longitudinal, component_longitudinal[ends])[:, None] + 1.0e-12
    for boundary in (lower, upper):
        boundary_matrix = boundary[None, :]
        valid = (
            non_parallel[:, None]
            & (boundary_matrix >= edge_min)
            & (boundary_matrix <= edge_max)
        )
        fraction = (boundary_matrix - start_longitudinal[:, None]) / safe_delta
        edge_lateral = start_lateral[:, None] + fraction * delta_lateral[:, None]
        include(edge_lateral, valid)
    return positive, negative, np.isfinite(positive) | np.isfinite(negative)


def inflate_obstacles_for_mesh_components(
    points: np.ndarray,
    components: list[np.ndarray],
    *,
    base_front_m: float,
    base_rear_m: float,
    base_left_m: float,
    base_right_m: float,
    clearance_m: float,
    path_tangent: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Encode live-arm width without pulling side obstacles toward the origin.

    NeuPAN already contains the fixed chassis rectangle.  The former
    ray-based implementation reduced the range of every retained return by
    the arm reach on that ray.  A wall beside the robot was consequently
    moved diagonally toward ``(0, 0)`` and could become a fictitious obstacle
    directly in front of the robot.

    This implementation works in the current local path frame instead.  It
    preserves the obstacle's longitudinal coordinate and contracts only its
    signed lateral coordinate by the *actual*, non-convex mesh components'
    lateral extension beyond the chassis.  At every obstacle position, a
    component contributes only its actual cross section in a longitudinal
    strip of ``clearance_m``. Each side is evaluated separately; disjoint
    left and right arm components are never convex-hulled together. The strip
    and lateral margin conservatively enclose the Euclidean clearance region.
    The collision guard remains responsible for the final swept-trajectory
    check, including the arm's longitudinal reach.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] != 2:
        raise ValueError("points must have shape (2, N)")
    if not components:
        raise ValueError("at least one mesh component is required")
    extents = (base_front_m, base_rear_m, base_left_m, base_right_m)
    if not all(np.isfinite(extents)) or min(extents) <= 0.0:
        raise ValueError("base footprint extents must be finite positive distances")
    clearance_m = float(clearance_m)
    if not np.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("clearance_m must be finite and nonnegative")
    if points.shape[1] == 0:
        return points.copy(), np.empty(0, dtype=np.float64)

    tangent = np.asarray(
        [1.0, 0.0] if path_tangent is None else path_tangent, dtype=np.float64
    ).reshape(-1)
    if tangent.size != 2 or not np.isfinite(tangent).all():
        raise ValueError("path_tangent must be a finite two-dimensional vector")
    tangent_norm = float(np.hypot(tangent[0], tangent[1]))
    if tangent_norm <= 1.0e-9:
        raise ValueError("path_tangent must have nonzero length")
    tangent /= tangent_norm
    normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)

    longitudinal = tangent @ points
    lateral = normal @ points
    positive_side = lateral >= 0.0
    base_support = np.where(
        positive_side, float(base_left_m), float(base_right_m)
    )
    mesh_support = base_support.copy()
    for component in [polygon_vertices_from_flat_xy(value) for value in components]:
        component_longitudinal = component @ tangent
        positive_support, negative_support, active = _lateral_support_in_longitudinal_strip(
            component_longitudinal,
            component @ normal,
            longitudinal,
            clearance_m,
        )
        if not np.any(active):
            continue
        support = np.where(
            positive_side, positive_support, negative_support
        )
        mesh_support = np.where(active, np.maximum(mesh_support, support), mesh_support)
    displacement = np.maximum(0.0, mesh_support - base_support) + clearance_m
    # A point exactly on the path centreline has no signed wall side.  Leave
    # it unchanged so an arm's side projection cannot fabricate a central
    # obstacle; the fixed chassis and final collision guard still see it.
    displacement = np.where(np.abs(lateral) > 1.0e-9, displacement, 0.0)
    adjusted_lateral = np.sign(lateral) * np.maximum(
        0.0, np.abs(lateral) - displacement
    )
    adjusted = (
        tangent[:, None] * longitudinal[None, :]
        + normal[:, None] * adjusted_lateral[None, :]
    )
    return adjusted, displacement


def mesh_component_overlap_mask(
    points: np.ndarray,
    components: list[np.ndarray],
    *,
    margin_m: float = 0.0,
) -> np.ndarray:
    """Return points geometrically overlapping a live mesh projection.

    This is intentionally evaluated on raw environment points, before the
    configuration-space remapping.  A negative planner clearance without an
    overlap here is a planning-space constraint, not evidence that the
    current physical pose is colliding.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] != 2:
        raise ValueError("points must have shape (2, N)")
    if not components:
        raise ValueError("at least one mesh component is required")
    margin_m = float(margin_m)
    if not np.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("margin_m must be finite and nonnegative")
    overlap = np.zeros(points.shape[1], dtype=bool)
    for component in components:
        overlap |= ~outside_polygon_footprint(
            points, component, margin_m=margin_m
        )
    return overlap
