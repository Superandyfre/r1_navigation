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
