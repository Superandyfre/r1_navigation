"""Obstacle geometry used directly in NeuPAN optimization."""

from __future__ import annotations
import math
import numpy as np


def signed_clearances(pose: np.ndarray, points: np.ndarray,
                      vertices: np.ndarray, G: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Signed point-to-footprint distances; negative values are inside."""
    x, y, yaw = np.asarray(pose, dtype=np.float64).reshape(-1)[:3]
    cloud = np.asarray(points, dtype=np.float64)
    polygon = np.asarray(vertices, dtype=np.float64).T
    starts = polygon
    edges = np.roll(polygon, -1, axis=0) - starts
    edge_lengths = np.sum(edges * edges, axis=1)
    dx, dy = cloud[0] - x, cloud[1] - y
    local = np.column_stack((math.cos(yaw) * dx + math.sin(yaw) * dy,
                             -math.sin(yaw) * dx + math.cos(yaw) * dy))
    inside = np.all(
        np.asarray(G) @ local.T <= np.asarray(h).reshape(-1, 1) + 1.0e-9,
        axis=0,
    )
    relative = local[:, None, :] - starts[None, :, :]
    fraction = np.clip(
        np.sum(relative * edges[None, :, :], axis=2) / edge_lengths[None, :],
        0.0, 1.0,
    )
    residual = relative - fraction[:, :, None] * edges[None, :, :]
    distance = np.sqrt(np.min(np.sum(residual * residual, axis=2), axis=1))
    return np.where(inside, -distance, distance)


def clearance_linearization(pose: np.ndarray, points: np.ndarray,
                            vertices: np.ndarray, G: np.ndarray, h: np.ndarray,
                            max_points: int, margin_m: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Linearize exact footprint clearance for the nearest obstacle points."""
    distances = signed_clearances(pose, points, vertices, G, h)
    selected = np.argsort(distances)[:max_points]
    nearby = points[:, selected]
    nominal = distances[selected] - margin_m
    pose = np.asarray(pose, dtype=np.float64).reshape(3)
    gradients = np.empty((selected.size, 3), dtype=np.float64)
    for axis, epsilon in enumerate((0.005, 0.005, 0.02)):
        delta = np.zeros(3)
        delta[axis] = epsilon
        gradients[:, axis] = (
            signed_clearances(pose + delta, nearby, vertices, G, h)
            - signed_clearances(pose - delta, nearby, vertices, G, h)
        ) / (2.0 * epsilon)
    offsets = gradients @ pose - nominal
    if selected.size < max_points:
        gradients = np.vstack((gradients, np.repeat(gradients[:1], max_points - selected.size, axis=0)))
        offsets = np.r_[offsets, np.repeat(offsets[:1], max_points - selected.size)]
    return gradients, offsets[:, None]


