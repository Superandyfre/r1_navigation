"""Candidate initializations and exact footprint checks for NeuPAN bypass solves."""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass


@dataclass
class BypassGuide:
    origin: np.ndarray
    tangent: np.ndarray
    side: int
    lateral_target: float
    obstacle_far: float
    goal_pose: np.ndarray
    phase: str = "side"
    backoff_distance: float = 0.0

    @property
    def normal(self) -> np.ndarray:
        return np.array((-self.tangent[1], self.tangent[0]))

    def update(self, pose: np.ndarray, footprint_radius: float, margin: float) -> None:
        relative = np.asarray(pose).reshape(-1)[:2] - self.origin
        along = float(self.tangent @ relative)
        across = float(self.normal @ relative)
        if self.phase == "backoff" and along <= -self.backoff_distance + 0.05:
            self.phase = "side"
        if self.phase == "side" and self.side * (across - self.lateral_target) >= -0.10:
            self.phase = "pass"
        if self.phase == "pass" and along >= self.obstacle_far + footprint_radius + margin:
            self.phase = "rejoin"
        if self.phase == "rejoin" and abs(across) <= 0.25:
            self.phase = "align"
        route_yaw = math.atan2(self.tangent[1], self.tangent[0])
        pose_yaw = float(np.asarray(pose).reshape(-1)[2])
        yaw_error = math.atan2(math.sin(route_yaw - pose_yaw),
                               math.cos(route_yaw - pose_yaw))
        if self.phase == "align" and abs(yaw_error) <= 0.20:
            self.phase = "final"
        goal_error = np.asarray(self.goal_pose).reshape(-1)[:2] - np.asarray(pose).reshape(-1)[:2]
        goal_yaw_error = math.atan2(
            math.sin(float(self.goal_pose[2]) - pose_yaw),
            math.cos(float(self.goal_pose[2]) - pose_yaw),
        )
        if self.phase == "final" and np.linalg.norm(goal_error) <= 0.06 and abs(goal_yaw_error) <= 0.20:
            self.phase = "done"

    def reference(self, pose: np.ndarray, steps: int, footprint_radius: float,
                  margin: float) -> np.ndarray:
        relative = np.asarray(pose).reshape(-1)[:2] - self.origin
        along = float(self.tangent @ relative)
        if self.phase == "backoff":
            goal_along = -self.backoff_distance
            goal_across = 0.0
            heading = math.atan2(self.tangent[1], self.tangent[0])
        elif self.phase == "side":
            goal_along = min(along, self.obstacle_far - footprint_radius - margin)
            goal_across = self.lateral_target
            heading = math.atan2(self.side * self.normal[1], self.side * self.normal[0])
        elif self.phase == "pass":
            goal_along = self.obstacle_far + footprint_radius + margin + 0.15
            goal_across = self.lateral_target
            heading = math.atan2(self.tangent[1], self.tangent[0])
        elif self.phase == "rejoin":
            goal_along = self.obstacle_far + footprint_radius + margin + 0.7
            goal_across = 0.0
        elif self.phase == "align":
            goal_along = along
            goal_across = float(self.normal @ relative)
            heading = math.atan2(self.tangent[1], self.tangent[0])
        else:
            result = np.repeat(np.asarray(self.goal_pose).reshape(3, 1), steps + 1, axis=1)
            delta = np.asarray(self.goal_pose).reshape(-1)[:2] - np.asarray(pose).reshape(-1)[:2]
            if float(np.linalg.norm(delta)) > 0.12:
                result[2, 1:] = math.atan2(delta[1], delta[0])
            result[:, 0] = np.asarray(pose).reshape(-1)[:3]
            return result
        goal = self.origin + goal_along * self.tangent + goal_across * self.normal
        if self.phase == "rejoin":
            delta = goal - np.asarray(pose).reshape(-1)[:2]
            heading = math.atan2(delta[1], delta[0])
        result = np.repeat(np.array([[goal[0]], [goal[1]], [heading]]), steps + 1, axis=1)
        result[:, 0] = np.asarray(pose).reshape(-1)[:3]
        return result


class BypassRecovery:
    """Steady-time hysteresis and bounded guide reconstruction per bypass."""
    def __init__(self):
        self.clear_since = None
        self.invalid_since = None
        self.rebuilds = 0
        self.exhausted = False

    def can_rejoin(self, now, route_clear, pose, route, clearance, margin):
        route = np.asarray(route, dtype=float)
        state = np.asarray(pose).reshape(-1)
        eligible = bool(route_clear and np.isfinite(route).all()
                        and route.ndim == 2 and route.shape[0] >= 3
                        and route.shape[1] >= 2 and clearance >= margin)
        best = math.inf
        heading_error = math.inf
        if eligible:
            projections = []
            arc = 0.0
            for a, b in zip(route[:2].T[:-1], route[:2].T[1:]):
                delta = b - a
                length = float(np.linalg.norm(delta))
                if length < 1e-6:
                    continue
                t = float(np.clip((state[:2]-a) @ delta / (length*length), 0., 1.))
                distance = float(np.linalg.norm(state[:2] - a - t*delta))
                projections.append((distance, arc + t*length))
                if distance < best:
                    best = distance
                    heading_error = abs(math.atan2(
                        math.sin(math.atan2(delta[1], delta[0]) - state[2]),
                        math.cos(math.atan2(delta[1], delta[0]) - state[2])))
                arc += length
            # At a crossing, proximity alone cannot identify forward progress.
            close_arcs = [s for d, s in projections if d <= best + .03]
            ambiguous = bool(close_arcs and max(close_arcs)-min(close_arcs) > .50)
            eligible = best <= .15 and heading_error <= .35 and not ambiguous
        if not eligible:
            self.clear_since = None
            return False
        if self.clear_since is None or now < self.clear_since:
            self.clear_since = now
        return now - self.clear_since >= .60

    def rejected(self, now):
        """Return True only when one of two bounded rebuilds becomes due."""
        if self.invalid_since is None or now < self.invalid_since:
            self.invalid_since = now
        if now - self.invalid_since < .8:
            return False
        self.invalid_since = now
        if self.rebuilds >= 2:
            self.exhausted = True
            return False
        self.rebuilds += 1
        return True

    def accepted(self):
        self.invalid_since = None


def make_bypass_guide(pose: np.ndarray, points: np.ndarray, reference: np.ndarray,
                      side: int, footprint_radius: float, margin: float,
                      goal_pose: np.ndarray, vertices: np.ndarray,
                      G: np.ndarray, h: np.ndarray) -> BypassGuide | None:
    origin = np.asarray(pose).reshape(-1)[:2].copy()
    direction = reference[:2, -1] - reference[:2, 0]
    length = float(np.linalg.norm(direction))
    if length < 1.0e-5:
        return None
    tangent = direction / length
    normal = np.array((-tangent[1], tangent[0]))
    relative = np.asarray(points) - origin[:, None]
    along = tangent @ relative
    across = normal @ relative
    relevant = (along > 0.0) & (along < 2.5) & (np.abs(across) < 1.5)
    if not np.any(relevant):
        return None
    route_blockers = relevant & (np.abs(across) <= footprint_radius + margin)
    if not np.any(route_blockers):
        return None
    front = float(np.min(along[route_blockers]))
    far = float(np.max(along[route_blockers]))
    component = relevant & (along >= front - 0.10) & (along <= far + 0.10)
    reach = float(np.max(across[component]) if side > 0 else np.min(across[component]))
    target = reach + side * (footprint_radius + margin)
    if abs(target) > 1.5:
        return None
    side_heading = math.atan2(side * normal[1], side * normal[0])
    current_yaw = float(np.asarray(pose).reshape(-1)[2])
    yaw_delta = math.atan2(math.sin(side_heading - current_yaw),
                           math.cos(side_heading - current_yaw))
    def side_is_clear(at: np.ndarray) -> bool:
        for fraction in np.linspace(0.0, 1.0, 9):
            rotating = np.array((at[0], at[1], current_yaw + fraction * yaw_delta))
            sliding_xy = at + fraction * target * normal
            sliding = np.array((sliding_xy[0], sliding_xy[1], side_heading))
            if (np.min(signed_clearances(rotating, points, vertices, G, h)) < margin
                    or np.min(signed_clearances(sliding, points, vertices, G, h)) < margin):
                return False
        return True

    backoff = 0.0
    if not side_is_clear(origin):
        backoff = 0.40
        if any(np.min(signed_clearances(
            np.array((*(origin - fraction * backoff * tangent), current_yaw)),
            points, vertices, G, h,
        )) < margin for fraction in np.linspace(0.0, 1.0, 11)):
            return None
        if not side_is_clear(origin - backoff * tangent):
            return None
    return BypassGuide(
        origin, tangent, side, target, far,
        np.asarray(goal_pose).reshape(-1)[:3].copy(),
        "backoff" if backoff else "side", backoff,
    )


def seed_reverse_controls(state: np.ndarray, steps: int, dt: float,
                          speed: float) -> tuple[np.ndarray, np.ndarray]:
    controls = np.zeros((2, steps), dtype=np.float64)
    controls[0] = -speed
    poses = np.empty((3, steps + 1), dtype=np.float64)
    poses[:, 0] = np.asarray(state).reshape(-1)[:3]
    for index in range(steps):
        poses[:, index + 1] = poses[:, index]
        poses[0, index + 1] -= speed * math.cos(poses[2, index]) * dt
        poses[1, index + 1] -= speed * math.sin(poses[2, index]) * dt
    return poses, controls


def seed_controls(state: np.ndarray, steps: int, dt: float, side: int,
                  max_turn: float, cruise_speed: float) -> tuple[np.ndarray, np.ndarray]:
    """Give the optimizer a feasible turning-side initial linearization."""
    controls = np.zeros((2, steps), dtype=np.float64)
    turn_steps = min(steps - 1, max(1, math.ceil(0.65 / (max_turn * dt))))
    controls[1, :turn_steps] = side * max_turn
    controls[0, turn_steps:] = cruise_speed
    poses = np.empty((3, steps + 1), dtype=np.float64)
    poses[:, 0] = np.asarray(state, dtype=np.float64).reshape(-1)[:3]
    for index in range(steps):
        x, y, yaw = poses[:, index]
        linear, angular = controls[:, index]
        midpoint = yaw + 0.5 * angular * dt
        poses[:, index + 1] = (
            x + linear * math.cos(midpoint) * dt,
            y + linear * math.sin(midpoint) * dt,
            yaw + angular * dt,
        )
    return poses, controls


def trajectory_clearance(states: np.ndarray, points: np.ndarray,
                         vertices: np.ndarray, G: np.ndarray, h: np.ndarray) -> float:
    """Smallest point-to-footprint distance along the predicted swept motion."""
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.size == 0:
        return math.inf
    trajectory = np.asarray(states, dtype=np.float64).T
    minimum = math.inf
    for start, end in zip(trajectory[:-1], trajectory[1:]):
        delta = end - start
        delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        samples = max(2, math.ceil(
            max(np.linalg.norm(delta[:2]) / 0.02, abs(delta[2]) / 0.05)
        ) + 1)
        for fraction in np.linspace(0.0, 1.0, samples):
            minimum = min(minimum, float(np.min(signed_clearances(
                start + fraction * delta, cloud, vertices, G, h
            ))))
    return minimum


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


def candidate_progress(states: np.ndarray, reference: np.ndarray) -> tuple[float, float]:
    start = states[:2, 0]
    direction = reference[:2, -1] - reference[:2, 0]
    norm = float(np.linalg.norm(direction))
    if norm < 1.0e-6:
        direction = np.asarray((math.cos(reference[2, 0]), math.sin(reference[2, 0])))
    else:
        direction /= norm
    displacement = states[:2, -1] - start
    return (float(direction @ displacement),
            float(np.cross(direction, displacement)))
