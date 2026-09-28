"""R1-specific path semantics for N2-PRE dry-run admission.

The standard ``nav_msgs/Path`` remains unchanged.  Gear is carried by a
separate R1 integration input and materialized as the fourth row of the
NeuPAN path contract before the worker receives it.
"""

from __future__ import annotations

import numpy as np


DEFAULT_GEAR = 1
VALID_GEARS = (-1, 1)
NAVIGATION_MODES = ("cyclic", "terminal")


def validate_gear(value: int | float) -> int:
    gear = int(value)
    if float(value) != gear or gear not in VALID_GEARS:
        raise ValueError(f"gear must be exactly -1 or +1, got {value!r}")
    return gear


def validate_navigation_mode(value: str) -> str:
    mode = str(value).strip().lower()
    if mode not in NAVIGATION_MODES:
        raise ValueError(
            f"navigation_mode must be one of {NAVIGATION_MODES}, got {value!r}"
        )
    return mode


def navigation_loop(mode: str) -> bool:
    return validate_navigation_mode(mode) == "cyclic"


def normalize_path_matrix(path_matrix: np.ndarray, default_gear: int = DEFAULT_GEAR) -> np.ndarray:
    """Return a finite 4xN path with explicit, auditable gear values."""
    array = np.asarray(path_matrix, dtype=np.float64)
    if array.ndim != 2 or array.shape[0] not in (3, 4) or array.shape[1] < 2:
        raise ValueError(f"path must be a 3xN or 4xN matrix with N>=2, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError("path contains NaN or Inf")
    if array.shape[0] == 3:
        gear = validate_gear(default_gear)
        array = np.vstack([array, np.full((1, array.shape[1]), gear, dtype=np.float64)])
    else:
        gears = np.asarray([validate_gear(value) for value in array[3]], dtype=np.float64)
        transitions = int(np.count_nonzero(np.diff(gears)))
        if transitions > 1:
            raise ValueError("path contains more than one gear transition")
        array = array.copy()
        array[3] = gears
    return np.ascontiguousarray(array)


def collapse_invalid_stationary_prefix(
        path_matrix: np.ndarray, position_epsilon_m: float = 1.0e-3,
        yaw_epsilon_rad: float = 0.05) -> np.ndarray:
    """Remove an inconsistent zero-length prefix before NeuPAN consumes a path.

    Smac may prepend identical positions while retaining an unrelated pose yaw.
    Those points are not an in-place rotation when their yaw is constant and
    disagrees with the first geometric segment. They make NeuPAN retain a
    near-zero first command indefinitely. Genuine in-place rotations retain
    changing yaws and pass through unchanged.
    """
    array = normalize_path_matrix(path_matrix)
    first_xy = array[:2, 0]
    first_moving_index = next(
        (index for index in range(1, array.shape[1])
         if float(np.linalg.norm(array[:2, index] - first_xy)) > position_epsilon_m),
        None,
    )
    if first_moving_index is None or first_moving_index == 1:
        return array

    prefix_yaws = array[2, :first_moving_index]
    prefix_yaw = float(prefix_yaws[0])
    yaw_span = np.max(np.abs(np.arctan2(
        np.sin(prefix_yaws - prefix_yaw), np.cos(prefix_yaws - prefix_yaw))))
    tangent = float(np.arctan2(
        array[1, first_moving_index] - first_xy[1],
        array[0, first_moving_index] - first_xy[0]))
    yaw_error = abs(float(np.arctan2(
        np.sin(tangent - prefix_yaw), np.cos(tangent - prefix_yaw))))
    if yaw_span > yaw_epsilon_rad or yaw_error <= yaw_epsilon_rad:
        return array

    collapsed = np.hstack((array[:, :1], array[:, first_moving_index:]))
    collapsed[2, 0] = tangent
    return np.ascontiguousarray(collapsed)


def apply_uniform_gear(path_matrix: np.ndarray, gear: int) -> np.ndarray:
    """Apply one explicit gear to a generated path without changing geometry."""
    array = normalize_path_matrix(path_matrix, default_gear=gear)
    array[3, :] = validate_gear(gear)
    return array


def gear_values(path_matrix: np.ndarray) -> list[int]:
    array = normalize_path_matrix(path_matrix)
    return [int(value) for value in array[3].tolist()]


def transition_requires_zero(active_gear: int, requested_gear: int) -> bool:
    return validate_gear(active_gear) != validate_gear(requested_gear)
