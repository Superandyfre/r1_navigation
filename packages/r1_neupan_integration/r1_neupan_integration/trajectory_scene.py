"""Timestamp-aligned, non-warping scene preparation for trajectory validation."""
import numpy as np


def project_scan_to_map(local_points, scan_pose, current_pose):
    """Use the scan-time pose for points and the current pose for rollout.

    The caller is responsible for obtaining TF at the actual scan timestamp,
    validating both pose timestamps, and rejecting stale/missing transforms.
    """
    points=np.asarray(local_points,dtype=np.float64)
    observed=np.asarray(scan_pose,dtype=np.float64).reshape(-1)
    current=np.asarray(current_pose,dtype=np.float64).reshape(-1)
    if (points.ndim!=2 or points.shape[0]!=2 or observed.shape!=(3,) or current.shape!=(3,) or
            not np.isfinite(points).all() or not np.isfinite(observed).all() or
            not np.isfinite(current).all()):
        raise ValueError("invalid_timestamped_scene")
    c,s=np.cos(observed[2]),np.sin(observed[2])
    rotation=np.array([[c,-s],[s,c]],dtype=np.float64)
    world=rotation @ points + observed[:2,None]
    return current.reshape(3,1).copy(),world
