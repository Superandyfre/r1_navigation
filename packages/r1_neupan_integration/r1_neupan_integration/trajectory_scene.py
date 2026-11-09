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


def make_time_aligned_scene(local_points, scan_stamp_sec, scan_received_monotonic,
                            ros_now_sec, monotonic_now, input_timeout, lookup_pose):
    """Build a scene only with a matching scan-time TF and a fresh current TF.

    lookup_pose(stamp) returns (x,y,yaw,tf_stamp_sec); None requests latest.
    All wall points use the scan-time transform; the rollout uses current pose.
    The caller rechecks task/scan identity before publishing any result.
    """
    times=np.asarray([scan_stamp_sec,scan_received_monotonic,ros_now_sec,
                      monotonic_now,input_timeout],dtype=np.float64)
    if (not np.isfinite(times).all() or scan_stamp_sec<=0 or scan_received_monotonic<=0 or
            input_timeout<=0 or scan_received_monotonic>monotonic_now or
            monotonic_now-scan_received_monotonic>input_timeout or
            ros_now_sec<scan_stamp_sec or ros_now_sec-scan_stamp_sec>input_timeout):
        return None
    try:
        scan_tf=lookup_pose(scan_stamp_sec)
        current_tf=lookup_pose(None)
        a=np.asarray(scan_tf,dtype=np.float64).reshape(-1)
        b=np.asarray(current_tf,dtype=np.float64).reshape(-1)
        if a.shape!=(4,) or b.shape!=(4,) or not np.isfinite(a).all() or not np.isfinite(b).all():
            return None
        # The timestamped lookup must describe the scan instant; the latest
        # transform must still be fresh rather than merely newly retrieved.
        tf_time_tolerance=1e-6
        if abs(a[3]-scan_stamp_sec)>tf_time_tolerance or b[3]<=0 or b[3]>ros_now_sec or ros_now_sec-b[3]>input_timeout:
            return None
        current,points=project_scan_to_map(local_points,a[:3],b[:3])
        # Convert the TF header age into the same monotonic clock without ever
        # making the pose look newer than its actual transform timestamp.
        pose_time=monotonic_now-(ros_now_sec-b[3])
        return dict(state=current,points=points,scan_received_monotonic=float(scan_received_monotonic),
                    pose_received_monotonic=float(pose_time),scan_stamp_sec=float(scan_stamp_sec),
                    pose_stamp_sec=float(b[3]))
    except Exception:
        # TF lookup failures and malformed observations are both unknown,
        # never evidence of a clear scene. The caller reports this state.
        return None
