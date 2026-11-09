"""Assemble an obstacle scene without relocating observations."""
from __future__ import annotations
import numpy as np


def assemble_scene(scan_xy, cloud_world_xyz=None, rear_world_xyz=None, base_z_world=0., arm_geometry=None, visibility_proven=False, base_pose_world=None, horizon_sec=None):
    """Keep the 2-D scan for chassis and add height-tagged arm observations.

    Unknown or missing sensor information is never treated as clear space:
    until both 3-D front and timestamped rear observations are available,
    the 2-D scan is evaluated against every robot component.
    """
    if horizon_sec is not None and (not np.isfinite(horizon_sec) or horizon_sec < 0.):
        raise ValueError("invalid arm prediction horizon")
    scan = np.asarray(scan_xy,dtype=np.float64)
    if scan.ndim != 2 or scan.shape[0] != 2 or not np.isfinite(scan).all():
        raise ValueError("invalid fused scan")
    if not np.isfinite(base_z_world):
        raise ValueError("invalid robot base height")
    pose = None if base_pose_world is None else np.asarray(base_pose_world,dtype=np.float64)
    if pose is not None and (pose.shape != (4,4) or not np.isfinite(pose).all()):
        raise ValueError("invalid robot base pose")
    clouds=[]
    for value in (cloud_world_xyz,rear_world_xyz):
        if value is None:
            clouds.append(None)
        else:
            xyz=np.asarray(value,dtype=np.float64)
            if xyz.ndim != 2 or xyz.shape[0] != 3 or not np.isfinite(xyz).all():
                raise ValueError("invalid timestamp-aligned obstacle cloud")
            clouds.append(xyz)
    # Receiving both topics is not proof that every arm-height ray was
    # observed. Only an explicit coverage certificate can downgrade a 2-D
    # obstacle to chassis-only.
    complete=all(x is not None for x in clouds) and arm_geometry is not None and visibility_proven
    arrays=[scan];height=[np.full(scan.shape[1],np.nan)]
    scope=[np.full(scan.shape[1],1 if complete else 0,dtype=np.uint8)]
    for xyz in clouds:
        if xyz is None:
            continue
        z=(xyz[2,:]-base_z_world if pose is None else
           pose[:3,2] @ (xyz-pose[:3,3:4]))
        if arm_geometry is not None:
            # The height interval comes from each measured component, never
            # from a generic fixed collision or braking distance.
            relevant=np.zeros(z.shape,dtype=bool)
            for (zmin,zmax),rate in zip(arm_geometry.height_ranges[1:], arm_geometry.motion_rates[1:]):
                # Unknown motion or horizon must never imply clear space.
                if rate is None or (rate > 0. and horizon_sec is None):
                    relevant[:] = True
                    break
                margin=rate * (arm_geometry.source_age_sec + (horizon_sec or 0.))
                relevant |= (z >= zmin-margin) & (z <= zmax+margin)
            xyz=xyz[:,relevant]
            z=z[relevant]
        arrays.append(xyz[:2,:].copy())
        height.append(z)
        scope.append(np.full(xyz.shape[1],2,dtype=np.uint8))
    return np.hstack(arrays),np.concatenate(height),np.concatenate(scope),complete



def select_sensor_scenes(snapshot, scan_stamp_ns, now_ns, max_skew_sec, max_age_sec, return_records=False):
    """Match both auxiliary sensors using source stamps, never receipt time."""
    if (type(scan_stamp_ns) is not int or type(now_ns) is not int or
            scan_stamp_ns <= 0 or now_ns < scan_stamp_ns or
            not np.isfinite(max_skew_sec) or max_skew_sec < 0 or
            not np.isfinite(max_age_sec) or max_age_sec <= 0):
        return None,None
    result=[]
    for kind in ("front","rear"):
        item=snapshot.get(kind)
        stamp=item.get("stamp_ns") if isinstance(item,dict) else None
        if (type(stamp) is not int or stamp <= 0 or stamp > now_ns or
                (now_ns-stamp)*1e-9 > max_age_sec or
                abs(stamp-scan_stamp_ns)*1e-9 > max_skew_sec):
            result.append(None)
        else:
            result.append(item if return_records else item["points"])
    return tuple(result)
