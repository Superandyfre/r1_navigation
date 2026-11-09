"""Validated immutable arm-geometry snapshots.

Times in this object identify actual source observations; receipt does not
refresh a stale source timestamp. Geometry is expressed in the robot frame.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import numpy as np


@dataclass(frozen=True)
class ArmComponent:
    frame: str
    polygon_xy: tuple
    z_min: float
    z_max: float
    position_xyz: tuple | None = None
    orientation_xyzw: tuple | None = None
    local_radius_m: float | None = None


@dataclass(frozen=True)
class ArmGeometrySnapshot:
    stamp_ns: int
    sequence: int
    geometry_version: int
    components: tuple
    received_monotonic: float

    @classmethod
    def decode(cls, payload, received_monotonic):
        data = json.loads(payload) if isinstance(payload, str) else payload
        if not isinstance(data, dict) or data.get("schema") != "r1_arm_geometry_v1":
            raise ValueError("invalid geometry schema")
        stamp = data.get("stamp_ns")
        sequence = data.get("sequence")
        version = data.get("geometry_version")
        if any(type(x) is not int or x < 1 for x in (stamp, sequence, version)):
            raise ValueError("invalid geometry identity")
        if not math.isfinite(received_monotonic) or received_monotonic <= 0:
            raise ValueError("invalid geometry receive time")
        raw = data.get("components")
        if not isinstance(raw, list) or not raw:
            raise ValueError("missing robot components")
        from neupan.dynamic_geometry import DynamicGeometry
        components = []
        for item in raw:
            if not isinstance(item, dict) or not isinstance(item.get("frame"), str) or not item["frame"]:
                raise ValueError("invalid component frame")
            zmin, zmax = float(item["z_min"]), float(item["z_max"])
            if not math.isfinite(zmin) or not math.isfinite(zmax) or zmin > zmax:
                raise ValueError("invalid component height")
            polygon = DynamicGeometry._validate(item["polygon_xy"])
            position=item.get("position_xyz")
            orientation=item.get("orientation_xyzw")
            radius=item.get("local_radius_m")
            if any(v is not None for v in (position,orientation,radius)):
                position=np.asarray(position,dtype=float).reshape(-1)
                orientation=np.asarray(orientation,dtype=float).reshape(-1)
                radius=float(radius)
                if (position.shape!=(3,) or orientation.shape!=(4,) or
                        not np.isfinite(position).all() or not np.isfinite(orientation).all() or
                        np.linalg.norm(orientation)<1e-9 or not math.isfinite(radius) or radius<0.):
                    raise ValueError("invalid component pose")
                position=tuple(position);orientation=tuple(orientation)
            components.append(ArmComponent(item["frame"],
                tuple(tuple(float(v) for v in point) for point in polygon), zmin, zmax,
                position,orientation,radius))
        return cls(stamp,sequence,version,tuple(components),float(received_monotonic))

    def to_dict(self):
        return dict(schema="r1_arm_geometry_v1",stamp_ns=self.stamp_ns,
                    sequence=self.sequence,geometry_version=self.geometry_version,
                    components=[dict(frame=c.frame,polygon_xy=[list(p) for p in c.polygon_xy],
                                     z_min=c.z_min,z_max=c.z_max,
                                     **({"position_xyz":list(c.position_xyz),
                                         "orientation_xyzw":list(c.orientation_xyzw),
                                         "local_radius_m":c.local_radius_m}
                                        if c.position_xyz is not None else {}))
                                for c in self.components])

    def planner_components(self, previous=None):
        previous_by_frame={} if previous is None else {c.frame:c for c in previous.components}
        dt=0. if previous is None else (self.stamp_ns-previous.stamp_ns)*1e-9
        result=[]
        for component in self.components:
            old=previous_by_frame.get(component.frame)
            rate=None
            if (dt>0. and old is not None and old.position_xyz is not None and
                    component.position_xyz is not None):
                a=np.asarray(old.orientation_xyzw,dtype=float)
                b=np.asarray(component.orientation_xyzw,dtype=float)
                a/=np.linalg.norm(a);b/=np.linalg.norm(b)
                angle=2.*math.acos(float(np.clip(abs(np.dot(a,b)),0.,1.)))
                translation=np.linalg.norm(np.asarray(component.position_xyz)-np.asarray(old.position_xyz))
                rate=float((translation+angle*component.local_radius_m)/dt)
            result.append(dict(polygon_xy=[list(point) for point in component.polygon_xy],
                               z_min=component.z_min,z_max=component.z_max,
                               uncertainty_rate_mps=rate))
        return result

    def age(self, ros_now_ns):
        if type(ros_now_ns) is not int or ros_now_ns < self.stamp_ns:
            return None
        return (ros_now_ns-self.stamp_ns)*1.e-9
