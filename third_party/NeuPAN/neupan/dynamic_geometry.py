"""Exact local-frame distance to a union of convex robot components.

This geometry represents the robot itself; it never moves obstacle points.
Each polygon is a separate component, so free space between arms remains free.
"""
from __future__ import annotations

import numpy as np


class DynamicGeometry:
    """Immutable, independently validated convex component snapshot."""

    def __init__(self, components, source_age_sec=0.):
        polygons, heights, rates = [], [], []
        if not np.isfinite(source_age_sec) or source_age_sec < 0:
            raise ValueError("invalid geometry age")
        self.source_age_sec=float(source_age_sec)
        for part in components:
            if isinstance(part, dict):
                poly = self._validate(part["polygon_xy"])
                z_min, z_max = float(part["z_min"]), float(part["z_max"])
                if not np.isfinite([z_min, z_max]).all() or z_min > z_max:
                    raise ValueError("invalid component height")
                heights.append((z_min,z_max))
                rate=part.get("uncertainty_rate_mps")
                if rate is not None and (not np.isfinite(rate) or rate < 0.):
                    raise ValueError("invalid component motion rate")
                rates.append(None if rate is None else float(rate))
            else:
                poly = self._validate(part)
                # Missing height is unknown, so do not assume vertical clearance.
                heights.append((-np.inf,np.inf))
                rates.append(0.)
            polygons.append(poly)
        self.components = tuple(polygons)
        # Geometry is immutable for a solve; derive polygon constants once,
        # instead of recalculating them for every horizon and candidate.
        cached=[]
        for poly in polygons:
            edge=np.roll(poly,-1,axis=0)-poly
            length=np.linalg.norm(edge,axis=1)
            outward=np.stack((edge[:,1],-edge[:,0]),axis=1)/length[:,None]
            cached.append((poly.min(axis=0),poly.max(axis=0),edge,length,outward))
        self._polygon_cache=tuple(cached)
        self.height_ranges = tuple(heights)
        self.motion_rates = tuple(rates)
        self.motion_known = all(rate is not None for rate in rates)
        if not self.components:
            raise ValueError("at least one robot component is required")
        from . import dynamic_geometry_native
        self._native_data = dynamic_geometry_native.prepare(self) if dynamic_geometry_native.available() else None

    @staticmethod
    def _validate(component):
        poly = np.asarray(component, dtype=np.float64).copy()
        if (poly.ndim != 2 or poly.shape[1] != 2 or poly.shape[0] < 3 or
                not np.isfinite(poly).all()):
            raise ValueError("invalid robot component")
        edges = np.roll(poly, -1, axis=0) - poly
        if np.any(np.linalg.norm(edges, axis=1) < 1.e-9):
            raise ValueError("degenerate robot component edge")
        cross = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
        area = np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) -
                      poly[:, 1] * np.roll(poly[:, 0], -1))
        if abs(area) < 1.e-9 or (np.any(cross > 1.e-9) and np.any(cross < -1.e-9)):
            raise ValueError("robot component must be convex and nondegenerate")
        if area < 0:
            poly = poly[::-1].copy()
        poly.setflags(write=False)
        return poly

    def trajectory_collision_candidates(self, states, points, point_heights=None, point_scope=None, padding=0., sample_times=None):
        """Return points inside any component's swept AABB at sampled poses.

        Points outside all such boxes cannot intersect the represented robot at
        any of these poses; those inside still require exact polygon checking.
        """
        states=np.asarray(states,dtype=np.float64)
        points=np.asarray(points,dtype=np.float64)
        if not np.isfinite(padding) or padding < 0.:
            raise ValueError("invalid broadphase padding")
        if (states.ndim!=2 or states.shape[0]!=3 or points.ndim!=2 or points.shape[0]!=2 or
                not np.isfinite(states).all() or not np.isfinite(points).all()):
            raise ValueError("invalid trajectory or obstacles")
        n=points.shape[1]
        heights=np.full(n,np.nan) if point_heights is None else np.asarray(point_heights,dtype=float).reshape(-1)
        scope=np.zeros(n,dtype=np.uint8) if point_scope is None else np.asarray(point_scope,dtype=np.uint8).reshape(-1)
        if heights.shape!=(n,) or scope.shape!=(n,) or np.isinf(heights).any() or np.any(scope>2):
            raise ValueError("invalid obstacle metadata")
        times=np.zeros(states.shape[1]) if sample_times is None else np.asarray(sample_times,dtype=float).reshape(-1)
        if times.shape!=(states.shape[1],) or not np.isfinite(times).all() or np.any(times<0):
            raise ValueError("invalid trajectory times")
        result=np.zeros(n,dtype=bool)
        c=np.cos(states[2]);sn=np.sin(states[2])
        for i,(poly,(zmin,zmax),rate) in enumerate(zip(self.components,self.height_ranges,self.motion_rates)):
            x=states[0,:,None]+c[:,None]*poly[None,:,0]-sn[:,None]*poly[None,:,1]
            y=states[1,:,None]+sn[:,None]*poly[None,:,0]+c[:,None]*poly[None,:,1]
            margin=(0. if rate is None else rate)*(times+self.source_age_sec)
            zmargin=float(np.max(margin))
            relevant=(np.isnan(heights)|((heights>=zmin-zmargin)&(heights<=zmax+zmargin)))
            relevant &= (scope != 1) if i else (scope != 2)
            result |= (relevant & (points[0]>=np.min(x-margin[:,None])-padding) &
                       (points[0]<=np.max(x+margin[:,None])+padding) &
                       (points[1]>=np.min(y-margin[:,None])-padding) &
                       (points[1]<=np.max(y+margin[:,None])+padding))
        return result

    def box_lower_bound(self, points, point_heights=None, point_scope=None, point_times=None):
        """AABB lower bound used only for risk ranking, never for collision proof."""
        pts=np.asarray(points,dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] != 2 or not np.isfinite(pts).all():
            raise ValueError("invalid obstacle points")
        n=pts.shape[1]
        heights=np.full(n,np.nan) if point_heights is None else np.asarray(point_heights,dtype=float).reshape(-1)
        scope=np.zeros(n,dtype=np.uint8) if point_scope is None else np.asarray(point_scope,dtype=np.uint8).reshape(-1)
        if heights.shape!=(n,) or scope.shape!=(n,) or np.isinf(heights).any() or np.any(scope > 2):
            raise ValueError("invalid obstacle metadata")
        times=np.zeros(n) if point_times is None else np.asarray(point_times,dtype=float).reshape(-1)
        if times.shape!=(n,) or not np.isfinite(times).all() or np.any(times<0):
            raise ValueError("invalid obstacle times")
        result=np.full(n,np.inf)
        for i,(poly,(zmin,zmax),rate,cached) in enumerate(zip(self.components,self.height_ranges,self.motion_rates,self._polygon_cache)):
            lo,hi=cached[:2]
            dx=np.maximum(np.maximum(lo[0]-pts[0],pts[0]-hi[0]),0.)
            dy=np.maximum(np.maximum(lo[1]-pts[1],pts[1]-hi[1]),0.)
            margin=(0. if rate is None else rate)*(times+self.source_age_sec)
            eligible=(np.isnan(heights)|((heights>=zmin-margin)&(heights<=zmax+margin)))
            eligible &= (scope != 1) if i else (scope != 2)
            distance=np.maximum(0.,np.hypot(dx,dy)-margin)
            np.minimum(result,distance,out=result,where=eligible)
        return result

    def distance_and_normal(self, points, point_heights=None, point_scope=None, point_times=None):
        """Signed distance, outward normal and support to nearest component.

        points are unchanged and have shape (2,N). Outside a component,
        distance is Euclidean to its nearest boundary; inside it is negative.
        Min across components preserves each distinct component. In overlaps,
        negative distance is conservative and is not claimed to be the exact
        signed distance to the boundary of the union.
        """
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] != 2 or not np.isfinite(pts).all():
            raise ValueError("invalid obstacle points")
        n = pts.shape[1]
        heights = np.full(n, np.nan) if point_heights is None else np.asarray(point_heights, dtype=np.float64).reshape(-1)
        if heights.shape != (n,) or np.isinf(heights).any():
            raise ValueError("invalid obstacle heights")
        point_scope = np.zeros(n,dtype=np.uint8) if point_scope is None else np.asarray(point_scope,dtype=np.uint8).reshape(-1)
        if point_scope.shape != (n,) or np.any(point_scope > 2):
            raise ValueError("invalid obstacle component scopes")
        times=np.zeros(n) if point_times is None else np.asarray(point_times,dtype=float).reshape(-1)
        if times.shape!=(n,) or not np.isfinite(times).all() or np.any(times<0):
            raise ValueError("invalid obstacle times")
        if self._native_data is not None:
            from . import dynamic_geometry_native
            return dynamic_geometry_native.distance(self,pts,heights,point_scope,times)
        best = np.full(n, np.inf)
        normals = np.zeros((2, n))
        supports = np.zeros(n)
        for component_index, (poly, (z_min,z_max), rate, cached) in enumerate(zip(self.components,self.height_ranges,self.motion_rates,self._polygon_cache)):
            margin=(0. if rate is None else rate)*(times+self.source_age_sec)
            overlaps_height = np.isnan(heights) | ((heights >= z_min-margin) & (heights <= z_max+margin))
            eligible = overlaps_height & ((point_scope != 1) if component_index else (point_scope != 2))
            # An AABB supplies a lower bound on the positive distance.
            # A point inside the box may have negative signed distance and must
            # also be checked when it already overlaps another component.
            lower, upper, edge, length, outward = cached
            dx = np.maximum(np.maximum(lower[0]-pts[0],pts[0]-upper[0]),0.)
            dy = np.maximum(np.maximum(lower[1]-pts[1],pts[1]-upper[1]),0.)
            box_distance = np.maximum(0.,np.hypot(dx,dy)-margin)
            possible = eligible & (((best >= 0.) & (box_distance <= best)) |
                                   ((best < 0.) & (box_distance == 0.)))
            selected = np.flatnonzero(possible)
            if selected.size == 0:
                continue
            selected_points = pts[:,selected].T
            diff = selected_points[None, :, :] - poly[:, None, :]
            projection = np.clip(np.sum(diff * edge[:, None, :], axis=2) /
                                 (length[:, None] ** 2), 0., 1.)
            closest = poly[:, None, :] + projection[:, :, None] * edge[:, None, :]
            delta = selected_points[None, :, :] - closest
            squared = np.sum(delta * delta, axis=2)
            indexes = np.argmin(squared, axis=0)
            cols = np.arange(selected.size)
            nearest = closest[indexes, cols]
            delta_near = delta[indexes, cols]
            distance = np.sqrt(squared[indexes, cols])
            inside = np.all(np.sum(diff * outward[:, None, :], axis=2) <= 1.e-10, axis=0)
            signed = np.where(inside, -distance, distance) - margin[selected]
            direction = np.divide(delta_near, distance[:, None],
                                  out=np.zeros_like(delta_near), where=distance[:, None] > 1.e-12)
            direction[inside] *= -1.
            zero = distance <= 1.e-12
            direction[zero] = outward[indexes[zero]]
            replace = signed < best[selected]
            chosen = selected[replace]
            best[chosen] = signed[replace]
            normals[:, chosen] = direction[replace].T
            supports[chosen] = np.sum(direction[replace] * nearest[replace], axis=1) + margin[chosen]
        return best, normals, supports
