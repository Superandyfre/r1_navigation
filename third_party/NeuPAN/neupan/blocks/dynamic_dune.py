"""Analytic dual-distance coefficients for an immutable multi-part robot.

The output has the same per-point/world-normal contract as NeuPAN's learned
DUNE, but uses exact convex component geometry instead of a fixed-body network.
"""
import numpy as np
import torch


class DynamicDUNE(torch.nn.Module):
    def __init__(self, geometry, step_time, inactive_distance=1.):
        super().__init__()
        self.geometry = geometry
        self.step_time = float(step_time)
        self.inactive_distance = float(inactive_distance)
        self.obstacle_points = None
        self.min_distance = float("inf")

    def forward(self, point_flow, rotations, obstacle_points, point_heights=None, point_scope=None):
        if len(point_flow) != len(rotations) or len(point_flow) != len(obstacle_points):
            raise ValueError("inconsistent dynamic point flow")
        self.obstacle_points = obstacle_points[0]
        sizes=[local.shape[1] for local in point_flow]
        if point_heights is not None and any(n != point_heights.numel() for n in sizes):
            raise ValueError("dynamic heights do not match point flow")
        if point_scope is not None and any(n != point_scope.numel() for n in sizes):
            raise ValueError("dynamic scopes do not match point flow")
        combined=torch.cat(point_flow,dim=1)
        heights=(None if point_heights is None else
                 np.tile(point_heights.detach().cpu().numpy().reshape(-1),len(point_flow)))
        scopes=(None if point_scope is None else
                np.tile(point_scope.detach().cpu().numpy().reshape(-1),len(point_flow)))
        times=np.concatenate([np.full(count,index*self.step_time) for index,count in enumerate(sizes)])
        distances,outward,support = self.geometry.distance_and_normal(
            combined.detach().cpu().numpy(),heights,scopes,times)
        # A height-disjoint component creates no active constraint at this
        # instant. Encode it as a finite zero-normal satisfied constraint.
        inactive=~np.isfinite(distances)
        support[inactive]=-self.inactive_distance
        outward[:,inactive]=0.
        distances[inactive]=self.inactive_distance
        offsets, normals, sorted_points = [], [], []
        start=0
        for index, (local, rotation, world, count) in enumerate(zip(point_flow, rotations, obstacle_points, sizes)):
            device, dtype = local.device, local.dtype
            values = torch.as_tensor(distances[start:start+count], device=device, dtype=dtype)
            supports = torch.as_tensor(support[start:start+count], device=device, dtype=dtype).reshape(1, -1)
            normal = torch.as_tensor(outward[:,start:start+count], device=device, dtype=dtype)
            start+=count
            lam = -rotation @ normal
            order = torch.argsort(values, stable=True)
            if index == 0:
                self.min_distance = torch.min(values) if values.numel() else float("inf")
            offsets.append(supports[:, order])
            normals.append(lam[:, order])
            sorted_points.append(world[:, order])
        return offsets, normals, sorted_points
