"""Regression for duplicate mesh padding in the planner point input."""
import unittest
import numpy as np
from r1_neupan_integration.scan_self_filter import inflate_obstacles_for_mesh_components

class SingleArmClearance(unittest.TestCase):
    def projected_lateral(self, point, mesh):
        projected, _ = inflate_obstacles_for_mesh_components(
            np.asarray(point, dtype=float).reshape(2, 1), [np.asarray(mesh)],
            base_front_m=.341, base_rear_m=.341, base_left_m=.266,
            base_right_m=.266, clearance_m=.04, path_tangent=np.array([1., 0.]))
        return abs(projected[1, 0])

    def test_no_duplicate_padding(self):
        mesh = [[-.1,-.32],[.2,-.32],[.2,-.29],[-.1,-.29]]
        padded = [[-.16,-.38],[.26,-.38],[.26,-.23],[-.16,-.23]]
        point = [.105, -.378]
        self.assertLess(self.projected_lateral(point, padded), .266)
        self.assertGreater(self.projected_lateral(point, mesh), .266)
        self.assertLess(self.projected_lateral([.105, -.32], mesh), .266)

if __name__ == "__main__": unittest.main()
