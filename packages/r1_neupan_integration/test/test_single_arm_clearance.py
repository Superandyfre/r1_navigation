"""Exercise the duplicate-padding failure and nearby real collision."""
import unittest
import numpy as np
from r1_neupan_integration.scan_self_filter import inflate_obstacles_for_mesh_components
from r1_neupan_integration.obstacle_motion_policy import MotionPolicyConfig, plan_obstacle_speed

class SingleArmClearance(unittest.TestCase):
    def decision(self, point, mesh):
        raw = np.asarray(point, dtype=float).reshape(2, 1)
        projected, _ = inflate_obstacles_for_mesh_components(
            raw, [mesh], base_front_m=.341, base_rear_m=.341,
            base_left_m=.266, base_right_m=.266, clearance_m=.04,
            path_tangent=np.array([1., 0.]))
        state = np.zeros((3, 1))
        path = np.vstack((np.linspace(0, .9, 19), np.zeros(19), np.zeros(19)))
        return plan_obstacle_speed(state, path, raw, projected,
                                   MotionPolicyConfig(stop_margin_m=.2))

    def test_side_return_is_not_pulled_inside_base_twice(self):
        mesh = np.array([[-.1,-.32],[.2,-.32],[.2,-.29],[-.1,-.29]])
        padded = np.array([[-.16,-.38],[.26,-.38],[.26,-.23],[-.16,-.23]])
        point = [.105, -.378]
        self.assertTrue(self.decision(point, padded).route_blocked)
        clear = self.decision(point, mesh)
        self.assertFalse(clear.route_blocked)
        self.assertGreater(clear.speed_profile[0], 0.)

    def test_obstacle_close_to_actual_arm_remains_blocked(self):
        mesh = np.array([[-.1,-.32],[.2,-.32],[.2,-.29],[-.1,-.29]])
        self.assertTrue(self.decision([.105, -.32], mesh).route_blocked)

if __name__ == '__main__':
    unittest.main()
