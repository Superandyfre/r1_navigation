"""No ROS/driver: route handoff, bounded retries and final heading regressions."""
import math
import os
import importlib
from pathlib import Path
from unittest.mock import patch
import unittest
import numpy as np
import yaml
from neupan.bypass import BypassGuide, BypassRecovery, make_route_rejoin_guide


class BypassRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.policy = BypassRecovery()
        self.route = np.array([[0., .5, 1.], [0., 0., 0.], [0., 0., 0.]])
        self.pose = np.array([[.1], [.02], [.05]])

    def rejoin(self, t, clear=True, pose=None, route=None, clearance=.06):
        return self.policy.can_rejoin(t, clear, self.pose if pose is None else pose,
            self.route if route is None else route, clearance, .05)

    def test_clear_route_requires_stability_and_resets_on_blockage(self):
        self.assertFalse(self.rejoin(1.))
        self.assertFalse(self.rejoin(1.5))
        self.assertFalse(self.rejoin(1.55, clear=False))
        self.assertFalse(self.rejoin(2.))
        self.assertTrue(self.rejoin(2.61))

    def test_nearby_obstacles_bad_heading_and_off_route_cannot_handoff(self):
        for kwargs in (dict(clearance=.049), dict(pose=np.array([.1, .02, math.pi])),
                       dict(pose=np.array([.1, .3, 0.]))):
            self.assertFalse(self.rejoin(1., **kwargs))
            self.assertFalse(self.rejoin(4., **kwargs))
        self.assertFalse(self.rejoin(5.))
        self.assertTrue(self.rejoin(5.7))

    def test_crossing_is_not_a_unique_progress_match(self):
        loop = np.array([[0., 1., 1., 0., 0., 1.],
                         [0., 0., 1., 1., 0., 0.], [0., 0., 0., 0., 0., 0.]])
        self.assertFalse(self.rejoin(1., route=loop))
        self.assertFalse(self.rejoin(5., route=loop))

    def test_only_two_rebuilds_then_exhaustion(self):
        self.assertFalse(self.policy.rejected(1.))
        self.assertTrue(self.policy.rejected(1.81))
        self.assertTrue(self.policy.rejected(2.62))
        self.assertFalse(self.policy.rejected(3.43))
        self.assertTrue(self.policy.exhausted)
        self.assertEqual(self.policy.rebuilds, 2)

    def test_valid_motion_breaks_consecutive_failure_window(self):
        self.policy.rejected(1.)
        self.policy.accepted()
        self.assertFalse(self.policy.rejected(3.))
        self.assertFalse(self.policy.rejected(3.5))

    def test_final_aims_at_goal_even_after_passing_original_tangent(self):
        guide = BypassGuide(np.zeros(2), np.array([1., 0.]), -1, -.5, .5,
                            np.array([-.6, .4, -math.pi]), phase='final')
        reference = guide.reference(np.array([[.3], [0.], [math.pi/2]]), 20, .4, .08)
        self.assertAlmostEqual(reference[2, 1], math.atan2(.4, -.9))
        self.assertAlmostEqual(reference[2, 0], math.pi/2)

    def test_connector_checks_swept_geometry_and_selects_forward_point(self):
        vertices = np.array([[.341, .341, -.341, -.341], [.266, -.266, -.266, .266]])
        G = np.array([[1., 0.], [-1., 0.], [0., 1.], [0., -1.]])
        h = np.array([[.341], [.341], [.266], [.266]])
        pose = np.array([.2, .5, 0.])
        far_cloud = np.array([[2., 2.], [2., -2.]])
        guide = make_route_rejoin_guide(pose, self.route, far_cloud, vertices, G, h, .05, [1., 0., 0.])
        self.assertIsNotNone(guide)
        self.assertEqual(guide.phase, 'route_rejoin')
        self.assertGreater(guide.route_target[0], pose[0])
        blocked = np.column_stack((far_cloud, [.3, .3]))
        self.assertIsNone(make_route_rejoin_guide(pose, self.route, blocked, vertices, G, h, .05, [1., 0., 0.]))


@unittest.skipUnless(os.environ.get('NEUPAN_SOLVER_REGRESSION') == '1', 'actual solver opt-in')
class ActualSolverRecoveryTests(unittest.TestCase):
    def test_clear_route_releases_old_final_guide_and_moves(self):
        from neupan import neupan
        root = Path(__file__).resolve().parents[3]
        config = yaml.safe_load((root/'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml').read_text())
        config['pan']['dune_checkpoint'] = str(root/'packages/r1_neupan_integration/config/models/dune_model_5000.pth')
        config['ipath']['loop'] = False
        core = neupan(**{k: v for k, v in config.items() if k not in ('robot', 'ipath', 'pan', 'adjust')},
            robot_kwargs=config['robot'], ipath_kwargs=config['ipath'],
            pan_kwargs=config['pan'], adjust_kwargs=config['adjust'])
        core.update_initial_path_from_goal(np.zeros((3, 1)), np.array([[2.], [0.], [0.]]))
        core.bypass_guide = BypassGuide(np.zeros(2), np.array([1., 0.]), 1, .5, .5,
                                       np.array([2., 0., 0.]), phase='final')
        core.bypass_active = True
        for stamp in (1., 1.3, 1.7):
            with patch.object(importlib.import_module('neupan.neupan').time, 'monotonic', return_value=stamp):
                action, info = core(np.zeros((3, 1)), np.array([[0., 1., 2.], [2., 2., 2.]]),
                    measured_velocity=np.zeros((2, 1)), reference_speed_profile=np.full(20, .2),
                    bypass_mode=True, bypass_route_clear=True)
        self.assertEqual(info['bypass_phase'], 'rejoined')
        self.assertFalse(info['stop'])
        self.assertGreater(float(action[0, 0]), .01)


if __name__ == '__main__':
    unittest.main()
