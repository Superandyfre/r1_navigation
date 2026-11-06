#!/usr/bin/env python3
"""Ensure all forward speed limits agree; reverse limits remain independent."""
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[3]

def read(relative):
    return yaml.safe_load((ROOT / relative).read_text())

class SpeedProfileConsistency(unittest.TestCase):
    def test_hw1_has_no_external_obstacle_motion_policy(self):
        params = read("packages/navigation_neupan_bridge/config/robot_r1_hw1.yaml")["n1_16_production_bridge"]["ros__parameters"]
        for name in ("obstacle_motion_enabled", "obstacle_braking_mps2", "obstacle_reaction_sec", "obstacle_stop_margin_m", "obstacle_preview_m"):
            self.assertNotIn(name, params)
        source = (ROOT / "packages/r1_neupan_integration/r1_neupan_integration/n1_16_node.py").read_text()
        self.assertNotIn("obstacle_motion_policy", source)
        self.assertNotIn("first_command_collision_distance", source)
        self.assertNotIn("path_deviation_blocked", source)

    def test_forward_speed_is_consistent_across_motion_chain(self):
        planner = read('packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml')
        nav = read('packages/navigation_neupan_bridge/config/nav2_params_r1_neupan_hw1.yaml')
        guard = read('packages/r1_collision_guard/config/r1_collision_guard_hw1.yaml')
        controller = nav['controller_server']['ros__parameters']['FollowPath']
        smoother = nav['neupan_velocity_smoother']['ros__parameters']
        guard = guard['r1_collision_guard']['ros__parameters']
        expected = planner['ref_speed']
        self.assertAlmostEqual(expected, 1.0)
        self.assertEqual(planner['robot']['max_speed'][0], expected)
        self.assertEqual(controller['max_linear'], expected)
        self.assertEqual(smoother['max_velocity'][0], expected)
        self.assertEqual(guard['max_linear_mps'], expected)
        self.assertEqual(controller['max_reverse'], .20)
        self.assertEqual(smoother['min_velocity'][0], -.20)
        self.assertEqual(guard['max_reverse_mps'], .20)
        self.assertLessEqual(smoother['max_accel'][0], planner['robot']['max_acce'][0])

if __name__ == '__main__':
    unittest.main()
