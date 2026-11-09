#!/usr/bin/env python3
"""Verify the real HW1 launch checks both directions before starting nodes."""
import importlib.util
import shutil
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    'motion_launch', ROOT / 'packages/navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)

class SpeedLaunchValidation(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bridge = Path(self.temp.name) / 'bridge'
        self.guard = Path(self.temp.name) / 'guard'
        for target, source in ((self.bridge, ROOT / 'packages/navigation_neupan_bridge/config'),
                               (self.guard, ROOT / 'packages/r1_collision_guard/config')):
            shutil.copytree(source, target / 'config')

    def tearDown(self):
        self.temp.cleanup()

    def alter(self, target, filename, func):
        path = target / 'config' / filename
        payload = yaml.safe_load(path.read_text())
        func(payload)
        path.write_text(yaml.safe_dump(payload))

    def test_configured_chain_passes_both_profiles(self):
        for profile in ('direct', 'smoothed'):
            launch._validate_speed_profile(str(self.bridge), str(self.guard), profile)

    def test_forward_and_reverse_limits_can_differ(self):
        launch._validate_speed_profile(str(self.bridge), str(self.guard), "smoothed")
        self.alter(self.bridge, "planner_r1_hw1.yaml",
                   lambda p: p["robot"].update(max_reverse_speed=.8))
        with self.assertRaisesRegex(RuntimeError, "reverse exceeds"):
            launch._validate_speed_profile(str(self.bridge), str(self.guard), "smoothed")

    def test_reverse_mismatch_is_rejected(self):
        self.alter(self.bridge, 'nav2_params_r1_neupan_hw1.yaml',
                   lambda p: p['controller_server']['ros__parameters']['FollowPath'].update(max_reverse=0.2))
        with self.assertRaisesRegex(RuntimeError, 'controller_reverse'):
            launch._validate_speed_profile(str(self.bridge), str(self.guard), 'smoothed')

    def test_smoother_reverse_mismatch_is_rejected_only_if_used(self):
        self.alter(self.bridge, 'nav2_params_r1_neupan_hw1.yaml',
                   lambda p: p['neupan_velocity_smoother']['ros__parameters']['min_velocity'].__setitem__(0, -0.2))
        with self.assertRaisesRegex(RuntimeError, 'smoother_reverse'):
            launch._validate_speed_profile(str(self.bridge), str(self.guard), 'smoothed')
        launch._validate_speed_profile(str(self.bridge), str(self.guard), 'direct')

    def test_guard_reverse_and_angular_mismatches_are_rejected(self):
        self.alter(self.guard, 'r1_collision_guard_hw1.yaml',
                   lambda p: p['r1_collision_guard']['ros__parameters'].update(max_reverse_mps=0.2))
        with self.assertRaisesRegex(RuntimeError, 'guard_reverse'):
            launch._validate_speed_profile(str(self.bridge), str(self.guard), 'direct')

if __name__ == '__main__':
    unittest.main()
