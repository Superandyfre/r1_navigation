"""Evaluate launch builders without running processes."""
import os
from pathlib import Path
import unittest
from unittest.mock import patch
from launch_parameter_contract import compile_contract, select_parameters, typed

BUNDLE = Path(os.environ.get("NEUPAN_TEST_BUNDLE", "/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-06"))
CALIBRATION = Path("/home/rpp/rpp_ws/config/r1_hw1_calibration.json")

class PinEncoding(unittest.TestCase):
    def test_exact_types_and_unknown_yaml_rejected(self):
        self.assertEqual(typed(False), dict(type=1, value=False))
        self.assertEqual(typed(.5), dict(type=3, value=.5))
        with self.assertRaises(ValueError): typed([False, 0])
        with self.assertRaises(ValueError): select_parameters({"/**": {"ros__parameters": {"unsafe": True}}}, "worker")

@unittest.skipUnless(os.environ.get("NEUPAN_LAUNCH_CONTRACT_TEST") == "1", "selected bundle opt-in only")
class InstalledControlBuilders(unittest.TestCase):
    def test_direct_smoothed_and_mesh_profiles(self):
        from launch.actions import ExecuteProcess, OpaqueFunction
        from launch_ros.actions import Node
        with patch.object(ExecuteProcess, "execute", side_effect=AssertionError("process forbidden")), \
             patch.object(Node, "execute", side_effect=AssertionError("node forbidden")), \
             patch.object(OpaqueFunction, "execute", side_effect=AssertionError("opaque forbidden")), \
             patch("subprocess.Popen", side_effect=AssertionError("subprocess forbidden")):
            for profile in ("direct", "smoothed"):
                for mesh in (False, True):
                    data = compile_contract(BUNDLE, CALIBRATION, profile, mesh)
                    nodes = data["pins"]["nodes"]
                    self.assertEqual("/neupan_velocity_smoother" in nodes, profile == "smoothed")
                    self.assertNotIn("collision_check_enabled", nodes["/r1_collision_guard"]["parameters"])
                    self.assertEqual(nodes["/r1_collision_guard"]["parameters"]["cmd_vel_out_topic"]["value"], "/cmd_vel")
                    self.assertFalse(data["review_complete"])
                    self.assertFalse(data["motion_authorized"])

if __name__ == "__main__": unittest.main()
