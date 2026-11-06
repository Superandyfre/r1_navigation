import os
from pathlib import Path
import re
import subprocess
import unittest
from unittest.mock import patch
from ui7_control_contract import button_options, trace

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = Path(os.environ.get("NEUPAN_TEST_BUNDLE", "/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-07"))
CALIBRATION = Path("/home/rpp/rpp_ws/config/r1_hw1_calibration.json")

class ButtonEnvironment(unittest.TestCase):
    def test_button_options_match_current_entry(self):
        script = (ROOT / "tools/starterbutton/start_navigation_high_sampling_neupan.sh").read_text()
        assignments = [line for line in script.splitlines() if re.match(r"^(scan_source|mesh_self_filter)=", line)]
        self.assertEqual(len(assignments), 2)
        command = '\n'.join(assignments) + '\nprintf "%s\\n" "$scan_source" "$mesh_self_filter"'
        for env in ({}, {"RPP_NEUPAN_SCAN_SOURCE": "livox_only"},
                    {"RPP_NEUPAN_USE_MESH_SELF_FILTER": "false"},
                    {"RPP_NEUPAN_COLLISION_CHECK_ENABLED": "true"}):
            values = subprocess.check_output(["/bin/bash", "-c", command], env=env, text=True).splitlines()
            self.assertEqual(list(button_options(env).values()), values)
        self.assertNotIn("collision_check_enabled", script)
        with self.assertRaises(ValueError): button_options({"RPP_NEUPAN_SCAN_SOURCE": "unknown"})

@unittest.skipUnless(os.environ.get("NEUPAN_UI7_PROPAGATION_TEST") == "1", "selected bundle opt-in only")
class UI7Propagation(unittest.TestCase):
    def test_ui7_propagation_without_collision_switch(self):
        from launch.actions import ExecuteProcess, OpaqueFunction
        from launch_ros.actions import Node
        with patch.object(ExecuteProcess, "execute", side_effect=AssertionError("process forbidden")), \
             patch.object(Node, "execute", side_effect=AssertionError("node forbidden")), \
             patch.object(OpaqueFunction, "execute", side_effect=AssertionError("opaque forbidden")), \
             patch("subprocess.Popen", side_effect=AssertionError("hardware probe forbidden")):
            for scan in ("dual", "livox_only"):
                for mesh in ("false", "true"):
                    result = trace(BUNDLE, CALIBRATION, {"RPP_NEUPAN_SCAN_SOURCE": scan,
                                                        "RPP_NEUPAN_USE_MESH_SELF_FILTER": mesh})
                    self.assertEqual(result["ui7_trace"]["button_options"],
                                     dict(scan_source=scan, use_mesh_self_filter=mesh))
                    guard = result["pins"]["nodes"]["/r1_collision_guard"]["parameters"]
                    self.assertNotIn("collision_check_enabled", guard)
                    self.assertFalse(result["motion_authorized"])

if __name__ == "__main__": unittest.main()
