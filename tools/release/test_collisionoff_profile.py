"""The released HW1 chain contains no external collision/braking policy."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]

class CollisionPolicyRemoved(unittest.TestCase):
    def test_guard_has_no_collision_or_braking_implementation(self):
        node = (ROOT / "packages/r1_collision_guard/src/collision_guard_node.cpp").read_text()
        self.assertNotIn("collision_check_enabled", node)
        self.assertNotIn("stopping_distance", node)
        self.assertNotIn("GuardLogic", node)
        self.assertFalse((ROOT / "packages/r1_collision_guard/src/guard_logic.cpp").exists())
        self.assertIn("authorization_->evaluate", node)

    def test_ui7_has_no_collision_switch_or_braking_policy(self):
        entry = (ROOT / "tools/starterbutton/start_navigation_high_sampling_neupan.sh").read_text()
        launch = (ROOT / "packages/navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py").read_text()
        worker = (ROOT / "packages/r1_neupan_integration/r1_neupan_integration/n1_16_node.py").read_text()
        self.assertNotIn("collision_check_enabled", entry)
        self.assertNotIn("collision_check_enabled", launch)
        self.assertNotIn("obstacle_motion_policy", worker)
        self.assertNotIn("stopping_distance", worker)

if __name__ == "__main__": unittest.main()
