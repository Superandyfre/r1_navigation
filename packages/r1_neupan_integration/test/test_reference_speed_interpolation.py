"""Reference speed must not be quantized by waypoint spacing."""
from types import SimpleNamespace
from pathlib import Path
import os
import yaml
import unittest
import numpy as np
from neupan.blocks.initial_path import InitialPath


class ReferenceSpeedInterpolation(unittest.TestCase):
    def planner(self, endpoint=3.0):
        p = InitialPath.__new__(InitialPath)
        p.T = 6
        p.dt = .25
        p.interval = .1
        p.robot = SimpleNamespace(L=1.)
        p.curve_list = [[np.array([[x], [0.], [0.], [1.]])
                         for x in np.linspace(0., endpoint, int(round(endpoint / .1)) + 1)]]
        p.curve_index = 0
        p.point_index = 0
        p.motion_predict_model = lambda state, velocity, length, dt: state.copy()
        return p

    def test_one_metre_per_second_with_fractional_waypoint_steps(self):
        p = self.planner()
        _, _, ref, speed = p.generate_nom_ref_state(np.zeros((3, 1)), np.zeros((2, 6)), 1.)
        np.testing.assert_allclose(ref[0], np.arange(7) * .25, atol=1e-9)
        np.testing.assert_allclose(speed, np.ones(6))

    def test_endpoint_does_not_extrapolate(self):
        p = self.planner(.5)
        _, _, ref, speed = p.generate_nom_ref_state(np.zeros((3, 1)), np.zeros((2, 6)), 1.)
        self.assertLessEqual(float(np.max(ref[0])), .5 + 1e-9)
        self.assertEqual(float(speed[-1]), 0.)

    def test_zero_reference_does_not_move(self):
        p = self.planner()
        _, _, ref, speed = p.generate_nom_ref_state(
            np.zeros((3, 1)), np.zeros((2, 6)), 0.)
        np.testing.assert_allclose(ref, 0.)
        np.testing.assert_allclose(speed, 0.)

@unittest.skipUnless(os.environ.get("NEUPAN_SOLVER_REGRESSION") == "1", "actual solver opt-in")
class ActualReferenceSpeed(unittest.TestCase):
    def test_clear_path_can_track_one_metre_per_second(self):
        from neupan import neupan
        root = Path(__file__).resolve().parents[3]
        cfg = yaml.safe_load((root / "packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml").read_text())
        cfg["pan"]["dune_checkpoint"] = str(root / "packages/r1_neupan_integration/config/models/dune_model_5000.pth")
        cfg["ipath"]["loop"] = False
        core = neupan(**{k: v for k, v in cfg.items() if k not in ("robot", "ipath", "pan", "adjust")},
            robot_kwargs=cfg["robot"], ipath_kwargs=cfg["ipath"],
            pan_kwargs=cfg["pan"], adjust_kwargs=cfg["adjust"])
        state = np.zeros((3, 1))
        core.update_initial_path_from_goal(state, np.array([[10.], [0.], [0.]]))
        velocity = np.zeros((2, 1))
        obstacles = np.array([[3., 5., 7.], [3., 3., 3.]])
        for _ in range(18):
            velocity, info = core(state, obstacles, measured_velocity=velocity)
            self.assertFalse(info["stop"])
            state[0, 0] += float(velocity[0, 0]) * .25
            state[2, 0] += float(velocity[1, 0]) * .25
        self.assertGreater(float(velocity[0, 0]), .95)
        self.assertLessEqual(float(velocity[0, 0]), 1.001)


if __name__ == "__main__":
    unittest.main()
