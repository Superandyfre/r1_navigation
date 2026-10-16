#!/usr/bin/env python3
"""Exercise the production worker's obstacle-speed request contract."""

from pathlib import Path
import tempfile

import numpy as np
import yaml

from r1_neupan_integration.n1_16_bridge import ProductionWholeCoreClient
from r1_neupan_integration.local_detour import LocalDetourPlanner
from r1_neupan_integration.obstacle_motion_policy import MotionPolicyConfig, plan_obstacle_speed


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    config = root / "navigation_neupan_bridge/config/planner_r1_hw1.yaml"
    with config.open(encoding="utf-8") as stream:
        horizon = int(yaml.safe_load(stream)["receding"])
    model = root / "r1_neupan_integration/config/models/dune_model_5000.pth"
    path = np.vstack((
        np.linspace(0.0, 3.0, 31), np.zeros(31), np.zeros(31), np.ones(31),
    ))
    state = np.asarray([[0.0], [0.0], [0.0]])
    points = np.asarray([[0.65, 0.66, 0.65], [0.0, 0.05, -0.05]])
    profile = np.interp(
        np.linspace(0.0, 5.0, horizon), np.arange(6),
        np.asarray([0.18, 0.15, 0.12, 0.08, 0.04, 0.0]),
    )
    with tempfile.TemporaryDirectory(prefix="r1_obstacle_worker_") as directory:
        client = ProductionWholeCoreClient(
            directory, str(config), str(model),
            {"iter_num": 2, "dune_max_num": 200, "nrmp_max_num": 10},
            None, torch_intraop=4, torch_interop=2,
            warmup_count=1, timeout_sec=0.25,
        )
        try:
            client.startup(path, state, points, 1, navigation_loop_mode="terminal")
            outputs, record = client.solve(
                state, points, path, 1, 1, 1.0,
                goal=path[:3, -1:], goal_version=1, cycle_id=1,
                navigation_loop_mode="terminal",
                reference_speed_profile=profile,
                measured_velocity=np.asarray([[0.20], [0.0]]),
            )
            assert record["finite"]
            assert np.isfinite(outputs["action"]).all()
            assert np.allclose(outputs["ref_speed"].reshape(-1), profile, atol=1.0e-8)
            assert outputs["optimized_velocity"][0, 0] <= 0.181
            assert record["timings"]["worker_core_total_ms"] < 250.0

            motion_config = MotionPolicyConfig()
            detour = LocalDetourPlanner(motion_config).update(
                state, path, points, points,
                plan_obstacle_speed(state, path, points, points, motion_config),
                1, 0.0,
            )
            assert detour is not None
            detour_outputs, detour_record = client.solve(
                state, points, path, 1, 2, 2.0,
                goal=path[:3, -1:], goal_version=1, cycle_id=2,
                navigation_loop_mode="terminal",
                reference_speed_profile=np.full(horizon, 0.16),
                measured_velocity=np.zeros((2, 1)),
                local_reference_path=detour.path,
                local_reference_id=detour.identifier,
            )
            assert detour_record["finite"]
            assert np.max(np.abs(detour_outputs["ref_state"][1])) > 0.01
            assert abs(detour_outputs["action"][1, 0]) > 0.001
            restored, restored_record = client.solve(
                state, np.empty((2, 0)), path, 1, 3, 3.0,
                goal=path[:3, -1:], goal_version=1, cycle_id=3,
                navigation_loop_mode="terminal",
                reference_speed_profile=np.full(horizon, 0.16),
                measured_velocity=np.zeros((2, 1)),
            )
            assert restored_record["finite"]
            assert np.max(np.abs(restored["ref_state"][1])) < 1.0e-6
        finally:
            client.close()
    print("obstacle_motion_worker_unit.py PASS")


if __name__ == "__main__":
    main()
