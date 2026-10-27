"""Deterministic, no-ROS closed-loop checks for the opt-in NeuPAN bypass."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import tempfile
import time

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "NeuPAN"))
sys.path.insert(0, str(ROOT / "r1_neupan_integration"))

from neupan.bypass import make_bypass_guide, signed_clearances  # noqa: E402
from neupan.neupan import neupan  # noqa: E402
from r1_neupan_integration.n1_16_bridge import ProductionWholeCoreClient  # noqa: E402
from r1_neupan_integration.n1_15_whole_core import same_terminal_pose  # noqa: E402


def planner() -> neupan:
    with (ROOT / "navigation_neupan_bridge/config/planner_r1_hw1.yaml").open() as stream:
        config = yaml.safe_load(stream)
    config.update(receding=20, bypass_enabled=True)
    config["robot"].update(
        path_relative_cost=True, geometry_collision_cost=True, geometry_margin_m=0.08,
    )
    config["adjust"]["ro_obs"] = 4000
    config["pan"]["dune_checkpoint"] = str(
        ROOT / "r1_neupan_integration/config/models/dune_model_5000.pth"
    )
    instance = neupan(
        **{key: value for key, value in config.items()
           if key not in ("robot", "ipath", "pan", "adjust")},
        robot_kwargs=config["robot"], ipath_kwargs=config["ipath"],
        pan_kwargs=config["pan"], adjust_kwargs=config["adjust"], train_kwargs={},
    )
    instance.update_initial_path_from_goal(
        np.zeros((3, 1)), np.array([[3.0], [0.0], [0.0]])
    )
    return instance


def table_points() -> np.ndarray:
    x = np.linspace(0.7, 1.35, 12)
    y = np.linspace(-0.45, 0.45, 16)
    return np.unique(np.concatenate((
        np.stack((x, np.full_like(x, -0.45))),
        np.stack((x, np.full_like(x, 0.45))),
        np.stack((np.full_like(y, 0.7), y)),
        np.stack((np.full_like(y, 1.35), y)),
    ), axis=1), axis=1)


def run_table(block_left: bool = False, close_obstacle: bool = False) -> None:
    core = planner()
    cloud = table_points()
    if close_obstacle:
        cloud[0] -= 0.22
    if block_left:
        x = np.linspace(-1.0, 2.0, 80)
        cloud = np.hstack((cloud, np.stack((x, np.full_like(x, 1.10)))))
    pose = np.zeros((3, 1))
    measured = np.zeros((2, 1))
    minimum = math.inf
    durations = []
    phases = set()
    minimum_linear = 0.0
    for step in range(460):
        core.ipath.closest_point(pose)
        started = time.perf_counter()
        action, info = core(
            pose, cloud, measured_velocity=measured, bypass_mode=True,
        )
        durations.append((time.perf_counter() - started) * 1000)
        phases.add(info["bypass_phase"])
        minimum = min(minimum, float(np.min(signed_clearances(
            pose, cloud, core.robot.vertices, core.robot.G, core.robot.h,
        ))))
        assert not info["stop"], (step, pose.ravel(), info["bypass_phase"])
        if info["arrive"]:
            break
        linear, angular = action.ravel()
        minimum_linear = min(minimum_linear, float(linear))
        assert np.isfinite(action).all()
        assert abs(linear) <= 0.201 and abs(angular) <= 0.251
        pose[0, 0] += linear * math.cos(pose[2, 0] + 0.05 * angular) * 0.1
        pose[1, 0] += linear * math.sin(pose[2, 0] + 0.05 * angular) * 0.1
        pose[2, 0] += angular * 0.1
        measured = action
    assert info["arrive"] and not info["stop"], (step, pose.ravel())
    assert np.linalg.norm(pose[:2, 0] - np.array((3.0, 0.0))) < 0.06
    assert minimum >= 0.05, minimum
    assert {"side", "pass", "rejoin", "align", "final", "done"} <= phases
    if block_left:
        assert core.bypass_side == -1
    if close_obstacle:
        assert "backoff" in phases and minimum_linear < -0.05
    label = "close table" if close_obstacle else "one-sided table" if block_left else "table"
    print(label, step + 1,
          pose.ravel(), "clearance", minimum,
          "p95_ms", np.percentile(durations, 95), "max_ms", max(durations))


def run_blocked_and_scan_loss() -> None:
    core = planner()
    x = np.linspace(-1.0, 2.0, 65)
    y = np.linspace(-0.2, 0.2, 12)
    blocked = np.concatenate((
        np.stack((x, np.full_like(x, 0.55))),
        np.stack((x, np.full_like(x, -0.55))),
        np.stack((np.full_like(y, 0.7), y)),
    ), axis=1)
    pose = np.zeros((3, 1))
    reference = np.vstack((np.linspace(0.0, 1.0, 21), np.zeros(21), np.zeros(21)))
    radius = float(np.max(np.linalg.norm(core.robot.vertices, axis=0)))
    for side in (-1, 1):
        guide = make_bypass_guide(
            pose, blocked, reference, side, radius, 0.08,
            np.array((3.0, 0.0, 0.0)), core.robot.vertices,
            core.robot.G, core.robot.h,
        )
        assert guide is None, side
    action, info = core(pose, blocked, measured_velocity=np.zeros((2, 1)),
                        bypass_mode=True)
    assert info["stop"] and not info["bypass_valid"] and not np.any(action)

    core = planner()
    action, info = core(pose, table_points(), measured_velocity=np.zeros((2, 1)),
                        bypass_mode=True)
    assert info["bypass_valid"]
    action, info = core(pose, None, measured_velocity=np.zeros((2, 1)),
                        bypass_mode=True)
    assert info["stop"] and not np.any(action) and core.bypass_guide is not None
    print("blocked corridor and scan loss: stopped")


def run_worker_smoke() -> None:
    assert same_terminal_pose(np.array([3.0, 0.0, math.pi]),
                              np.array([3.0, 0.0, -math.pi]))
    assert not same_terminal_pose(np.array([3.0, 0.0, 0.0]),
                                  np.array([3.1, 0.0, 0.0]))
    config = ROOT / "navigation_neupan_bridge/config/planner_r1_hw1.yaml"
    checkpoint = ROOT / "r1_neupan_integration/config/models/dune_model_5000.pth"
    x = np.linspace(0.0, 3.0, 31)
    path = np.vstack((x, np.zeros_like(x), np.zeros_like(x), np.ones_like(x)))
    state = np.zeros((3, 1))
    cloud = table_points()
    with tempfile.TemporaryDirectory(prefix="neupan_bypass_worker_") as output:
        client = ProductionWholeCoreClient(
            output, str(config), str(checkpoint), {}, None,
            warmup_count=1, timeout_sec=2.0,
        )
        try:
            client.startup(path, state, cloud, 1, navigation_loop_mode="terminal")
            outputs, record = client.solve(
                state, cloud, path, 1, 1, time.time(),
                goal=path[:3, -1:], goal_version=1, cycle_id=1,
                navigation_loop_mode="terminal", measured_velocity=np.zeros((2, 1)),
                bypass_mode=True,
            )
            assert record["info"]["bypass_valid"]
            assert record["info"]["bypass_phase"] == "side"
            assert np.isfinite(outputs["action"]).all()
            print("worker", record["info"]["bypass_phase"], outputs["action"].ravel(),
                  "roundtrip_ms", record["timings"]["main_full_call_ms"])
            reverse_path = path.copy()
            reverse_path[3] = -1
            outputs, record = client.solve(
                state, cloud, reverse_path, 2, 2, time.time(),
                goal=path[:3, -1:], goal_version=1, cycle_id=2,
                navigation_loop_mode="terminal", active_gear=-1,
                measured_velocity=np.zeros((2, 1)), bypass_mode=True,
            )
            assert record["info"]["bypass_phase"] == "side"
            assert np.isfinite(outputs["action"]).all()
            print("worker gear change", record["info"]["bypass_phase"],
                  outputs["action"].ravel())
            rerouted_path = path.copy()
            rerouted_path[1, 8:24] = 0.35 * np.sin(np.linspace(0.0, math.pi, 16))
            outputs, record = client.solve(
                state, cloud, rerouted_path, 3, 3, time.time(),
                goal=path[:3, -1:], goal_version=2, cycle_id=3,
                navigation_loop_mode="terminal", active_gear=1,
                measured_velocity=np.zeros((2, 1)), bypass_mode=True,
            )
            assert record["path_version"] == 3 and record["goal_version"] == 2
            assert record["info"]["bypass_valid"]
            assert record["info"]["bypass_phase"] == "side"
            print("worker same-goal replan", record["info"]["bypass_phase"],
                  outputs["action"].ravel())
            close_cloud = cloud.copy()
            close_cloud[0] -= 0.22
            new_goal_path = path.copy()
            new_goal_path[0, -1] = 3.4
            outputs, record = client.solve(
                state, close_cloud, new_goal_path, 4, 4, time.time(),
                goal=new_goal_path[:3, -1:], goal_version=3, cycle_id=4,
                navigation_loop_mode="terminal", active_gear=1,
                measured_velocity=np.zeros((2, 1)), bypass_mode=True,
            )
            assert record["info"]["bypass_phase"] == "backoff"
            assert outputs["action"][0, 0] < -0.05
            backoff_reroute = new_goal_path.copy()
            backoff_reroute[1, 8:24] = -0.35 * np.sin(np.linspace(0.0, math.pi, 16))
            outputs, record = client.solve(
                state, close_cloud, backoff_reroute, 5, 5, time.time(),
                goal=new_goal_path[:3, -1:], goal_version=4, cycle_id=5,
                navigation_loop_mode="terminal", active_gear=1,
                measured_velocity=np.zeros((2, 1)), bypass_mode=True,
            )
            assert record["info"]["bypass_phase"] == "backoff"
            assert outputs["action"][0, 0] < -0.05
            print("worker backoff replan", outputs["action"].ravel())
            reverse_path = new_goal_path.copy()
            reverse_path[3] = -1
            outputs, record = client.solve(
                state, close_cloud, reverse_path, 6, 6, time.time(),
                goal=new_goal_path[:3, -1:], goal_version=4, cycle_id=6,
                navigation_loop_mode="terminal", active_gear=-1,
                measured_velocity=np.zeros((2, 1)), bypass_mode=True,
            )
            assert record["info"]["bypass_phase"] == "backoff"
            assert outputs["action"][0, 0] < -0.05
            print("worker reverse transition", outputs["action"].ravel())
            outputs, record = client.solve(
                state, close_cloud, new_goal_path, 7, 7, time.time(),
                goal=None, goal_version=5, cycle_id=7,
                navigation_loop_mode="terminal", active_gear=1,
                measured_velocity=np.zeros((2, 1)), bypass_mode=True,
            )
            assert record["info"]["bypass_phase"] == "backoff"
            assert outputs["action"][0, 0] < -0.05
        finally:
            client.close()


if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("--worker-only", action="store_true")
    arguments = argument_parser.parse_args()
    if arguments.worker_only:
        run_worker_smoke()
    else:
        run_blocked_and_scan_loss()
        run_table()
        run_table(block_left=True)
        run_table(close_obstacle=True)
