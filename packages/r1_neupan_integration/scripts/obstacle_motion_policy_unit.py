#!/usr/bin/env python3
"""Check obstacle-aware reference speed and the NeuPAN first-step bounds."""

from dataclasses import replace
import numpy as np

from r1_neupan_integration.obstacle_motion_policy import (
    MotionPolicyConfig,
    first_command_collision_distance,
    first_swept_collision_distance,
    plan_obstacle_speed,
)
from r1_neupan_integration.local_detour import (
    LocalDetourController, LocalDetourPlanner, ReverseEscapePlanner,
)


def check_geometry() -> None:
    config = MotionPolicyConfig(stop_margin_m=0.20)
    state = np.asarray([[0.0], [0.0], [0.0]])
    path = np.vstack((np.linspace(0.0, 1.0, 21), np.zeros(21), np.zeros(21)))
    empty = np.empty((2, 0))
    clear = plan_obstacle_speed(state, path, empty, empty, config)
    assert clear.reason == "route_clear"
    assert np.allclose(clear.speed_profile, 0.20)

    roadside = np.asarray([[0.60, 0.80], [0.40, -0.40]])
    assert np.isinf(first_swept_collision_distance(state, path, roadside, config))
    assert not plan_obstacle_speed(state, path, roadside, roadside, config).route_blocked

    # A smoothed command may turn toward a point outside the reference path.
    turning_obstacle = np.asarray([[0.50], [0.35]])
    assert not plan_obstacle_speed(
        state, path, turning_obstacle, turning_obstacle, config
    ).route_blocked
    assert first_command_collision_distance(
        state, 0.20, 0.25, turning_obstacle, config
    ) < 0.30

    ahead = np.asarray([[0.60], [0.0]])
    blocked = plan_obstacle_speed(state, path, ahead, ahead, config)
    assert 0.258 <= blocked.free_distance_m <= 0.261
    assert blocked.route_blocked
    assert 0.0 < blocked.speed_profile[0] < 0.20
    assert np.all(np.diff(blocked.speed_profile) <= 1.0e-9)

    further = np.asarray([[0.70], [0.0]])
    slowing = plan_obstacle_speed(state, path, further, further, config)
    assert 0.0 < slowing.speed_profile[0] <= 0.20
    assert slowing.speed_profile[0] > blocked.speed_profile[0]

    close = np.asarray([[0.54], [0.0]])
    stopped = plan_obstacle_speed(state, path, close, close, config)
    assert 0.198 <= stopped.free_distance_m <= 0.201
    assert stopped.speed_profile[0] == 0.0
    assert np.all(stopped.speed_profile == 0.0)

    # A 15 cm side gap remains traversable; a scan clearing restores speed.
    narrow_side = np.asarray([[0.1, 0.6], [0.42, -0.42]])
    assert not plan_obstacle_speed(state, path, narrow_side, narrow_side, config).route_blocked
    assert np.allclose(plan_obstacle_speed(state, path, empty, empty, config).speed_profile, 0.20)

    rear = np.asarray([[-0.48], [0.0]])
    assert first_command_collision_distance(state, -0.05, 0.0, rear, config) < 0.20
    assert np.isinf(first_command_collision_distance(state, 0.05, 0.0, rear, config))
    assert np.isinf(first_command_collision_distance(state, -0.05, 0.0, roadside, config))


def check_neupan() -> None:
    from neupan import neupan

    planner = neupan(
        receding=6, step_time=0.25, ref_speed=0.20, device="cpu",
        robot_kwargs={
            "kinematics": "diff", "max_speed": [0.20, 0.25],
            "max_acce": [0.30, 0.50], "length": 0.682, "width": 0.532,
            "smooth_weight": 0.05,
        },
        ipath_kwargs={"loop": False, "interval": 0.1, "min_radius": 0.242},
        pan_kwargs={"iter_num": 1, "dune_max_num": 0, "nrmp_max_num": 0},
        adjust_kwargs={"q_s": [8.0, 10.0, 2.5], "p_u": 1.4},
        train_kwargs={},
    )
    path = [np.asarray([[x], [0.0], [0.0], [1.0]]) for x in np.linspace(0, 2, 21)]
    planner.set_initial_path(path)
    state = np.asarray([[0.0], [0.0], [0.0]])
    profile = np.asarray([0.16, 0.12, 0.08, 0.04, 0.0, 0.0])
    action, info = planner(
        state, None, reference_speed_profile=profile,
        measured_velocity=np.asarray([[0.20], [0.0]]),
    )
    velocity = info["vel_tensor"].detach().cpu().numpy()
    assert action.shape == (2, 1)
    assert np.isfinite(velocity).all()
    assert 0.12 <= velocity[0, 0] <= 0.165
    assert np.all(velocity[0] <= np.maximum(profile, 0.20 - 0.30 * 0.25 * np.arange(1, 7)) + 1.0e-3)
    assert np.allclose(info["ref_speed_tensor"].detach().cpu().numpy(), profile, atol=1.0e-9)

    import cvxpy as cp

    planner.robot.para_speed_limits.value = np.zeros(6)
    planner.robot.para_measured_u.value = np.zeros((2, 1))
    planner.robot.para_first_step_bound.value = np.asarray([[0.08], [0.13]])
    planner.robot.para_s.value = np.zeros((3, 7))
    problem = cp.Problem(
        cp.Minimize(planner.robot.indep_u[0, 0]),
        planner.robot.bound_su_constraints(),
    )
    problem.solve(solver=cp.OSQP)
    assert problem.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE)
    assert planner.robot.indep_u.value[0, 0] < -0.05


def check_local_detour() -> None:
    config = MotionPolicyConfig()
    state = np.asarray([[0.0], [0.0], [0.0]])
    path = np.vstack((
        np.linspace(0.0, 3.0, 61), np.zeros(61), np.zeros(61), np.ones(61),
    ))
    obstacle = np.asarray([[0.60, 0.60, 0.65], [0.00, 0.05, -0.05]])
    decision = plan_obstacle_speed(state, path, obstacle, obstacle, config)
    detours = LocalDetourPlanner(config)
    detour = detours.update(state, path, obstacle, obstacle, decision, 1, 0.0)
    assert detour is not None
    assert detour.path.shape[0] == 4
    assert abs(detour.path[1]).max() > 0.55
    assert np.isinf(first_swept_collision_distance(state, detour.path, obstacle, config))
    assert detours.update(state, path, obstacle, obstacle, decision, 1, 0.1).identifier == detour.identifier
    assert detours.update(state, path, obstacle, obstacle, decision, 2, 0.2) is not None

    persistent = LocalDetourPlanner(config)
    persistent_detour = persistent.update(
        state, path, obstacle, obstacle, decision, 1, 0.0
    )
    assert persistent_detour is not None
    empty = np.empty((2, 0))
    clear_decision = plan_obstacle_speed(state, path, empty, empty, config)
    for cycle in range(4):
        assert persistent.update(
            state, path, obstacle, obstacle, clear_decision,
            1, 0.1 * (cycle + 1),
        ).identifier == persistent_detour.identifier
    assert persistent.update(
        state, path, obstacle, obstacle, clear_decision,
        2, 0.6,
    ).identifier == persistent_detour.identifier
    new_goal_path = path.copy()
    new_goal_path[0, -1] += 0.50
    new_goal_detour = persistent.update(
        state, new_goal_path, obstacle, obstacle, decision,
        3, 0.7,
    )
    assert new_goal_detour is not None
    assert new_goal_detour.identifier != persistent_detour.identifier

    early_obstacle = np.asarray([[1.40], [0.0]])
    long_preview = replace(config, preview_m=1.50)
    assert not plan_obstacle_speed(
        state, path, early_obstacle, early_obstacle, config
    ).route_blocked
    early_decision = plan_obstacle_speed(
        state, path, early_obstacle, early_obstacle, long_preview
    )
    assert early_decision.route_blocked
    assert LocalDetourPlanner(long_preview).update(
        state, path, early_obstacle, early_obstacle,
        early_decision, 1, 0.0,
    ) is not None

    short_route = path[:, :26]
    short_detour = LocalDetourPlanner(config).update(
        state, short_route, obstacle, obstacle,
        plan_obstacle_speed(state, short_route, obstacle, obstacle, config), 1, 0.0,
    )
    assert short_detour is not None
    assert np.allclose(short_detour.join_xy, short_route[:2, -1])
    short_pose = state.copy()
    short_controller = LocalDetourController()
    short_phases = set()
    for _ in range(500):
        linear, angular, phase = short_controller.command(short_pose, short_detour)
        short_phases.add(phase)
        stopping_distance = (
            abs(linear) * config.reaction_sec
            + linear ** 2 / (2.0 * config.braking_mps2)
            + config.stop_margin_m
        )
        clearance = first_command_collision_distance(
            short_pose, linear, angular, obstacle,
            replace(config, preview_m=max(0.30, stopping_distance + 0.10)),
        )
        assert clearance >= stopping_distance, (
            phase, short_pose.reshape(-1).tolist(), linear, angular, clearance
        )
        yaw_mid = short_pose[2, 0] + 0.05 * angular
        short_pose[0, 0] += 0.10 * linear * np.cos(yaw_mid)
        short_pose[1, 0] += 0.10 * linear * np.sin(yaw_mid)
        short_pose[2, 0] += 0.10 * angular
        if np.linalg.norm(short_pose[:2, 0] - short_detour.join_xy) < 0.10:
            break
    assert short_phases == {"TURN_SIDE", "PASS", "REJOIN"}
    assert np.linalg.norm(short_pose[:2, 0] - short_detour.join_xy) < 0.10
    assert LocalDetourPlanner(config).update(
        state, path[:, :19], obstacle, obstacle,
        plan_obstacle_speed(state, path[:, :19], obstacle, obstacle, config), 1, 0.0,
    ) is None

    intruder = np.hstack((obstacle, detour.side_xy.reshape(2, 1)))
    revised = detours.update(
        state, path, intruder, intruder,
        plan_obstacle_speed(state, path, intruder, intruder, config), 2, 0.3,
    )
    assert revised is None or revised.identifier != detour.identifier
    detours.reset()
    detour = detours.update(state, path, obstacle, obstacle, decision, 2, 0.4)
    assert detour is not None

    simulated = state.copy()
    controller = LocalDetourController()
    passed_side = False
    for cycle in range(500):
        original_decision = plan_obstacle_speed(simulated, path, obstacle, obstacle, config)
        if cycle % 12 in (4, 5, 6, 7):
            original_decision = plan_obstacle_speed(simulated, path, empty, empty, config)
        tracked = detours.update(
            simulated, path, obstacle, obstacle, original_decision,
            2 + cycle // 30, cycle * 0.1,
        )
        if tracked is None:
            break
        assert not plan_obstacle_speed(
            simulated, tracked.path, obstacle, obstacle, config
        ).route_blocked
        linear, angular, phase = controller.command(simulated, detour)
        stopping_distance = (
            abs(linear) * config.reaction_sec
            + linear ** 2 / (2.0 * config.braking_mps2)
            + config.stop_margin_m
        )
        clearance = first_command_collision_distance(
            simulated, linear, angular, obstacle,
            replace(config, preview_m=max(0.30, stopping_distance + 0.10)),
        )
        assert np.isinf(clearance), (
            simulated.reshape(-1).tolist(), phase, linear, angular, clearance
        )
        yaw_mid = simulated[2, 0] + 0.05 * angular
        simulated[0, 0] += 0.10 * linear * np.cos(yaw_mid)
        simulated[1, 0] += 0.10 * linear * np.sin(yaw_mid)
        simulated[2, 0] += 0.10 * angular
        passed_side |= phase == "PASS"
        if np.linalg.norm(simulated[:2, 0] - detour.join_xy) < 0.10:
            break
    assert passed_side
    assert simulated[0, 0] > 1.50
    assert abs(simulated[1, 0]) < 0.15
    joined = np.asarray([
        [detour.join_xy[0]], [detour.join_xy[1]], [0.0],
    ])
    assert detours.update(
        joined, path, obstacle, obstacle,
        plan_obstacle_speed(joined, path, obstacle, obstacle, config),
        2, 60.0,
    ) is None

    left_blocked = np.hstack((obstacle, np.asarray([[0.40], [0.65]])))
    right_detour = LocalDetourPlanner(config).update(
        state, path, left_blocked, left_blocked,
        plan_obstacle_speed(state, path, left_blocked, left_blocked, config), 1, 0.0,
    )
    assert right_detour is not None and right_detour.side == -1

    # A roadside point farther ahead must not extend the blocking object's
    # length until the remaining route appears too short for either bypass.
    short_path = path[:, :31]
    roadside_ahead = np.hstack((obstacle, np.asarray([[1.20], [0.60]])))
    short_planner = LocalDetourPlanner(config)
    roadside_decision = plan_obstacle_speed(
        state, short_path, roadside_ahead, roadside_ahead, config
    )
    roadside_detour = short_planner.update(
        state, short_path, roadside_ahead, roadside_ahead,
        roadside_decision, 1, 0.0,
    )
    assert roadside_detour is not None and roadside_detour.side == -1
    assert short_planner.last_status["sides"]["right"] == "feasible"
    assert np.isinf(first_swept_collision_distance(
        state, roadside_detour.path, roadside_ahead, config
    ))

    # Neither side of a narrow aisle is a detour; the blocker remains a stop.
    walls = np.asarray([[0.3, 0.6, 0.3, 0.6], [0.65, 0.65, -0.65, -0.65]])
    crowded = np.hstack((obstacle, walls))
    narrow_planner = LocalDetourPlanner(config)
    assert narrow_planner.update(
        state, path, crowded, crowded,
        plan_obstacle_speed(state, path, crowded, crowded, config), 1, 0.0,
    ) is None
    assert narrow_planner.last_status["state"] == "no_feasible_side"

    close = np.asarray([[0.43], [0.0]])
    blocked = plan_obstacle_speed(state, path, close, close, config)
    escape = ReverseEscapePlanner(config)
    assert escape.update(state, 0.0, close, empty, blocked, False, 1, 1, 0.0)[0] is None
    assert escape.update(state, 0.0, close, empty, blocked, False, 1, 1, 1.6)[0] == -0.10
    backed = state.copy()
    backed[0, 0] = -0.41
    assert escape.update(backed, 0.0, close, empty, blocked, False, -1, 1, 3.0)[0] == 0.06
    assert escape.update(backed, 0.0, close, empty, blocked, False, 1, 1, 4.0)[0] is None
    assert escape.update(backed, 0.0, close, empty, blocked, False, 1, 1, 6.0)[0] is None
    escape = ReverseEscapePlanner(config)
    assert escape.update(state, 0.0, close, empty, blocked, False, 1, 2, 0.0)[1] == "WAITING"
    early_blocker = np.asarray([[0.75], [0.0]])
    early_decision = plan_obstacle_speed(
        state, path, early_blocker, early_blocker, config
    )
    assert early_decision.speed_profile[0] > 0.03
    early_escape = ReverseEscapePlanner(config)
    assert early_escape.update(
        state, 0.0, early_blocker, empty, early_decision, False, 1, 4, 0.0
    )[1] == "WAITING"
    assert early_escape.update(
        state, 0.0, early_blocker, empty, early_decision, False, 1, 4, 1.6
    )[1] == "BACKING"
    clear = plan_obstacle_speed(state, path, empty, empty, config)
    reverse_context = ReverseEscapePlanner(config)
    assert reverse_context.update(
        state, 0.0, empty, empty, clear, False, -1, 5, 0.0
    ) == (None, "NOT_BLOCKED")
    assert escape.update(state, 0.0, empty, empty, clear, False, 1, 2, 1.0)[1] == "NOT_BLOCKED"
    assert escape.update(state, 0.0, close, empty, blocked, False, 1, 2, 1.1)[1] == "WAITING"
    assert escape.update(state, 0.0, close, empty, blocked, False, 1, 2, 2.0)[1] == "WAITING"
    assert escape.update(state, 0.0, close, empty, blocked, False, 1, 2, 2.7)[1] == "BACKING"
    assert escape.update(state, 0.0, close, empty, blocked, False, -1, 3, 2.8) == (
        0.06, "RETURN_FORWARD_GEAR"
    )
    rear_blocked = np.asarray([[-0.45], [0.0]])
    escape = ReverseEscapePlanner(config)
    escape.update(state, 0.0, rear_blocked, empty, blocked, False, 1, 1, 0.0)
    assert escape.update(state, 0.0, rear_blocked, empty, blocked, False, 1, 1, 2.0)[1] == "REAR_BLOCKED"


def main() -> None:
    check_geometry()
    check_neupan()
    check_local_detour()
    print("obstacle_motion_policy_unit.py PASS")


if __name__ == "__main__":
    main()
