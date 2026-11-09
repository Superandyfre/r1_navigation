"""Offline regressions for NeuPAN's learned geometry and candidate planning."""
from pathlib import Path
import numpy as np
import pytest
import torch
import yaml
from neupan import neupan
from neupan.blocks.pan import PAN
from neupan.blocks.dune import DUNE
from neupan.robot.robot import robot
from r1_neupan_integration.scan_self_filter import combine_environment_and_projection

ROOT = Path(__file__).resolve().parents[3]


def config():
    c = yaml.safe_load((ROOT / 'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml').read_text())
    c['pan']['dune_checkpoint'] = str(ROOT / 'packages/r1_neupan_integration/config/models/dune_model_5000.pth')
    c['ipath']['loop'] = False
    return c


def planner():
    c = config()
    result = neupan(**{k:v for k,v in c.items() if k not in ('robot','ipath','pan','adjust')},
        robot_kwargs=c['robot'], ipath_kwargs=c['ipath'], pan_kwargs=c['pan'], adjust_kwargs=c['adjust'])
    result.set_initial_path([np.array([[x],[0.],[0.],[1.]]) for x in np.linspace(0.,4.,41)])
    return result


def test_hw1_motion_uses_planner_direction_and_smoother_without_gear_stop():
    p=ROOT / 'packages/navigation_neupan_bridge/config/robot_r1_hw1.yaml'
    bridge=yaml.safe_load(p.read_text())['n1_16_production_bridge']['ros__parameters']
    assert bridge['allow_reverse'] is True
    assert bridge['independent_reference_direction'] is True
    assert bridge['auto_gear_transition_enabled'] is False


def test_risk_sampling_retains_near_points_and_scan_coverage():
    points = torch.stack((torch.linspace(4.,8.,1000), torch.linspace(-3.,3.,1000)))
    points[:, 321] = torch.tensor([.2,.03])
    points[:, 654] = torch.tensor([.6,-.04])
    states = torch.tensor([[0., .5, 1.], [0., 0., 0.], [0., 0., 0.]])
    indices = PAN.select_obstacle_indices(states, points, torch.zeros_like(points), 80, .25)
    assert len(indices) == 80
    assert len(torch.unique(indices)) == 80
    assert {321, 654} <= set(indices.tolist())
    assert 0 in indices and 999 in indices


def test_risk_sampling_prioritizes_rear_corner_over_nearer_center_points():
    # Side clutter is closer to the robot origin, but a rear-corner return is
    # much closer to the physical boundary and must reach the optimizer.
    c = config(); body = robot(c['receding'], c['step_time'], **c['robot'])
    side = torch.stack((torch.zeros(300), torch.full((300,), .31)))
    corner = torch.stack((torch.full((300,), -.342), torch.full((300,), .265)))
    points = torch.cat((side, corner), dim=1)
    state = torch.zeros((3, 3))
    selected = PAN.select_obstacle_indices(state, points, torch.zeros_like(points), 20, .25,
                                             robot_geometry=(body.G, body.h))
    assert int((selected >= 300).sum()) >= 15
    # Rotating the body and the scene together must preserve selection.
    rotated = torch.stack((-points[1], points[0]))
    state[2, :] = np.pi / 2
    selected_rotated = PAN.select_obstacle_indices(state, rotated, torch.zeros_like(rotated), 20, .25,
                                                     robot_geometry=(body.G, body.h))
    assert set(selected.tolist()) == set(selected_rotated.tolist())


def test_dune_dual_outputs_obey_training_constraint():
    c = config(); r = robot(c['receding'], c['step_time'], **c['robot'])
    dune = DUNE(1, checkpoint=c['pan']['dune_checkpoint'], robot=r, dune_max_num=100)
    points = torch.tensor([[.35,.4,.5,.7,1.], [0.,.1,-.2,.2,0.]], dtype=torch.float32)
    mus, _, sorted_points = dune([points, points], [torch.eye(2), torch.eye(2)], [points, points])
    for mu in mus:
        assert torch.all(mu >= 0)
        assert float(torch.linalg.vector_norm(dune.G.T @ mu,dim=0).max()) <= 1.00001
    selected = sorted_points[0]
    learned = dune.cal_objective_distance(mus[0], selected).detach().numpy()
    exact = np.hypot(np.maximum(np.abs(selected[0].numpy())-.341,0),
                     np.maximum(np.abs(selected[1].numpy())-.266,0))
    assert np.all(learned <= exact + 1e-5)


def test_arm_projection_keeps_physical_returns():
    raw = np.array([[.731, .8], [.244, .0]])
    projected = np.array([[.731, .8], [.204, .0]])
    combined = combine_environment_and_projection(raw, projected)
    np.testing.assert_array_equal(combined[:, :2], raw)
    np.testing.assert_array_equal(combined[:, 2], projected[:, 0])


@pytest.mark.parametrize('offset,expected_side', [(0., None), (.18, -1), (-.18, 1)])
def test_solver_produces_a_nonlinear_collision_free_bypass(offset, expected_side):
    core = planner()
    points = np.vstack((np.full(11,1.),np.linspace(offset-.32,offset+.32,11)))
    state = np.zeros((3,1))
    action, info = core(state, points, measured_velocity=np.zeros((2,1)))
    assert not info['stop']
    assert np.isfinite(action).all()
    if expected_side is not None:
        assert core.preferred_avoidance_side == expected_side
    velocities = info['vel_tensor'].detach().numpy()
    states = core._rollout_controls(state, velocities, subdivisions=3)
    assert np.max(np.abs(states[1])) > .25
    assert states[0,-1] > .7
    for x,y,yaw in states.T:
        dx,dy = points[0]-x,points[1]-y
        local_x = np.cos(yaw)*dx + np.sin(yaw)*dy
        local_y = -np.sin(yaw)*dx + np.cos(yaw)*dy
        assert not np.any((np.abs(local_x)<.341) & (np.abs(local_y)<.266))


def test_clear_path_needs_no_detour_candidates():
    core=planner()
    action,info=core(np.zeros((3,1)),np.array([[3.,5.,7.],[3.,3.,3.]]),measured_velocity=np.zeros((2,1)))
    assert not info['stop']
    assert action[0,0] > 0
    assert len(info['avoidance_candidates']) == 1
    assert info['avoidance_side'] == 0
from r1_neupan_integration.command_stabilizer import GearTransitionConfig, GearTransitionGate


def test_closed_loop_bypass_with_real_gear_transitions():
    core = planner()
    points = np.vstack((np.full(11, 1.), np.linspace(-.32, .32, 11)))
    state = np.zeros((3, 1))
    velocity = np.zeros((2, 1))
    gate = GearTransitionGate(GearTransitionConfig(
        mismatch_deadband_mps=.025, confirm_cycles=5,
        opposite_direction_commit_cycles=4, stopped_speed_mps=.015,
        stop_timeout_sec=2.0))
    gear = 1
    transitions = 0
    peak_lateral = 0.
    for cycle in range(120):
        action, info = core(state, points, measured_velocity=velocity)
        if info['stop']:
            gate.reset()
            command = np.zeros((2, 1))
        else:
            decision = gate.decide(
                linear_x=float(action[0, 0]), angular_z=float(action[1, 0]),
                active_gear=gear, observed_linear_x=float(velocity[0, 0]),
                odom_fresh=True, now=cycle*.1)
            command = np.array([[decision.linear_x], [decision.angular_z]])
            if decision.request_gear is not None:
                gear = decision.request_gear
                transitions += 1
                previous = (core.preferred_avoidance_side,
                            core.preferred_avoidance_mode, core.candidate_search_cursor)
                core.set_initial_path([np.array([[x], [0.], [0.], [float(gear)]])
                                       for x in np.linspace(0., 4., 41)])
                core.reset()
                (core.preferred_avoidance_side, core.preferred_avoidance_mode,
                 core.candidate_search_cursor) = previous
        velocity = command
        state = core.ipath.motion_predict_model(state, velocity, core.robot.L, .1)
        x,y,yaw = state[:, 0]
        dx,dy = points[0]-x,points[1]-y
        local_x = np.cos(yaw)*dx + np.sin(yaw)*dy
        local_y = -np.sin(yaw)*dx + np.cos(yaw)*dy
        assert not np.any((np.abs(local_x)<.341) & (np.abs(local_y)<.266)), cycle
        peak_lateral = max(peak_lateral, abs(y))
    assert state[0, 0] > 2.0, state.ravel()
    assert peak_lateral > .3
    assert transitions <= 4

def test_planner_checkpoint_restores_avoidance_direction_and_search():
    from r1_neupan_integration.n1_15_whole_core import _planner_exact_state, _restore_planner_state
    core = planner()
    core.preferred_avoidance_side = -1
    core.preferred_avoidance_mode = 'retreat'
    core.candidate_search_cursor = 3
    core.cur_vel_array[:, 0] = [-.12, .2]
    snapshot = _planner_exact_state(core)
    core.reset()
    _restore_planner_state(core, snapshot)
    assert core.preferred_avoidance_side == -1
    assert core.preferred_avoidance_mode == 'retreat'
    assert core.candidate_search_cursor == 3
    np.testing.assert_allclose(core.cur_vel_array[:, 0], [-.12, .2])

def test_reverse_path_bypasses_obstacle_without_intersection():
    core = planner()
    core.set_initial_path([np.array([[x], [0.], [0.], [-1.]]) for x in np.linspace(0., -4., 41)])
    points = np.vstack((np.full(11, -1.), np.linspace(-.32, .32, 11)))
    state = np.zeros((3, 1))
    action, info = core(state, points, measured_velocity=np.zeros((2, 1)))
    assert not info['stop']
    from r1_neupan_integration.n1_15_whole_core import _planner_semantics
    trace = _planner_semantics(core)
    for name in ('dune_selected_points_map_xy', 'nrmp_selected_points_map_xy'):
        selected = np.asarray(trace[name])
        assert selected.ndim == 2 and selected.shape[1] == 2 and len(selected) > 0
        assert np.allclose(selected[:, 0], -1.)  # the reverse wall survived selection
    states = core._rollout_controls(state, info['vel_tensor'].detach().numpy(), subdivisions=3)
    assert states[0, -1] < -.8
    assert np.max(np.abs(states[1])) > .25
    for x, y, yaw in states.T:
        dx, dy = points[0]-x, points[1]-y
        local_x = np.cos(yaw)*dx + np.sin(yaw)*dy
        local_y = -np.sin(yaw)*dx + np.cos(yaw)*dy
        assert not np.any((np.abs(local_x)<.341) & (np.abs(local_y)<.266))


def test_continuation_rechecks_new_rear_wall_and_does_not_renew_horizon():
    core=planner();state=np.zeros((3,1))
    controls=np.vstack((np.full(4,-.2),np.zeros(4)))
    clear=np.vstack((np.full(11,-2.),np.linspace(-.4,.4,11)))
    wall=np.vstack((np.full(11,-.43),np.linspace(-.4,.4,11)))
    accepted=core.validate_remaining_trajectory(state,controls,.1,clear)
    assert accepted is not None and np.allclose(accepted[0],[-.2,0.])
    assert core.validate_remaining_trajectory(state,controls,.1,wall) is None
    assert core.validate_remaining_trajectory(state,controls,1.0,clear) is None
    assert core.validate_remaining_trajectory(state,controls,.1,None) is None


def test_reverse_motion_near_rear_wall_recovers_without_body_crossing():
    # The current rearward velocity cannot reverse instantaneously. Verify
    # the actual optimized rollout, not merely its first command.
    core=planner()
    state=np.zeros((3,1)); measured=np.array([[-.2],[0.]])
    wall=np.vstack((np.full(41,-.43),np.linspace(-.5,.5,41)))
    action,info=core(state,wall,measured_velocity=measured)
    assert not info['stop'] and np.isfinite(action).all()
    controls=info['vel_tensor'].detach().numpy()
    dense=core._rollout_controls(state,controls,subdivisions=3)
    assert dense[0,-1] > 0.
    for x,y,yaw in dense.T:
        dx,dy=wall[0]-x,wall[1]-y
        local_x=np.cos(yaw)*dx+np.sin(yaw)*dy
        local_y=-np.sin(yaw)*dx+np.cos(yaw)*dy
        assert not np.any((np.abs(local_x)<.341)&(np.abs(local_y)<.266))
    from r1_neupan_integration.n1_15_whole_core import _planner_semantics
    trace=_planner_semantics(core)
    assert any(np.isclose(p[0],-.43) for p in trace['dune_selected_points_map_xy'])
    assert any(np.isclose(p[0],-.43) for p in trace['nrmp_selected_points_map_xy'])


def test_escape_feasibility_requires_improvement_without_new_overlap():
    classify = neupan._candidate_feasibility
    escaping = np.array([[-.05, .10], [-.04, .08], [-.01, .03]], dtype=np.float32)
    assert classify(escaping)[:2] == (True, True)
    worsens = np.array([[-.05, .10], [-.06, .08], [-.01, .03]], dtype=np.float32)
    assert classify(worsens)[:2] == (False, False)
    new_overlap = np.array([[-.05, .10], [-.04, -.01], [-.01, .03]], dtype=np.float32)
    assert classify(new_overlap)[:2] == (False, False)
    stationary = np.array([[-.05, .10], [-.05, .10], [-.05, .10]], dtype=np.float32)
    assert classify(stationary)[:2] == (False, False)
    masked_worsening = np.array([[-.10, -.10], [-.19, 0.], [-.19, 0.]], dtype=np.float32)
    assert classify(masked_worsening)[:2] == (False, False)


def test_projected_initial_overlap_can_escape_in_real_solver():
    core = planner()
    points = np.array([[.2185, .2185, .2247, .2247, .1901, .1901, 2.],
                       [.33, .2625, .3331, .2656, .3227, .2552, 2.]])
    state = np.zeros((3, 1))
    action, info = core(state, points, measured_velocity=np.zeros((2, 1)))
    assert not info['stop'], info
    assert np.linalg.norm(action) > 0.01
    selected = next(candidate for candidate in info['avoidance_candidates'] if candidate['selected'])
    assert selected['initial_minimum'] < 0.0
    assert selected['feasible'] and selected['escaping']
    assert selected['final_penetration'] < selected['initial_penetration']
    controls = info['vel_tensor'].detach().numpy()
    dense = core._rollout_controls(state, controls, subdivisions=3)
    assert core._candidate_feasibility(core._candidate_distances(dense, points, sample_dt=core.dt/3))[0]


def test_reverse_warm_start_can_recover_forward_and_compares_both_seeds():
    core = planner()
    core.cur_vel_array[0, :] = -.12
    # Close enough to exercise avoidance but offset so a forward route exists.
    points = np.vstack((np.full(11, 1.), np.linspace(.1, .74, 11)))
    action, info = core(np.zeros((3,1)), points, measured_velocity=np.zeros((2,1)))
    modes = {c["seed_mode"] for c in info["avoidance_candidates"]}
    assert {"forward", "reverse"} <= modes
    assert not info["stop"]
    assert action[0,0] > 0, (action, info["avoidance_candidates"])
