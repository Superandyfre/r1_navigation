#!/usr/bin/env python3
"""Replay a static 2-D scan through NeuPAN and the real gear gate, without ROS."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml
from neupan import neupan
from r1_neupan_integration.command_stabilizer import GearTransitionConfig, GearTransitionGate
from r1_neupan_integration.scan_self_filter import combine_environment_and_projection

ROOT = Path(__file__).resolve().parents[2]


def replay(path, cycles):
    fixture = np.load(path, allow_pickle=False)
    raw = np.asarray(fixture['retained'], dtype=np.float64)
    points = combine_environment_and_projection(raw, fixture['inflated'])
    cfg = yaml.safe_load((ROOT / 'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml').read_text())
    cfg['pan']['dune_checkpoint'] = str(ROOT / 'packages/r1_neupan_integration/config/models/dune_model_5000.pth')
    cfg['ipath']['loop'] = False
    core = neupan(**{k: v for k, v in cfg.items() if k not in ('robot', 'ipath', 'pan', 'adjust')},
                  robot_kwargs=cfg['robot'], ipath_kwargs=cfg['ipath'],
                  pan_kwargs=cfg['pan'], adjust_kwargs=cfg['adjust'])
    def set_path(gear):
        core.set_initial_path([np.array([[x], [0.], [0.], [float(gear)]]) for x in np.linspace(0., 4., 41)])
    set_path(1)
    gate = GearTransitionGate(GearTransitionConfig(mismatch_deadband_mps=.025, confirm_cycles=5,
        opposite_direction_commit_cycles=4, stopped_speed_mps=.015, stop_timeout_sec=2.))
    state, velocity = np.zeros((3, 1)), np.zeros((2, 1))
    gear, changes, overlaps, times, trail, reasons = 1, 0, [], [], [], {}
    for cycle in range(cycles):
        started = time.perf_counter()
        action, info = core(state, points, measured_velocity=velocity)
        times.append(time.perf_counter() - started)
        if info['stop']:
            gate.reset()
            command = np.zeros((2, 1))
            reason = info['stop_reason']
        else:
            decision = gate.decide(linear_x=float(action[0, 0]), angular_z=float(action[1, 0]),
                active_gear=gear, observed_linear_x=float(velocity[0, 0]), odom_fresh=True, now=cycle*.1)
            command = np.array([[decision.linear_x], [decision.angular_z]])
            reason = decision.reason
            if decision.request_gear is not None:
                gear = decision.request_gear
                changes += 1
                context = (core.preferred_avoidance_side, core.preferred_avoidance_mode,
                           core.candidate_search_cursor)
                set_path(gear)
                core.reset()
                (core.preferred_avoidance_side, core.preferred_avoidance_mode,
                 core.candidate_search_cursor) = context
        reasons[reason] = reasons.get(reason, 0) + 1
        velocity = command
        state = core.ipath.motion_predict_model(state, velocity, core.robot.L, .1)
        x, y, yaw = state[:, 0]
        dx, dy = raw[0] - x, raw[1] - y
        px, py = np.cos(yaw)*dx + np.sin(yaw)*dy, -np.sin(yaw)*dx + np.cos(yaw)*dy
        inside = ((np.abs(px) < float(cfg["robot"]["length"])/2) &
                  (np.abs(py) < float(cfg["robot"]["width"])/2))
        if np.any(inside): overlaps.append(cycle)
        trail.append(state[:, 0].tolist())
        if overlaps: break
    return dict(schema='neupan_static_scan_replay_v1', physical_motion=False,
        assumptions=['static scan and fixed straight reference in base coordinates',
                     'ideal odometry and commands; no velocity smoother or actuator dynamics',
                     'rectangle-only chassis overlap checked at 0.1-second samples; arms not checked'],
        scan=str(path), raw_points=raw.shape[1], planner_points=points.shape[1],
        cycles=len(trail), final_state=state[:, 0].tolist(), peak_lateral_m=max(abs(row[1]) for row in trail),
        gear_transitions=changes, chassis_overlap_cycles=overlaps,
        solve_ms_p95=float(np.percentile(times, 95)*1000), solve_ms_max=max(times)*1000,
        reasons=reasons, trajectory=trail)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan', type=Path, help='NPZ containing retained and inflated 2xN arrays')
    parser.add_argument('--cycles', type=int, default=140)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.cycles < 1: parser.error('cycles must be positive')
    result = replay(args.scan, args.cycles)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('cycles','final_state','peak_lateral_m',
        'gear_transitions','chassis_overlap_cycles','solve_ms_p95','solve_ms_max')}, indent=2))
    return 1 if result['chassis_overlap_cycles'] else 0

if __name__ == '__main__':
    raise SystemExit(main())
