#!/usr/bin/env python3
"""Run a same-input NeuPAN core versus wrapper-conversion A/B experiment."""

import argparse
import json
import os
import time

import numpy as np
import yaml
from neupan import neupan
from neupan.util import get_transform


def summary(values):
    values = sorted(float(value) for value in values)
    if not values:
        return {'count': 0}
    index = min(len(values) - 1, int(np.ceil(0.95 * len(values)) - 1))
    return {
        'count': len(values),
        'mean_ms': float(np.mean(values)),
        'median_ms': float(np.median(values)),
        'p95_ms': values[index],
        'max_ms': max(values),
    }


def load_planner(config_file, checkpoint_file, initial_path):
    with open(config_file, 'r', encoding='utf-8') as stream:
        planner_yaml = yaml.safe_load(stream) or {}
    pan = dict(planner_yaml.get('pan') or {})
    pan['dune_checkpoint'] = checkpoint_file
    planner = neupan.init_from_yaml(
        config_file,
        pan=pan,
    )
    planner.set_initial_path([
        np.vstack([initial_path[:, index:index + 1], [[1.0]]])
        for index in range(initial_path.shape[1])
    ])
    return planner


def wrapper_convert(snapshot):
    ranges = snapshot['scan_ranges'].astype(float)
    if ranges.size == 0:
        return snapshot['obstacle_points'].copy()
    angles = np.linspace(
        float(snapshot['scan_angle_min']),
        float(snapshot['scan_angle_max']),
        len(ranges),
    )
    valid = (
        (ranges >= float(snapshot['scan_range_min']))
        & (ranges <= float(snapshot['scan_range_max']))
        & (angles > -np.pi)
        & (angles < np.pi)
    )
    valid_ranges = ranges[valid]
    valid_angles = angles[valid]
    points = np.vstack([
        valid_ranges * np.cos(valid_angles),
        valid_ranges * np.sin(valid_angles),
    ])
    trans, rotation = get_transform(snapshot['robot_state'].reshape(3, 1))
    return rotation @ points + trans


def run_calls(planner, state, points, warmup, samples):
    for _ in range(warmup):
        planner(state, points)
    times = []
    for _ in range(samples):
        started = time.perf_counter()
        planner(state, points)
        times.append((time.perf_counter() - started) * 1000.0)
    return summary(times)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--samples', type=int, default=30)
    args = parser.parse_args()

    snapshot = np.load(args.snapshot)
    state = snapshot['robot_state'].astype(float)
    points = snapshot['obstacle_points'].astype(float)
    initial_path = snapshot['initial_path'].astype(float)

    core_planner = load_planner(args.config, args.checkpoint, initial_path)
    wrapper_planner = load_planner(args.config, args.checkpoint, initial_path)

    converted_points = wrapper_convert(snapshot)
    conversion_times = []
    for _ in range(args.warmup):
        wrapper_convert(snapshot)
    for _ in range(args.samples):
        started = time.perf_counter()
        converted_points = wrapper_convert(snapshot)
        conversion_times.append((time.perf_counter() - started) * 1000.0)

    core_only = run_calls(
        core_planner, state, points, args.warmup, args.samples
    )
    wrapper_integrated = []
    for _ in range(args.warmup):
        wrapper_planner(state, converted_points)
    for _ in range(args.samples):
        started = time.perf_counter()
        wrapper_planner(state, converted_points)
        wrapper_integrated.append((time.perf_counter() - started) * 1000.0)

    report = {
        'snapshot': os.path.abspath(args.snapshot),
        'inputs': {
            'robot_state_shape': list(state.shape),
            'captured_obstacle_points': int(points.shape[1]),
            'converted_obstacle_points': int(converted_points.shape[1]),
            'path_points': int(initial_path.shape[1]),
            'scan_ranges': int(snapshot['scan_ranges'].size),
        },
        'method': {
            'warmup': args.warmup,
            'samples': args.samples,
            'core_only': 'same saved wrapper-converted points -> NeuPAN core',
            'wrapper_integrated': 'same saved raw LaserScan -> wrapper conversion -> NeuPAN core',
        },
        'timing': {
            'core_only_ms': core_only,
            'wrapper_conversion_only_ms': summary(conversion_times),
            'wrapper_integrated_core_ms': summary(wrapper_integrated),
        },
    }
    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, 'w', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
