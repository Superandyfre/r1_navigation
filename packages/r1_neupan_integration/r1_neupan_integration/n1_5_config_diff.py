#!/usr/bin/env python3
"""Emit a machine-readable N0 official demo versus N1 planner diff."""

import argparse
import json
import os

import yaml


DEFAULTS = {
    'adjust.ro_obs': 400.0,
    'adjust.bk': 0.1,
}


def flatten(value, prefix=''):
    result = {}
    for key, child in value.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(child, dict):
            result.update(flatten(child, name))
        else:
            result[name] = child
    return result


def load(path):
    with open(path, 'r', encoding='utf-8') as stream:
        value = yaml.safe_load(stream) or {}
    flattened = flatten(value)
    for key, default in DEFAULTS.items():
        flattened.setdefault(key, default)
    return flattened


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n0', required=True)
    parser.add_argument('--n1', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    n0 = load(args.n0)
    n1 = load(args.n1)
    keys = sorted(set(n0) | set(n1))
    differences = []
    for key in keys:
        if n0.get(key) != n1.get(key):
            differences.append({
                'field': key,
                'n0': n0.get(key),
                'n1': n1.get(key),
            })
    report = {
        'n0_config': os.path.abspath(args.n0),
        'n1_config': os.path.abspath(args.n1),
        'all_fields': {
            key: {'n0': n0.get(key), 'n1': n1.get(key)} for key in keys
        },
        'differences': differences,
        'performance_relevant_fields': [
            item for item in differences
            if item['field'] in {
                'receding', 'step_time', 'pan.iter_num', 'pan.iter_threshold',
                'pan.dune_max_num', 'pan.nrmp_max_num', 'device', 'ref_speed',
                'robot.length', 'robot.width', 'robot.max_speed',
                'robot.max_acce', 'adjust.d_max', 'adjust.d_min', 'adjust.ro_obs',
                'adjust.bk',
            }
        ],
    }
    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, 'w', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
