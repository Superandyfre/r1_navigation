#!/usr/bin/env python3
"""Replay one N1.10 snapshot in a fresh NeuPAN-only process.

The snapshot is never mixed with another snapshot.  The planner velocity
state is restored before every forward so measured calls use the same saved
cycle input rather than a stateful sequence of different inputs.
"""

from __future__ import annotations

import argparse
import json
import resource
import sys
import traceback
from pathlib import Path

import numpy as np

from n1_9_core_benchmark import (
    Instrument,
    configure_runtime,
    finite_tree,
    set_path,
    sha256,
    stats,
    value_stats,
)


def metadata_from_snapshot(snapshot) -> dict:
    raw = snapshot.get('metadata_json')
    if raw is None:
        return {}
    value = raw.item() if hasattr(raw, 'item') else raw
    return json.loads(str(value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--torch-num-threads', type=int, required=True, choices=(1, 2, 4, 8))
    parser.add_argument('--torch-num-interop-threads', type=int, default=0)
    parser.add_argument('--thread-env-policy', choices=('inherit', 'match_intra'), default='inherit')
    parser.add_argument('--warmup', type=int, default=30)
    parser.add_argument('--samples', type=int, default=300)
    args = parser.parse_args()

    runtime = configure_runtime(
        args.torch_num_threads, args.torch_num_interop_threads,
        args.thread_env_policy,
    )
    result = {
        'schema': 'r1_n1_10_offline_replay_v1',
        'method': {
            'warmup': args.warmup,
            'samples': args.samples,
            'separate_process_per_snapshot': True,
            'reset_saved_velocity_before_each_forward': True,
        },
        'runtime': runtime,
        'snapshot': {
            'path': str(Path(args.snapshot).resolve()),
            'sha256': sha256(args.snapshot),
        },
        'config': {
            'path': str(Path(args.config).resolve()),
            'sha256': sha256(args.config),
        },
    }
    try:
        from neupan import neupan

        snapshot = np.load(args.snapshot, allow_pickle=False)
        metadata = metadata_from_snapshot(snapshot)
        result['online'] = {
            'snapshot_id': metadata.get('cycle_id'),
            'online_class': metadata.get('online_class'),
            'timing': metadata.get('timing'),
            'solver': metadata.get('solver'),
            'runtime': metadata.get('runtime'),
        }
        planner_yaml = {}
        import yaml
        planner_yaml = yaml.safe_load(Path(args.config).read_text(encoding='utf-8')) or {}
        pan = dict(planner_yaml.get('pan') or {})
        pan['dune_checkpoint'] = args.checkpoint
        planner = neupan.init_from_yaml(args.config, pan=pan, device='cpu')
        set_path(planner, snapshot['initial_path'].astype(float))
        state = snapshot['robot_state'].astype(float)
        points = snapshot['obstacle_points'].astype(float)
        if all(key in snapshot.files for key in (
                'pan_current_nom_s', 'pan_current_nom_u',
                'pan_current_nom_mu', 'pan_current_nom_lam')):
            from neupan.configuration import np_to_tensor
            planner.pan.current_nom_values = [
                np_to_tensor(snapshot['pan_current_nom_s'].astype(float)),
                np_to_tensor(snapshot['pan_current_nom_u'].astype(float)),
                [np_to_tensor(snapshot['pan_current_nom_mu'].astype(float))],
                [np_to_tensor(snapshot['pan_current_nom_lam'].astype(float))],
            ]
            result['input_state_compatibility'] = 'pan_current_nom_restored'
        else:
            result['input_state_compatibility'] = 'pan_current_nom_unavailable'
        if 'robot_velocity' in snapshot.files:
            saved_velocity = snapshot['robot_velocity'].astype(float)
            result['input_compatibility'] = 'n1_10_snapshot'
        else:
            # The frozen N1.9 core snapshot predates N1.10 capture fields.
            # It remains useful for a harness smoke test only.
            saved_velocity = None
            result['input_compatibility'] = 'legacy_n1_9_snapshot_zero_velocity'
        if saved_velocity is None:
            saved_velocity = np.zeros_like(planner.cur_vel_array, dtype=float)
        instrument = Instrument(planner)

        def replay_one() -> bool:
            planner.cur_vel_array = saved_velocity.copy()
            return instrument.run_one(state, points)

        for _ in range(args.warmup):
            if not replay_one():
                raise RuntimeError(
                    'warmup produced an invalid result: '
                    f'exceptions={instrument.uncaught_exceptions[-1:]}, '
                    f'nan_inf={instrument.nan_inf_outputs}'
                )
        instrument.reset()
        for _ in range(args.samples):
            replay_one()

        result['effective_pan'] = {
            'receding': planner.T,
            'iter_num': planner.pan.iter_num,
            'iter_threshold': planner.pan.iter_threshold,
            'dune_max_num': planner.pan.dune_max_num,
            'nrmp_max_num': planner.pan.nrmp_max_num,
        }
        result['timing_ms'] = {
            'dune': stats(instrument.dune),
            'nrmp': stats(instrument.nrmp),
            'pan': stats(instrument.pan),
            'full_forward': stats(instrument.full),
            'ecos_solve_wall': stats(instrument.solver_wall),
            'ecos_solve_cpu': stats(instrument.solver_cpu),
        }
        result['cpu_percent'] = value_stats(instrument.cpu_pct)
        result['validation'] = {
            'total_forwards': args.samples,
            'completed_forwards': len(instrument.full),
            'solver_calls': instrument.solver_calls,
            'ecos_success': instrument.solver_calls - instrument.solver_failures,
            'ecos_failure': instrument.solver_failures,
            'uncaught_solver_error': len(instrument.uncaught_exceptions),
            'nan_inf_outputs': instrument.nan_inf_outputs,
            'pan_iterations': value_stats(instrument.iterations),
            'exceptions': instrument.uncaught_exceptions[:10],
            'solver_events': instrument.solver_events[:20],
            'finite_outputs': all(
                finite_tree(value) for value in (instrument.full, instrument.dune, instrument.nrmp)
            ),
        }
    except Exception as exc:
        result['exception'] = {
            'type': type(exc).__name__,
            'message': str(exc),
            'traceback': traceback.format_exc(),
        }
    result['process'] = {
        'cpu_user_sec': resource.getrusage(resource.RUSAGE_SELF).ru_utime,
        'cpu_system_sec': resource.getrusage(resource.RUSAGE_SELF).ru_stime,
        'max_rss_kb': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    }
    destination = Path(args.output).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 1 if 'exception' in result else 0


if __name__ == '__main__':
    raise SystemExit(main())
