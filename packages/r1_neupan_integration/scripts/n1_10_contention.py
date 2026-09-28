#!/usr/bin/env python3
"""Run one offline snapshot replay while adding bounded system load."""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import signal
import subprocess
import time

from n1_6_benchmark import cleanup_world


ROOT = pathlib.Path(__file__).resolve().parents[3]
PYTHON = '/home/pinggu/venvs/neupan_ros_jazzy/bin/python'
WORLD = ROOT / 'install' / 'r1_gazebo' / 'share' / 'r1_gazebo' / 'worlds' / 'navigation_test.sdf'


def stop_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass


def load_commands(mode: str) -> list[list[str]]:
    if mode == 'b0':
        return []
    if mode == 'b1':
        return [['gz', 'sim', '-r', '-s', str(WORLD)]]
    launch = (
        'source /opt/ros/jazzy/setup.bash && '
        'source install/setup.bash && '
        'exec ros2 launch r1_gazebo r1_nav_sim.launch.py '
        'headless:=true world=' + str(WORLD) + ' rviz:=false '
        'lidar_profile:=' + ('lidar_smoke' if mode == 'b2' else 'lidar_navigation')
    )
    commands = [['bash', '-lc', launch]]
    if mode == 'b3':
        commands.append([
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash && source install/setup.bash && '
            'exec ros2 launch r1_lidar_preprocess pointcloud_to_scan.launch.py '
            'profile:=lidar_navigation input_topic:=/lidar/points '
            'output_topic:=/contention_scan use_sim_time:=true',
        ])
    return commands


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('b0', 'b1', 'b2', 'b3'), required=True)
    parser.add_argument('--snapshot', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--load-log-dir', required=True)
    parser.add_argument('--affinity', default='')
    parser.add_argument('--load-affinity', default='')
    parser.add_argument('--startup-wait-sec', type=float, default=12.0)
    args = parser.parse_args()

    output = pathlib.Path(args.output).resolve()
    log_dir = pathlib.Path(args.load_log_dir).resolve()
    log_dir.mkdir(parents=True, exist_ok=True)
    loads = []
    replay = None
    try:
        commands = load_commands(args.mode)
        if args.load_affinity:
            commands = [['taskset', '-c', args.load_affinity] + command for command in commands]
        for index, command in enumerate(commands):
            log = (log_dir / f'{args.mode}_load_{index}.log').open('w', encoding='utf-8')
            loads.append(subprocess.Popen(
                command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            ))
        if loads:
            time.sleep(args.startup_wait_sec)
        replay_command = [PYTHON, str(ROOT / 'src/r1_neupan_integration/scripts/n1_10_replay.py'),
                          '--snapshot', args.snapshot, '--config', args.config,
                          '--checkpoint', args.checkpoint, '--torch-num-threads', '1',
                          '--torch-num-interop-threads', '0', '--thread-env-policy', 'match_intra',
                          '--warmup', '30', '--samples', '300', '--output', str(output)]
        if args.affinity:
            replay_command = ['taskset', '-c', args.affinity] + replay_command
        replay = subprocess.run(replay_command, cwd=ROOT, check=False, text=True,
                                capture_output=True)
    finally:
        if replay is not None and replay.stdout:
            (log_dir / f'{args.mode}_replay.stdout').write_text(replay.stdout, encoding='utf-8')
        for process in reversed(loads):
            stop_group(process)
        if args.mode in ('b2', 'b3'):
            cleanup_world('obstacle')
    result = json.loads(output.read_text(encoding='utf-8')) if output.is_file() else {
        'exception': 'replay output missing',
    }
    result['contention'] = {
        'mode': args.mode,
        'affinity': args.affinity or 'none',
        'load_affinity': args.load_affinity or 'none',
        'load_commands': commands if 'commands' in locals() else [],
        'load_pids': [process.pid for process in loads],
        'startup_wait_sec': args.startup_wait_sec if loads else 0.0,
    }
    output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps({
        'mode': args.mode,
        'affinity': args.affinity or 'none',
        'returncode': replay.returncode if replay is not None else None,
        'nrmp': (result.get('timing_ms') or {}).get('nrmp'),
        'full': (result.get('timing_ms') or {}).get('full_forward'),
    }, indent=2))
    return 0 if replay is not None and replay.returncode == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
