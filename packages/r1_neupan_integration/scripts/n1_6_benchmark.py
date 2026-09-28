#!/usr/bin/env python3
"""Run repeatable N1.6 NeuPAN timing candidates with per-run cleanup.

The runner owns one ROS launch per run.  It keeps latest-only enabled, excludes
startup warmup from the metrics node, samples the launch process subtree, and
aggregates all requested repetitions without selecting a best run.
"""

import argparse
import json
import math
import os
import pathlib
import signal
import statistics
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[3]
INSTALL = ROOT / 'install'
PYTHON = '/home/pinggu/venvs/neupan_ros_jazzy/bin/python'
PAGE_SIZE = os.sysconf('SC_PAGE_SIZE')
CLK_TCK = os.sysconf('SC_CLK_TCK')


def read_text(path):
    try:
        return pathlib.Path(path).read_text(encoding='utf-8').strip()
    except OSError:
        return None


def system_snapshot():
    """Read host conditions only; this harness never changes them."""
    thermal = {
        path.name: read_text(path / 'temp')
        for path in pathlib.Path('/sys/class/thermal').glob('thermal_zone*')
    }
    governors = {
        path.parts[-3]: read_text(path / 'scaling_governor')
        for path in sorted(pathlib.Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq'))[:24]
    }
    gpu = None
    try:
        gpu = subprocess.run(
            ['nvidia-smi', '--query-gpu=utilization.gpu,temperature.gpu', '--format=csv,noheader,nounits'],
            text=True, capture_output=True, timeout=5, check=False,
        ).stdout.strip() or None
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return {
        'load_average': list(os.getloadavg()),
        'cpu_governors': governors,
        'cpu_temperature_raw': thermal,
        'gpu_utilization_temperature': gpu,
    }


def lightweight_system_snapshot():
    """Low-impact timeline sample; avoid subprocess/GPU queries per tick."""
    thermal = {
        path.name: read_text(path / 'temp')
        for path in pathlib.Path('/sys/class/thermal').glob('thermal_zone*')
    }
    frequencies = {
        path.parts[-3]: read_text(path)
        for path in sorted(pathlib.Path('/sys/devices/system/cpu').glob(
            'cpu[0-9]*/cpufreq/scaling_cur_freq'
        ))[:24]
    }
    return {
        'load_average': list(os.getloadavg()),
        'cpu_temperature_raw': thermal,
        'cpu_frequency_raw': frequencies,
    }


def proc_stat(pid):
    try:
        raw = pathlib.Path(f'/proc/{pid}/stat').read_text(encoding='ascii')
        closing = raw.rfind(')')
        fields = raw[closing + 2:].split()
        ppid = int(fields[1])
        utime = int(fields[11])
        stime = int(fields[12])
        statm = pathlib.Path(f'/proc/{pid}/statm').read_text(encoding='ascii').split()
        rss_mb = int(statm[1]) * PAGE_SIZE / (1024.0 * 1024.0)
        cmdline_path = pathlib.Path(f'/proc/{pid}/cmdline')
        cmdline = cmdline_path.read_bytes().replace(b'\0', b' ').decode(
            'utf-8', errors='replace'
        ).strip()
        return {
            'pid': pid,
            'ppid': ppid,
            'cpu_ticks': utime + stime,
            'rss_mb': rss_mb,
            'cmdline': cmdline,
        }
    except (FileNotFoundError, PermissionError, ValueError, IndexError):
        return None


def process_snapshot():
    snapshot = {}
    for entry in pathlib.Path('/proc').glob('[0-9]*'):
        item = proc_stat(int(entry.name))
        if item is not None:
            snapshot[item['pid']] = item
    return snapshot


def subtree(root_pid, snapshot):
    children = {}
    for item in snapshot.values():
        children.setdefault(item['ppid'], []).append(item['pid'])
    result = set()
    pending = [root_pid]
    while pending:
        pid = pending.pop()
        if pid in result:
            continue
        result.add(pid)
        pending.extend(children.get(pid, []))
    return [snapshot[pid] for pid in result if pid in snapshot]


def process_kind(cmdline):
    if 'neupan_node' in cmdline:
        return 'neupan'
    if 'gz sim' in cmdline or '/gz sim' in cmdline:
        return 'gazebo'
    if 'pointcloud_to_laserscan' in cmdline:
        return 'pointcloud_to_laserscan'
    if 'parameter_bridge' in cmdline:
        return 'ros_gz_bridge'
    if 'robot_state_publisher' in cmdline:
        return 'robot_state_publisher'
    if 'ros2 launch' in cmdline or 'launch.py' in cmdline:
        return 'ros_launch'
    return 'other'


def sample_processes(root_pid, previous, previous_wall):
    snapshot = process_snapshot()
    items = subtree(root_pid, snapshot)
    now = time.monotonic()
    samples = {}
    for item in items:
        kind = process_kind(item['cmdline'])
        if kind == 'other':
            continue
        if previous_wall is None or item['pid'] not in previous:
            cpu_percent = None
        else:
            elapsed = now - previous_wall
            cpu_percent = (
                (item['cpu_ticks'] - previous[item['pid']]) / CLK_TCK
                / elapsed * 100.0
                if elapsed > 0 else None
            )
        sample = samples.setdefault(kind, {'cpu': [], 'rss_mb': [], 'pids': set()})
        sample['pids'].add(item['pid'])
        sample['rss_mb'].append(item['rss_mb'])
        if cpu_percent is not None and math.isfinite(cpu_percent):
            sample['cpu'].append(max(0.0, cpu_percent))
    next_previous = {item['pid']: item['cpu_ticks'] for item in items}
    return samples, next_previous, now


def merge_samples(aggregate, samples):
    for kind, values in samples.items():
        target = aggregate.setdefault(kind, {'cpu': [], 'rss_mb': [], 'pids': set()})
        target['cpu'].extend(values['cpu'])
        target['rss_mb'].extend(values['rss_mb'])
        target['pids'].update(values['pids'])


def timeline_samples(samples):
    return {
        kind: {
            'cpu': list(values['cpu']),
            'rss_mb': list(values['rss_mb']),
            'pids': sorted(values['pids']),
        }
        for kind, values in samples.items()
    }


def sample_summary(aggregate):
    result = {}
    for kind, values in sorted(aggregate.items()):
        result[kind] = {
            'cpu_percent': {
                'mean': statistics.fmean(values['cpu']) if values['cpu'] else None,
                'max': max(values['cpu']) if values['cpu'] else None,
                'samples': len(values['cpu']),
            },
            'rss_mb': {
                'mean': statistics.fmean(values['rss_mb']) if values['rss_mb'] else None,
                'max': max(values['rss_mb']) if values['rss_mb'] else None,
                'samples': len(values['rss_mb']),
            },
            'pids_observed': sorted(values['pids']),
        }
    return result


def terminate_group(process, grace_sec=8.0):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=grace_sec)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=grace_sec)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=grace_sec)


def cleanup_world(scenario):
    executable = INSTALL / 'r1_neupan_integration' / 'lib' / 'r1_neupan_integration' / 'cleanup_n1_world'
    if scenario == 'free_space':
        world = INSTALL / 'r1_neupan_integration' / 'share' / 'r1_neupan_integration' / 'worlds' / 'free_space.sdf'
    else:
        world = INSTALL / 'r1_gazebo' / 'share' / 'r1_gazebo' / 'worlds' / 'navigation_test.sdf'
    if executable.is_file() and world.is_file():
        subprocess.run([str(executable), str(world)], cwd=ROOT, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=15)
    # A gz child can be adopted by init after ros2 launch exits.  The helper
    # above is intentionally world-scoped, but the adopted child may ignore
    # its first TERM; finish the same exact-world cleanup here.
    world_text = str(world)
    for _ in range(3):
        snapshot = process_snapshot()
        matches = [
            item['pid'] for item in snapshot.values()
            if world_text in item['cmdline'] and 'gz sim' in item['cmdline']
        ]
        if not matches:
            break
        for pid in matches:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(1.0)
        remaining = [pid for pid in matches if proc_stat(pid) is not None]
        if remaining and _ == 2:
            for pid in remaining:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def residual_world_processes(scenario):
    if scenario == 'free_space':
        world = INSTALL / 'r1_neupan_integration' / 'share' / 'r1_neupan_integration' / 'worlds' / 'free_space.sdf'
    else:
        world = INSTALL / 'r1_gazebo' / 'share' / 'r1_gazebo' / 'worlds' / 'navigation_test.sdf'
    return [
        {'pid': item['pid'], 'cmdline': item['cmdline']}
        for item in process_snapshot().values()
        if str(world) in item['cmdline'] and 'gz sim' in item['cmdline']
    ]


def candidate_args(args):
    return {
        'planner_iter_threshold': args.iter_threshold,
        'planner_nrmp_max_num': args.nrmp_max_num,
        'planner_dune_max_num': args.dune_max_num,
        'planner_receding': args.receding,
        'planner_iter_num': args.iter_num,
    }


def run_one(args, scenario, repetition):
    out_dir = pathlib.Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f'{args.candidate}_{scenario}_{repetition:02d}'
    metrics_path = out_dir / f'{stem}.json'
    csv_path = out_dir / f'{stem}.timing.csv'
    log_path = out_dir / f'{stem}.launch.log'
    runtime_path = out_dir / f'{stem}.runtime.json'
    snapshot_dir = None
    timeline_path = None
    if args.n110_snapshot_root:
        snapshot_dir = pathlib.Path(args.n110_snapshot_root).resolve() / stem
        snapshot_dir.mkdir(parents=True, exist_ok=True)
        timeline_path = snapshot_dir / 'process_timeline.jsonl'
    cleanup_world(scenario)
    system_before = system_snapshot()
    overrides = candidate_args(args)
    launch_command = [
        'exec', 'ros2', 'launch', 'r1_neupan_integration',
        'n1_validation.launch.py',
        f'scenario:={scenario}',
        f'duration_sec:={args.duration_sec}',
        f'warmup_sec:={args.warmup_sec}',
        f'warmup_timing_messages:={args.warmup_timing_messages}',
        'lidar_profile:=lidar_navigation',
        'latest_scan_only:=true',
        'enable_visualization:=false',
        'control_frequency:=50.0',
        f'planner_iter_threshold:={overrides["planner_iter_threshold"]}',
        f'planner_nrmp_max_num:={overrides["planner_nrmp_max_num"]}',
        f'planner_dune_max_num:={overrides["planner_dune_max_num"]}',
        f'planner_receding:={overrides["planner_receding"]}',
        f'planner_iter_num:={overrides["planner_iter_num"]}',
        f'python_executable:={PYTHON}',
        f'metrics_file:={metrics_path}',
        f'metrics_csv_file:={csv_path}',
        f'torch_num_threads:={args.torch_num_threads}',
        f'torch_num_interop_threads:={args.torch_num_interop_threads}',
        f'thread_env_policy:={args.thread_env_policy}',
        f'runtime_report_file:={runtime_path}',
    ]
    if snapshot_dir is not None:
        launch_command.append(f'n110_snapshot_dir:={snapshot_dir}')
        launch_command.extend([
            f'n110_normal_max_ms:={args.n110_normal_max_ms}',
            f'n110_medium_min_ms:={args.n110_medium_min_ms}',
            f'n110_medium_max_ms:={args.n110_medium_max_ms}',
            f'n110_spike_min_ms:={args.n110_spike_min_ms}',
        ])
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    env['GZ_IP'] = '127.0.0.1'
    started = time.monotonic()
    aggregate = {}
    previous = None
    previous_wall = None
    sampling_started = None
    log_stream = log_path.open('w', encoding='utf-8')
    timeline_stream = (
        timeline_path.open('w', encoding='utf-8')
        if timeline_path is not None else None
    )
    process = subprocess.Popen(
        [
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash && '
            'source install/setup.bash && '
            + ' '.join(launch_command),
        ],
        cwd=ROOT,
        env=env,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    timeout = args.startup_timeout_sec + args.warmup_sec + args.duration_sec + args.shutdown_timeout_sec
    timed_out = False
    try:
        while process.poll() is None:
            elapsed = time.monotonic() - started
            if elapsed > timeout:
                timed_out = True
                terminate_group(process)
                break
            samples, next_previous, now = sample_processes(process.pid, previous, previous_wall)
            previous = next_previous
            previous_wall = now
            if timeline_stream is not None:
                timeline_stream.write(json.dumps({
                    'monotonic': now,
                    'processes': timeline_samples(samples),
                    'system': lightweight_system_snapshot(),
                }, sort_keys=True) + '\n')
                timeline_stream.flush()
            if elapsed >= args.warmup_sec:
                if sampling_started is None:
                    sampling_started = now
                merge_samples(aggregate, samples)
            time.sleep(args.sample_period_sec)
        process.wait(timeout=args.shutdown_timeout_sec)
    except subprocess.TimeoutExpired:
        timed_out = True
        terminate_group(process)
    finally:
        log_stream.close()
        if timeline_stream is not None:
            timeline_stream.close()
        cleanup_world(scenario)
    system_after = system_snapshot()
    residual = residual_world_processes(scenario)

    report = None
    if metrics_path.is_file() and metrics_path.stat().st_size > 0:
        try:
            report = json.loads(metrics_path.read_text(encoding='utf-8'))
        except json.JSONDecodeError:
            report = None
    result = {
        'candidate': args.candidate,
        'scenario': scenario,
        'repetition': repetition,
        'metrics_file': str(metrics_path),
        'timing_csv_file': str(csv_path),
        'launch_log': str(log_path),
        'runtime_report_file': str(runtime_path),
        'n110_snapshot_dir': str(snapshot_dir) if snapshot_dir is not None else None,
        'process_timeline_file': str(timeline_path) if timeline_path is not None else None,
        'launch_exit_code': process.returncode,
        'timed_out': timed_out,
        'measurement_sampling_sec': (
            time.monotonic() - sampling_started if sampling_started is not None else 0.0
        ),
        'process_samples': sample_summary(aggregate),
        'system_before': system_before,
        'system_after': system_after,
        'residual_world_processes': residual,
        'report': report,
    }
    result_path = out_dir / f'{stem}.run.json'
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps({
        'scenario': scenario,
        'repetition': repetition,
        'exit': process.returncode,
        'timed_out': timed_out,
        'metrics': str(metrics_path),
        'records': len((report or {}).get('timing', {}).get('cycle_records', [])),
    }), flush=True)
    return result


def number_summary(values):
    values = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    if not values:
        return {'count': 0, 'median': None, 'min': None, 'max': None}
    return {
        'count': len(values),
        'median': statistics.median(values),
        'min': min(values),
        'max': max(values),
    }


def summarize_run(result):
    report = result.get('report') or {}
    topics = report.get('topics', {})
    timing = report.get('timing', {})
    usage = report.get('scan_usage', {})
    output = (topics.get('neupan_cmd_vel') or {}).get('hz')
    planner = timing.get('planner_compute') or {}
    e2e = timing.get('scan_receive_to_output') or {}
    breakdown = timing.get('breakdown') or {}
    return {
        'scenario': result['scenario'],
        'repetition': result['repetition'],
        'output_hz': output,
        'scan_hz': (topics.get('scan_neupan') or {}).get('hz'),
        'planner_hz': (topics.get('neupan_timing') or {}).get('hz'),
        'planner_mean_ms': planner.get('mean_ms'),
        'planner_median_ms': planner.get('median_ms'),
        'planner_p95_ms': planner.get('p95_ms'),
        'planner_max_ms': planner.get('max_ms'),
        'e2e_mean_ms': e2e.get('mean_ms'),
        'e2e_median_ms': e2e.get('median_ms'),
        'e2e_p95_ms': e2e.get('p95_ms'),
        'e2e_max_ms': e2e.get('max_ms'),
        'unique_scans': usage.get('unique_scans_processed'),
        'processed_scans': usage.get('planning_cycles'),
        'dropped_obsolete_scans': usage.get('dropped_obsolete_scans'),
        'duplicate_scans': usage.get('duplicate_scan_cycles'),
        'effective_obstacle_points': usage.get('effective_obstacle_points'),
        'dune_effective_points': timing.get('dune_effective_points'),
        'nrmp_effective_points': timing.get('nrmp_effective_points'),
        'dune_ms': breakdown.get('dune_ms'),
        'nrmp_ms': breakdown.get('nrmp_ms'),
        'pan_iterations': timing.get('pan_iterations'),
        'pan_early_convergence': timing.get('pan_early_convergence'),
        'ecos': timing.get('ecos'),
        'process_samples': result.get('process_samples', {}),
        'launch_exit_code': result.get('launch_exit_code'),
        'timed_out': result.get('timed_out'),
        'runtime_policy': (report.get('experiment') or {}).get('torch_runtime_policy'),
        'system_before': result.get('system_before'),
        'system_after': result.get('system_after'),
        'residual_world_processes': result.get('residual_world_processes'),
    }


def aggregate_summary(args, results):
    per_run = [summarize_run(result) for result in results]
    by_scenario = {}
    for scenario in args.scenarios:
        rows = [row for row in per_run if row['scenario'] == scenario]
        by_scenario[scenario] = {
            'runs': rows,
            'median_output_hz': statistics.median(
                [row['output_hz'] for row in rows if row['output_hz'] is not None]
            ) if any(row['output_hz'] is not None for row in rows) else None,
            'worst_run_output_hz': min(
                [row['output_hz'] for row in rows if row['output_hz'] is not None]
            ) if any(row['output_hz'] is not None for row in rows) else None,
            'median_p95_ms': statistics.median(
                [row['planner_p95_ms'] for row in rows if row['planner_p95_ms'] is not None]
            ) if any(row['planner_p95_ms'] is not None for row in rows) else None,
            'worst_run_p95_ms': max(
                [row['planner_p95_ms'] for row in rows if row['planner_p95_ms'] is not None]
            ) if any(row['planner_p95_ms'] is not None for row in rows) else None,
        }
    return {
        'schema': 'r1_n1_runtime_benchmark_v2',
        'candidate': args.candidate,
        'configuration': {
            'scenario_worlds': {
                'free_space': 'r1_neupan_integration/worlds/free_space.sdf',
                'obstacle': 'r1_gazebo/worlds/navigation_test.sdf',
            },
            'lidar_profile': 'lidar_navigation',
            'scan_ranges': 720,
            'latest_scan_only': True,
            'warmup_sec': args.warmup_sec,
            'warmup_timing_messages': args.warmup_timing_messages,
            'duration_sec': args.duration_sec,
            'repetitions': args.repetitions,
            'overrides': candidate_args(args),
            'torch_runtime': {
                'torch_num_threads': args.torch_num_threads,
                'torch_num_interop_threads': args.torch_num_interop_threads,
                'thread_env_policy': args.thread_env_policy,
            },
        },
        'aggregate': by_scenario,
        'runs': per_run,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--output-dir', default='n1_results/n1_6')
    parser.add_argument('--scenarios', nargs='+', default=['free_space', 'obstacle'],
                        choices=['free_space', 'obstacle'])
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument('--duration-sec', type=float, default=30.0)
    parser.add_argument('--warmup-sec', type=float, default=15.0)
    parser.add_argument('--warmup-timing-messages', type=int, default=3)
    parser.add_argument('--sample-period-sec', type=float, default=0.5)
    parser.add_argument('--startup-timeout-sec', type=float, default=90.0)
    parser.add_argument('--shutdown-timeout-sec', type=float, default=20.0)
    parser.add_argument('--ros-domain-id', type=int, default=42)
    parser.add_argument('--iter-threshold', type=float, default=-1.0)
    parser.add_argument('--nrmp-max-num', type=int, default=0)
    parser.add_argument('--dune-max-num', type=int, default=0)
    parser.add_argument('--receding', type=int, default=0)
    parser.add_argument('--iter-num', type=int, default=0)
    parser.add_argument('--torch-num-threads', type=int, required=True, choices=[1, 2, 4, 8])
    parser.add_argument('--torch-num-interop-threads', type=int, default=0)
    parser.add_argument('--thread-env-policy', choices=['inherit', 'match_intra'], default='inherit')
    parser.add_argument('--n110-snapshot-root', default='',
                        help='Enable N1.10 selected snapshots and timestamped process evidence')
    parser.add_argument('--n110-normal-max-ms', type=float, default=20.0)
    parser.add_argument('--n110-medium-min-ms', type=float, default=50.0)
    parser.add_argument('--n110-medium-max-ms', type=float, default=200.0)
    parser.add_argument('--n110-spike-min-ms', type=float, default=300.0)
    args = parser.parse_args()
    if args.repetitions < 1 or args.duration_sec <= 0 or args.warmup_sec < 0 or args.warmup_timing_messages < 0:
        parser.error('repetitions must be positive, duration_sec > 0, warmup_sec >= 0, warmup_timing_messages >= 0')
    if args.torch_num_interop_threads < 0:
        parser.error('torch-num-interop-threads must be >= 0')
    if args.thread_env_policy == 'match_intra' and args.torch_num_threads <= 0:
        parser.error('match_intra requires torch-num-threads > 0')
    results = []
    for scenario in args.scenarios:
        for repetition in range(1, args.repetitions + 1):
            results.append(run_one(args, scenario, repetition))
    summary = aggregate_summary(args, results)
    output = pathlib.Path(args.output_dir).resolve() / f'{args.candidate}_summary.json'
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(f'Wrote {output}', flush=True)
    return 0 if all(result.get('report') is not None and not result.get('timed_out') for result in results) else 2


if __name__ == '__main__':
    sys.exit(main())
