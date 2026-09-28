#!/usr/bin/env python3
"""Collect N1 topic, timing, stale-scan, and command-value evidence."""

import csv
import json
import math
import os
import statistics
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage


def finite(value):
    return math.isfinite(float(value))


def frequency_summary(times):
    if len(times) < 2:
        return {'count': len(times), 'hz': None, 'interval_ms': {}}
    intervals = [b - a for a, b in zip(times, times[1:])]
    intervals_ms = [value * 1000.0 for value in intervals]
    return {
        'count': len(times),
        'hz': (len(times) - 1) / (times[-1] - times[0])
        if times[-1] > times[0] else None,
        'interval_ms': {
            'mean': statistics.fmean(intervals_ms),
            'median': statistics.median(intervals_ms),
            'p95': sorted(intervals_ms)[min(len(intervals_ms) - 1, int(math.ceil(0.95 * len(intervals_ms)) - 1))],
            'max': max(intervals_ms),
        },
    }


def value_summary(values):
    if not values:
        return {'count': 0, 'min': None, 'mean': None, 'max': None}
    return {
        'count': len(values),
        'min': min(values),
        'mean': statistics.fmean(values),
        'max': max(values),
    }


def latency_summary(values):
    if not values:
        return {
            'count': 0, 'mean_ms': None, 'median_ms': None, 'p95_ms': None,
            'p99_ms': None, 'max_ms': None, 'over_20_ms': 0,
            'over_50_ms': 0, 'over_100_ms': 0, 'over_500_ms': 0,
            'over_1000_ms': 0,
        }
    ordered = sorted(values)
    p95 = ordered[min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered)) - 1))]
    return {
        'count': len(values),
        'mean_ms': statistics.fmean(values),
        'median_ms': statistics.median(values),
        'p95_ms': p95,
        'p99_ms': ordered[min(len(ordered) - 1, int(math.ceil(0.99 * len(ordered)) - 1))],
        'max_ms': max(values),
        'over_20_ms': sum(value > 20.0 for value in values),
        'over_50_ms': sum(value > 50.0 for value in values),
        'over_100_ms': sum(value > 100.0 for value in values),
        'over_500_ms': sum(value >= 500.0 for value in values),
        'over_1000_ms': sum(value >= 1000.0 for value in values),
    }


TIMING_FIELDS = [
    'schema',
    'cycle_id',
    'scan_counter',
    'scan_stamp_sec',
    'scan_receive_monotonic',
    'path_version',
    'tf_stamp_sec',
    'cycle_start_monotonic',
    'planner_start_monotonic',
    'planner_end_monotonic',
    'output_publish_monotonic',
    'scan_processing_ms',
    'scan_conversion_ms',
    'scan_filter_ms',
    'scan_tf_lookup_ms',
    'scan_transform_ms',
    'tf_lookup_ms',
    'robot_state_assembly_ms',
    'path_callback_ms',
    'path_transform_ms',
    'state_copy_ms',
    'path_message_publish_ms',
    'core_forward_ms',
    'pan_ms',
    'dune_ms',
    'nrmp_ms',
    'core_other_ms',
    'trajectory_publish_ms',
    'cmd_publish_ms',
    'visualization_ms',
    'obstacle_points_count',
    'scan_delta_since_previous',
    'dropped_obsolete_scans',
    'dune_effective_points_count',
    'nrmp_effective_points_count',
    'pan_iterations',
    'pan_early_converged',
    'ecos_solve_ms',
    'ecos_solve_cpu_ms',
    'ecos_solve_calls',
    'solver_failures',
    'core_forward_cpu_ms',
    'n114_parameter_count',
    'n114_parameter_elements',
    'n114_payload_bytes',
    'n114_parameter_extraction_ms',
    'n114_request_serialization_ms',
    'n114_request_send_ms',
    'n114_worker_queue_wait_ms',
    'n114_worker_decode_ms',
    'n114_worker_solve_wall_ms',
    'n114_worker_solve_cpu_ms',
    'n114_worker_result_serialization_ms',
    'n114_response_return_ms',
    'n114_main_reconstruction_ms',
    'n114_ipc_roundtrip_ms',
    'n114_main_wait_ms',
    'n114_full_call_ms',
    'n114_worker_sequence',
    'n114_status',
]

STAGE_FIELDS = [field for field in TIMING_FIELDS if field.endswith('_ms')]

N115_TIMING_FIELDS = [
    'schema', 'cycle_id', 'scan_counter', 'scan_stamp_sec',
    'scan_receive_monotonic', 'path_version', 'tf_stamp_sec',
    'cycle_start_monotonic', 'worker_core_start_monotonic',
    'worker_core_end_monotonic', 'output_publish_monotonic',
    'main_input_extraction_ms', 'main_request_prep_ms',
    'main_request_serialization_ms', 'main_request_send_ms', 'main_wait_ms',
    'main_response_receive_ms', 'main_reconstruction_ms', 'main_ipc_roundtrip_ms',
    'main_publish_ms', 'main_full_cycle_ms', 'worker_decode_input_ms',
    'worker_dune_ms', 'worker_nrmp_ms', 'worker_cvxpy_ms', 'worker_pan_ms',
    'worker_core_total_ms', 'worker_core_cpu_ms', 'worker_result_pack_ms',
    'worker_result_serialization_ms', 'worker_response_send_ms',
    'obstacle_points_count', 'worker_pan_iterations',
    'worker_pan_early_converged', 'worker_dune_effective_points_count',
    'worker_nrmp_effective_points_count', 'dropped_obsolete_scans',
    'duplicate_scan', 'queue_depth', 'request_id', 'worker_sequence',
    'worker_pid', 'worker_failure', 'finite', 'nan_inf',
]
N115_STAGE_FIELDS = [field for field in N115_TIMING_FIELDS if field.endswith('_ms')]
N116_TIMING_FIELDS = [
    'schema', 'cycle_id', 'scan_counter', 'scan_stamp_sec',
    'scan_receive_monotonic', 'path_version', 'goal_version', 'tf_stamp_sec',
    'cycle_start_monotonic', 'worker_core_start_monotonic',
    'worker_core_end_monotonic', 'output_publish_monotonic',
    'main_input_extraction_ms', 'main_request_prep_ms',
    'main_request_serialization_ms', 'main_request_send_ms', 'main_wait_ms',
    'main_response_receive_ms', 'main_reconstruction_ms', 'main_ipc_roundtrip_ms',
    'main_publish_ms', 'main_full_cycle_ms', 'worker_decode_input_ms',
    'worker_dune_ms', 'worker_nrmp_ms', 'worker_cvxpy_ms', 'worker_pan_ms',
    'worker_core_total_ms', 'worker_core_cpu_ms', 'worker_result_pack_ms',
    'worker_result_serialization_ms', 'worker_response_send_ms',
    'obstacle_points_count', 'worker_pan_iterations',
    'worker_pan_early_converged', 'worker_dune_effective_points_count',
    'worker_nrmp_effective_points_count', 'dropped_obsolete_scans',
    'duplicate_scan', 'queue_depth', 'request_id', 'worker_sequence',
    'worker_pid', 'worker_generation', 'worker_failure', 'finite', 'nan_inf',
    'stale_response',
]
N116_STAGE_FIELDS = [field for field in N116_TIMING_FIELDS if field.endswith('_ms')]
ALL_STAGE_FIELDS = list(dict.fromkeys(STAGE_FIELDS + N115_STAGE_FIELDS + N116_STAGE_FIELDS))


class N1Metrics(Node):
    def __init__(self):
        super().__init__('r1_n1_metrics')
        self.declare_parameter('duration_sec', 30.0)
        self.declare_parameter('warmup_sec', 0.0)
        self.declare_parameter('warmup_timing_messages', 0)
        self.declare_parameter('output_file', 'n1_results/result.json')
        self.declare_parameter('scan_topic', '/scan_neupan')
        self.declare_parameter('path_topic', '/r1_test_path')
        self.declare_parameter('plan_topic', '/neupan_plan')
        self.declare_parameter('cmd_topic', '/neupan_cmd_vel')
        self.declare_parameter('timing_topic', '/neupan_timing')
        self.declare_parameter('timing_csv_file', '')
        self.declare_parameter('scenario', 'unknown')
        self.declare_parameter('control_frequency', 0.0)
        self.declare_parameter('latest_scan_only', False)
        self.declare_parameter('enable_visualization', False)
        self.declare_parameter('scan_downsample', 1)
        self.declare_parameter('planner_config_file', 'planner.yaml')
        self.declare_parameter('planner_receding', 0)
        self.declare_parameter('planner_iter_num', 0)
        self.declare_parameter('planner_dune_max_num', 0)
        self.declare_parameter('planner_nrmp_max_num', 0)
        self.declare_parameter('planner_iter_threshold', -1.0)
        self.declare_parameter('torch_num_threads', 0)
        self.declare_parameter('torch_num_interop_threads', 0)
        self.declare_parameter('thread_env_policy', 'inherit')
        self.declare_parameter('runtime_report_file', '')
        self.declare_parameter('bridge_status_topic', '/neupan_bridge_status')

        sensor_qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.scan_times = []
        self.odom_times = []
        self.path_times = []
        self.plan_times = []
        self.cmd_times = []
        self.timing_times = []
        self.clock_wall_times = []
        self.clock_seconds = []
        self.tf_times = []
        self.tf_transform_counts = []
        self.tf_frame_pairs = set()
        self.scan_frames = set()
        self.path_frames = set()
        self.odom_frames = set()
        self.odom_child_frames = set()
        self.scan_count = 0
        self.scan_ranges = 0
        self.invalid_scan_values = 0
        self.scan_valid_range_counts = []
        self.scan_min_ranges = []
        self.plan_pose_counts = []
        self.linear_x = []
        self.angular_z = []
        self.invalid_cmd_values = 0
        self.timing_scan_counters = []
        self.planner_ms = []
        self.scan_to_output_ms = []
        self.scan_age_ms = []
        self.repeated_scan_cycles = 0
        self.previous_timing_scan_counter = None
        self.timing_records = []
        self.bridge_status_events = []
        self.bridge_status_counts = Counter()
        self.started = time.monotonic()
        self.measurement_started = None
        self.warmup_timing_count = 0
        self.finished = False

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, sensor_qos)
        self.create_subscription(Path, self.get_parameter('path_topic').value, self.path_cb, 10)
        self.create_subscription(Path, self.get_parameter('plan_topic').value, self.plan_cb, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_topic').value, self.cmd_cb, 50)
        self.create_subscription(Float64MultiArray, self.get_parameter('timing_topic').value, self.timing_cb, 50)
        self.create_subscription(String, self.get_parameter('bridge_status_topic').value, self.bridge_status_cb, 50)
        self.create_subscription(Clock, '/clock', self.clock_cb, 50)
        tf_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, tf_qos)
        self.timer = self.create_timer(0.25, self.check_done)

    def scan_cb(self, message):
        if self.measurement_started is None:
            return
        self.scan_times.append(time.monotonic())
        self.scan_count += 1
        self.scan_ranges = len(message.ranges)
        self.scan_frames.add(message.header.frame_id)
        self.invalid_scan_values += sum(1 for value in message.ranges if not finite(value) and not math.isinf(value))
        valid_ranges = [
            float(value) for value in message.ranges
            if finite(value) and message.range_min <= value <= message.range_max
        ]
        self.scan_valid_range_counts.append(len(valid_ranges))
        if valid_ranges:
            self.scan_min_ranges.append(min(valid_ranges))

    def odom_cb(self, message):
        if self.measurement_started is None:
            return
        self.odom_times.append(time.monotonic())
        self.odom_frames.add(message.header.frame_id)
        self.odom_child_frames.add(message.child_frame_id)

    def path_cb(self, message):
        if self.measurement_started is None:
            return
        self.path_times.append(time.monotonic())
        self.path_frames.add(message.header.frame_id)

    def plan_cb(self, message):
        if self.measurement_started is None:
            return
        self.plan_times.append(time.monotonic())
        self.plan_pose_counts.append(len(message.poses))

    def cmd_cb(self, message):
        if self.measurement_started is None:
            return
        self.cmd_times.append(time.monotonic())
        values = (float(message.linear.x), float(message.angular.z))
        if all(finite(value) for value in values):
            self.linear_x.append(values[0])
            self.angular_z.append(values[1])
        else:
            self.invalid_cmd_values += 1

    def timing_cb(self, message):
        if self.measurement_started is None:
            if time.monotonic() - self.started >= float(self.get_parameter('warmup_sec').value):
                self.warmup_timing_count += 1
            return
        now = time.monotonic()
        self.timing_times.append(now)
        data = list(message.data)
        if len(data) < 6:
            return
        if len(data) >= len(N116_TIMING_FIELDS) and data[0] == 4.0:
            record = {
                field: float(value)
                for field, value in zip(N116_TIMING_FIELDS, data)
            }
            record['planner_start_monotonic'] = record['worker_core_start_monotonic']
            record['planner_end_monotonic'] = record['worker_core_end_monotonic']
            planner_start = record['planner_start_monotonic']
            planner_end = record['planner_end_monotonic']
            scan_receive = record['scan_receive_monotonic']
            output = record['output_publish_monotonic']
            scan_counter = record['scan_counter']
            self.timing_records.append(record)
        elif len(data) >= len(N115_TIMING_FIELDS) and data[0] == 3.0:
            record = {
                field: float(value)
                for field, value in zip(N115_TIMING_FIELDS, data)
            }
            # Keep the existing generic report calculations meaningful while
            # preserving the worker-specific timing fields.
            record['planner_start_monotonic'] = record['worker_core_start_monotonic']
            record['planner_end_monotonic'] = record['worker_core_end_monotonic']
            planner_start = record['planner_start_monotonic']
            planner_end = record['planner_end_monotonic']
            scan_receive = record['scan_receive_monotonic']
            output = record['output_publish_monotonic']
            scan_counter = record['scan_counter']
            self.timing_records.append(record)
        elif len(data) >= len(TIMING_FIELDS):
            record = {
                field: float(value)
                for field, value in zip(TIMING_FIELDS, data)
            }
            self.timing_records.append(record)
            planner_start = record['planner_start_monotonic']
            planner_end = record['planner_end_monotonic']
            scan_receive = record['scan_receive_monotonic']
            output = record['output_publish_monotonic']
            scan_counter = record['scan_counter']
        else:
            # Preserve compatibility with the N1 six-field timing stream.
            planner_start, planner_end = data[2], data[3]
            scan_receive, output = data[1], data[4]
            scan_counter = data[5]
        self.timing_scan_counters.append(scan_counter)
        if self.previous_timing_scan_counter == scan_counter:
            self.repeated_scan_cycles += 1
        self.previous_timing_scan_counter = scan_counter
        self.planner_ms.append((planner_end - planner_start) * 1000.0)
        if scan_receive >= 0.0:
            self.scan_to_output_ms.append((output - scan_receive) * 1000.0)
            self.scan_age_ms.append((planner_start - scan_receive) * 1000.0)

    def bridge_status_cb(self, message):
        try:
            event = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            event = {'code': 'malformed_status_message', 'message': str(message.data)}
        code = str(event.get('code', 'unknown'))
        self.bridge_status_counts[code] += 1
        self.bridge_status_events.append(event)

    def clock_cb(self, message):
        if self.measurement_started is None:
            return
        self.clock_wall_times.append(time.monotonic())
        self.clock_seconds.append(
            float(message.clock.sec) + float(message.clock.nanosec) * 1e-9
        )

    def tf_cb(self, message):
        if self.measurement_started is None:
            return
        self.tf_times.append(time.monotonic())
        self.tf_transform_counts.append(len(message.transforms))
        for transform in message.transforms:
            self.tf_frame_pairs.add(
                (transform.header.frame_id, transform.child_frame_id)
            )

    def check_done(self):
        if self.measurement_started is None:
            warmup = float(self.get_parameter('warmup_sec').value)
            warmup_cycles = int(self.get_parameter('warmup_timing_messages').value)
            if (
                time.monotonic() - self.started < warmup
                or self.warmup_timing_count < warmup_cycles
            ):
                return
            self.measurement_started = time.monotonic()
            self.get_logger().info(
                f'N1 measurement started after {self.measurement_started - self.started:.2f}s warmup'
            )
            return
        duration = float(self.get_parameter('duration_sec').value)
        if not self.finished and time.monotonic() - self.measurement_started >= duration:
            self.finished = True
            self.write_report()
            self.get_logger().info('N1 metrics complete; shutting down')
            rclpy.shutdown()

    def write_report(self):
        output_file = os.path.abspath(str(self.get_parameter('output_file').value))
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        processed_counter = [
            int(record['scan_counter']) for record in self.timing_records
            if record['scan_counter'] >= 0
        ]
        scan_use_counts = Counter(processed_counter)
        unique_processed = len(scan_use_counts)
        duplicate_cycles = sum(max(0, count - 1) for count in scan_use_counts.values())
        timing_breakdown = {
            field: latency_summary([
                record[field] for record in self.timing_records
                if field in record
            ])
            for field in ALL_STAGE_FIELDS
        }
        pan_iterations = [
            record.get('pan_iterations', 0.0) for record in self.timing_records
        ]
        early_converged = [
            record.get('pan_early_converged', 0.0) for record in self.timing_records
        ]
        dropped_obsolete = sum(
            int(record.get('dropped_obsolete_scans', 0.0))
            for record in self.timing_records
        )
        solver_failures = sum(
            int(record.get('solver_failures', 0.0))
            for record in self.timing_records
        )
        ecos_calls = sum(
            int(record.get('ecos_solve_calls', 0.0))
            for record in self.timing_records
        )
        early_count = sum(1 for value in early_converged if value > 0.5)
        runtime_report_file = str(self.get_parameter('runtime_report_file').value)
        runtime_report = None
        runtime_report_error = None
        if runtime_report_file:
            try:
                with open(runtime_report_file, 'r', encoding='utf-8') as stream:
                    runtime_report = json.load(stream)
            except (OSError, json.JSONDecodeError) as exc:
                runtime_report_error = f'{type(exc).__name__}: {exc}'
        report = {
            'experiment': {
                'scenario': str(self.get_parameter('scenario').value),
                'control_frequency_hz': float(self.get_parameter('control_frequency').value),
                'latest_scan_only': bool(self.get_parameter('latest_scan_only').value),
                'enable_visualization': bool(self.get_parameter('enable_visualization').value),
                'scan_downsample': int(self.get_parameter('scan_downsample').value),
                'planner_config_file': str(self.get_parameter('planner_config_file').value),
                'timing_schema': (
                    'n1_16_production_bridge' if any(record.get('schema') == 4.0 for record in self.timing_records)
                    else 'n1_15_whole_core' if any(record.get('schema') == 3.0 for record in self.timing_records)
                    else 'legacy_n1'
                ),
                'planner_overrides': {
                    'receding': int(self.get_parameter('planner_receding').value),
                    'iter_num': int(self.get_parameter('planner_iter_num').value),
                    'dune_max_num': int(self.get_parameter('planner_dune_max_num').value),
                    'nrmp_max_num': int(self.get_parameter('planner_nrmp_max_num').value),
                    'iter_threshold': float(self.get_parameter('planner_iter_threshold').value),
                },
                'torch_runtime_policy': {
                    'torch_num_threads': int(self.get_parameter('torch_num_threads').value),
                    'torch_num_interop_threads': int(self.get_parameter('torch_num_interop_threads').value),
                    'thread_env_policy': str(self.get_parameter('thread_env_policy').value),
                    'runtime_report_file': runtime_report_file or None,
                    'runtime_report': runtime_report,
                    'runtime_report_error': runtime_report_error,
                },
            },
            'duration_wall_sec': (
                time.monotonic() - self.measurement_started
                if self.measurement_started is not None else 0.0
            ),
            'warmup_sec': float(self.get_parameter('warmup_sec').value),
            'warmup_timing_messages': int(
                self.get_parameter('warmup_timing_messages').value
            ),
            'warmup_timing_observed': self.warmup_timing_count,
            'topics': {
                'scan_neupan': frequency_summary(self.scan_times),
                'odom': frequency_summary(self.odom_times),
                'r1_test_path': frequency_summary(self.path_times),
                'neupan_plan': frequency_summary(self.plan_times),
                'neupan_cmd_vel': frequency_summary(self.cmd_times),
                'neupan_timing': frequency_summary(self.timing_times),
                'tf': frequency_summary(self.tf_times),
            },
            'frames': {
                'scan': sorted(self.scan_frames),
                'path': sorted(self.path_frames),
                'odom_header': sorted(self.odom_frames),
                'odom_child': sorted(self.odom_child_frames),
            },
            'scan': {
                'messages': self.scan_count,
                'ranges_per_message': self.scan_ranges,
                'invalid_finite_values': self.invalid_scan_values,
                'valid_ranges_per_message': value_summary(self.scan_valid_range_counts),
                'minimum_range_m': value_summary(self.scan_min_ranges),
            },
            'planner_output': {
                'plan_pose_count': value_summary(self.plan_pose_counts),
                'linear_x_mps': value_summary(self.linear_x),
                'angular_z_rps': value_summary(self.angular_z),
                'invalid_cmd_messages': self.invalid_cmd_values,
            },
                'timing': {
                'planner_compute': latency_summary(self.planner_ms),
                'scan_receive_to_output': latency_summary(self.scan_to_output_ms),
                'scan_age_at_planner_start': latency_summary(self.scan_age_ms),
                'timing_messages': len(self.timing_times),
                'repeated_latest_scan_cycles': self.repeated_scan_cycles,
                'unique_scan_counters': len(set(self.timing_scan_counters)),
                'cycle_records': self.timing_records,
                    'breakdown': timing_breakdown,
                    'whole_core': latency_summary([
                        record['worker_core_total_ms'] for record in self.timing_records
                        if 'worker_core_total_ms' in record
                    ]),
                    'worker_dune': latency_summary([
                        record['worker_dune_ms'] for record in self.timing_records
                        if 'worker_dune_ms' in record
                    ]),
                    'worker_nrmp': latency_summary([
                        record['worker_nrmp_ms'] for record in self.timing_records
                        if 'worker_nrmp_ms' in record
                    ]),
                    'worker_cvxpy': latency_summary([
                        record['worker_cvxpy_ms'] for record in self.timing_records
                        if 'worker_cvxpy_ms' in record
                    ]),
                    'ipc_main': latency_summary([
                        record['main_ipc_roundtrip_ms'] for record in self.timing_records
                        if 'main_ipc_roundtrip_ms' in record
                    ]),
                    'queue_depth_max': max([
                        record.get('queue_depth', 0.0) for record in self.timing_records
                    ] or [0.0]),
                    'worker_failures': sum(
                        int(record.get('worker_failure', 0.0))
                        for record in self.timing_records
                    ),
                    'nan_inf_or_crash_markers': sum(
                        int(record.get('nan_inf', 0.0))
                        for record in self.timing_records
                    ),
                'cycle_total': latency_summary([
                    (record['output_publish_monotonic'] - record['cycle_start_monotonic']) * 1000.0
                    for record in self.timing_records
                    if 'output_publish_monotonic' in record
                ]),
                'dropped_obsolete_scans': dropped_obsolete,
                'dune_effective_points': value_summary([
                    record.get('dune_effective_points_count', 0.0)
                    for record in self.timing_records
                ]),
                'nrmp_effective_points': value_summary([
                    record.get('nrmp_effective_points_count', 0.0)
                    for record in self.timing_records
                ]),
                'pan_iterations': value_summary(pan_iterations),
                'pan_early_convergence': {
                    'count': early_count,
                    'total': len(early_converged),
                    'ratio': early_count / len(early_converged)
                    if early_converged else None,
                },
                'ecos': {
                    'solver': 'ECOS via cvxpylayers',
                    'solve_calls': ecos_calls,
                    'solve_time_ms': latency_summary([
                        record.get('ecos_solve_ms', 0.0)
                        for record in self.timing_records
                    ]),
                    'solve_cpu_time_ms': latency_summary([
                        record.get('ecos_solve_cpu_ms', 0.0)
                        for record in self.timing_records
                    ]),
                    'solver_failures': solver_failures,
                    'status_observation': (
                        'return-success or exception only; cvxpylayers does not expose ECOS status text'
                    ),
                },
            },
            'scan_usage': {
                'scans_received': self.scan_count,
                'planning_cycles': len(processed_counter),
                'unique_scans_processed': unique_processed,
                'average_plans_per_scan': (
                    len(processed_counter) / unique_processed
                    if unique_processed else None
                ),
                'max_plans_per_scan': max(scan_use_counts.values()) if scan_use_counts else 0,
                'duplicate_scan_cycles': duplicate_cycles,
                'dropped_obsolete_scans': dropped_obsolete,
                'scan_use_counts': {
                    str(counter): count for counter, count in sorted(scan_use_counts.items())
                },
                'effective_obstacle_points': value_summary([
                    record['obstacle_points_count']
                    for record in self.timing_records
                    if 'obstacle_points_count' in record
                ]),
            },
            'clock': {
                'messages': len(self.clock_seconds),
                'sim_seconds_span': (
                    self.clock_seconds[-1] - self.clock_seconds[0]
                    if len(self.clock_seconds) >= 2 else None
                ),
                'wall_seconds_span': (
                    self.clock_wall_times[-1] - self.clock_wall_times[0]
                    if len(self.clock_wall_times) >= 2 else None
                ),
            },
            'tf': {
                'transforms_per_message': value_summary(self.tf_transform_counts),
                'frame_pairs': [
                    {'parent': parent, 'child': child}
                    for parent, child in sorted(self.tf_frame_pairs)
                ],
            },
            'stability': {
                'nan_inf_or_crash_markers': self.invalid_cmd_values + self.invalid_scan_values + sum(
                    int(record.get('nan_inf', 0.0)) for record in self.timing_records
                ),
                'backlog_policy': 'wrapper stores latest scan; no unbounded scan queue in integration patch',
                'worker_failures': sum(
                    int(record.get('worker_failure', 0.0)) for record in self.timing_records
                ),
                'bridge_status_counts': dict(self.bridge_status_counts),
                'stale_responses': sum(
                    int(record.get('stale_response', 0.0))
                    for record in self.timing_records
                ) + self.bridge_status_counts.get('stale_response', 0),
                'bridge_status_events': self.bridge_status_events,
            },
            'bridge': {
                'status_counts': dict(self.bridge_status_counts),
                'status_events': self.bridge_status_events,
            },
        }
        with open(output_file, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
        csv_file = str(self.get_parameter('timing_csv_file').value)
        if not csv_file:
            csv_file = os.path.splitext(output_file)[0] + '.timing.csv'
        csv_file = os.path.abspath(csv_file)
        os.makedirs(os.path.dirname(csv_file), exist_ok=True)
        with open(csv_file, 'w', encoding='utf-8', newline='') as stream:
            if any(record.get('schema') == 4.0 for record in self.timing_records):
                fieldnames = N116_TIMING_FIELDS
            elif any(record.get('schema') == 3.0 for record in self.timing_records):
                fieldnames = N115_TIMING_FIELDS
            else:
                fieldnames = TIMING_FIELDS
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.timing_records)
        self.get_logger().info(f'Wrote N1 metrics to {output_file}')
        self.get_logger().info(f'Wrote timing CSV to {csv_file}')


def main(args=None):
    rclpy.init(args=args)
    node = N1Metrics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finished:
            node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
