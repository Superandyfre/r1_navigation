"""Real N116Node input/result protocol in isolated Domain 190; fake IPC solver.

No hardware driver, model loading or real cmd_vel. This verifies the Node, not
NeuPAN solver behavior. Enable explicitly with NEUPAN_ISOLATED_ROS_TEST=1.
"""
import json
import os
from pathlib import Path
import signal
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from r1_neupan_integration.execution_protocol import CONTEXT_SCHEMA, SOURCE_SCHEMA
from r1_neupan_integration.n1_16_node import N116Node


@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'isolated ROS opt-in')
class ExecutionContextROS(unittest.TestCase):
    def exercise(self, mutation):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '190')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        packages = Path(__file__).resolve().parents[2]
        old_signal = signal.getsignal(signal.SIGUSR1)
        with tempfile.TemporaryDirectory() as temp:
            ledger = Path(temp) / 'existing-ledger.json'
            ledger.write_text(json.dumps(dict(path_version=900, goal_version=901)))
            with patch.dict(os.environ, {'N116_CTX_LEDGER': str(ledger)}):
                rclpy.init(args=['--ros-args',
                    '-p', 'robot_config_dir:=' + str(packages / 'navigation_neupan_bridge/config'),
                    '-p', 'planner_config_file:=planner_r1_hw1.yaml',
                    '-p', 'dune_checkpoint_file:=' + str(packages / 'r1_neupan_integration/config/models/dune_model_5000.pth'),
                    '-p', 'n115_output_dir:=' + temp,
                    '-p', 'map_frame:=map',
                    '-p', 'execution_context_topic:=/test_wp3_worker/context',
                    '-p', 'worker_health_topic:=/test_wp3_worker/health',
                    '-p', 'planner_result_topic:=/test_wp3_worker/source',
                    '-p', 'cmd_vel_topic:=/test_wp3_worker/observed_cmd',
                    '-p', 'forward_only_enabled:=false',
                    '-p', 'command_stabilizer_enabled:=false',
                    '-p', 'auto_gear_transition_enabled:=false',
                    '-p', 'obstacle_motion_enabled:=false'])
                node = None
                executor = MultiThreadedExecutor(num_threads=3)
                try:
                    node = N116Node()
                    self.assertTrue(node._planner_scheduler.stop(1.))
                    self.assertEqual(node.nav2_path_version, 0)
                    self.assertEqual(node.goal_version, -1)
                    self.assertEqual(json.loads(ledger.read_text())['path_version'], 900)
                    node._robot_transform = lambda: True
                    node.robot_state = np.zeros((3, 1))
                    node.dynamic_footprint_enabled = False
                    node.scan_counter = 1
                    node.latest_scan_receive_monotonic = time.monotonic() - .1
                    node.latest_scan_update_monotonic = node.latest_scan_receive_monotonic + .01
                    old_scan_time = node.latest_scan_receive_monotonic
                    old_scan_update = node.latest_scan_update_monotonic
                    node.obstacle_points = np.empty((2, 0))
                    node.observed_odom_received_monotonic = time.monotonic()
                    node.client.startup = lambda *a, **k: node.client.health.ready(node.client.worker_generation)
                    node.client.mark_stale_response = Mock()
                    rows = []
                    node.create_subscription(String, '/test_wp3_worker/source',
                                             lambda msg: rows.append(json.loads(msg.data)), 20)
                    pub = node.create_publisher(String, '/test_wp3_worker/context', 20)
                    executor.add_node(node)
                    payload = dict(schema=CONTEXT_SCHEMA, authority_boot_id='authority-a',
                                   controller_boot_id='controller-a', navigation_attempt_id='attempt-a',
                                   path_version=1, goal_version=0, sequence=1,
                                   monotonic=time.monotonic(), state='ACTIVE', frame_id='map',
                                   path=[[0., 0., 0.], [1., 0., 0.]], goal=[1., 0., 0.])
                    until = time.monotonic() + 2
                    while node._execution_store.active is None and time.monotonic() < until:
                        pub.publish(String(data=json.dumps(payload)))
                        executor.spin_once(timeout_sec=.02)
                    self.assertIsNotNone(node._execution_store.active)
                    self.assertEqual(node.client.health.snapshot()['navigation_attempt_id'], 'attempt-a')
                    captured = {}
                    def fake_solve(state, points, path, version, scan, stamp, **kwargs):
                        captured.update(kwargs)
                        # A newer scan arrives during the flight. Never attach
                        # its timestamp to the captured old scan's result.
                        with node.state_lock:
                            node.latest_scan_receive_monotonic = time.monotonic()
                            node.latest_scan_update_monotonic = time.monotonic()
                            node.scan_counter += 1
                        now = time.monotonic()
                        outputs = dict(opt_state=np.array([[0., .1], [0., 0.], [0., 0.]]),
                                       ref_state=np.array([[0., .1], [0., 0.], [0., 0.]]),
                                       action=np.array([[.1], [.0]]))
                        record = dict(worker_generation=node.client.worker_generation, request_id=1,
                                      scan_counter=scan, cycle_id=kwargs['cycle_id'], info={},
                                      timings=dict(request_dispatched_monotonic=now,
                                                   worker_solve_end_monotonic=now + .000001))
                        return outputs, record
                    node.client.solve = fake_solve
                    if mutation == 'end-during-postprocessing':
                        original = node._path_message
                        def end_before_publish(*args, **kwargs):
                            end = dict(payload, state='END', sequence=2, monotonic=time.monotonic())
                            node.execution_context_callback(String(data=json.dumps(end)))
                            return original(*args, **kwargs)
                        node._path_message = end_before_publish
                    node.run()
                    until = time.monotonic() + .3
                    while time.monotonic() < until:
                        executor.spin_once(timeout_sec=.02)
                    self.assertIsNone(node.client.process)
                    self.assertIsNone(node.client.health.fault)
                    self.assertEqual(captured['scan_callback_receive_monotonic'], old_scan_time)
                    self.assertEqual(captured['scan_update_monotonic'], old_scan_update)
                    if mutation == 'end-during-postprocessing':
                        self.assertFalse(rows)
                    else:
                        self.assertTrue(rows)
                        self.assertEqual(rows[-1]['schema'], SOURCE_SCHEMA)
                        self.assertEqual(rows[-1]['scan_receive_monotonic'], old_scan_time)
                        self.assertEqual(rows[-1]['scan_counter'], 1)
                        self.assertEqual(rows[-1]['navigation_attempt_id'], 'attempt-a')
                    self.assertFalse(any(name == '/cmd_vel' for name, _ in node.get_topic_names_and_types()))
                    print(json.dumps(dict(scenario=mutation, published_sources=len(rows),
                                          scan_snapshot_preserved=True, ledger_ignored=True)))
                finally:
                    if node is not None:
                        node.finalize()
                    executor.shutdown(timeout_sec=2)
                    if node is not None:
                        node.destroy_node()
                    rclpy.shutdown()
                    signal.signal(signal.SIGUSR1, old_signal)

    def test_real_node_publishes_v2_with_immutable_input_snapshot(self):
        self.exercise('new-scan-during-solve')

    def test_end_during_postprocessing_prevents_old_source_publication(self):
        self.exercise('end-during-postprocessing')


if __name__ == '__main__':
    unittest.main()
