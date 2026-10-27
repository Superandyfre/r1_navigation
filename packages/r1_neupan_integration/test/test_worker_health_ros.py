"""Isolated ROS health publisher test. No velocity publishers or drivers exist.

Run explicitly with ROS_DOMAIN_ID=187 and ROS_LOCALHOST_ONLY=1. This harness
uses the production health publishing method and actual client failure path,
with fake worker cleanup. It is NOT full production Node/solver acceptance.
"""
import concurrent.futures
import json
import os
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from r1_neupan_integration.n1_16_bridge import BridgeProtocolError, ProductionWholeCoreClient
from r1_neupan_integration.n1_16_node import N116Node


@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'explicit isolated ROS test only')
class WorkerHealthROSTests(unittest.TestCase):
    def test_fatal_broadcast_and_heartbeat_continue_during_blocked_cleanup(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '187')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        rclpy.init()
        node = Node('neupan_wp1_health_harness')
        rows = []
        with tempfile.TemporaryDirectory() as temp:
            client = ProductionWholeCoreClient(temp, 'cfg', 'model', {}, 1)
            client.health.ready(client.worker_generation)
            publisher = node.create_publisher(String, '/test_neupan_worker_health', 10)
            harness = SimpleNamespace(client=client, worker_health_pub=publisher, _health_publish_lock=threading.Lock())
            group = MutuallyExclusiveCallbackGroup()
            timer = node.create_timer(.025, lambda: N116Node._publish_worker_health(harness), callback_group=group)
            sub = node.create_subscription(String, '/test_neupan_worker_health',
                                           lambda msg: rows.append(json.loads(msg.data)), 10)
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(node)
            spin = threading.Thread(target=executor.spin)
            spin.start()
            entered, release = threading.Event(), threading.Event()
            def cleanup(_):
                entered.set()
                if not release.wait(5):
                    raise RuntimeError('test cleanup release timeout')
            client._quarantine_worker = cleanup
            def fail():
                try:
                    client._fail('worker_response_failure', 'isolated injection', request_id=3)
                except BridgeProtocolError as exc:
                    return exc.code
                raise AssertionError('failure did not raise')
            try:
                deadline = time.monotonic() + 3
                while not any(row['state'] == 'RUNNING' for row in rows) and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(any(row['state'] == 'RUNNING' for row in rows))
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                    future = pool.submit(fail)
                    try:
                        self.assertTrue(entered.wait(1))
                        deadline = time.monotonic() + 2
                        while len([r for r in rows if r['state'] == 'FAULT_LATCHED']) < 4 and time.monotonic() < deadline:
                            time.sleep(.01)
                        faults = [r for r in rows if r['state'] == 'FAULT_LATCHED']
                        self.assertGreaterEqual(len(faults), 4)
                        self.assertFalse(future.done(), 'cleanup must still be blocked')
                        self.assertEqual(len({r['fault_id'] for r in faults}), 1)
                        self.assertEqual(sorted(r['sequence'] for r in faults), [r['sequence'] for r in faults])
                        print(json.dumps({'scenario': 'WP1 blocked cleanup health transport',
                                          'fault_heartbeats_during_cleanup': len(faults),
                                          'first_fault_transport_delay_sec': faults[0]['monotonic'] - faults[0]['occurrence_monotonic'],
                                          'note': 'sender timing only, not physical stop or hard realtime bound'}, sort_keys=True))
                    finally:
                        release.set()
                    self.assertEqual(future.result(timeout=2), 'worker_response_failure')
                self.assertFalse(any('/cmd_vel' in name for name, _ in node.get_topic_names_and_types()))
            finally:
                release.set()
                timer.cancel()
                executor.shutdown(timeout_sec=2)
                spin.join(2)
                node.destroy_node()
                rclpy.shutdown()

    def test_production_node_initializes_and_publishes_fatal_without_worker(self):
        from pathlib import Path
        import signal
        packages = Path(__file__).resolve().parents[2]
        old_signal = signal.getsignal(signal.SIGUSR1)
        with tempfile.TemporaryDirectory() as temp:
            rclpy.init(args=[
                '--ros-args',
                '-p', 'robot_config_dir:=' + str(packages / 'navigation_neupan_bridge/config'),
                '-p', 'planner_config_file:=planner_r1_hw1.yaml',
                '-p', 'dune_checkpoint_file:=' + str(packages / 'r1_neupan_integration/config/models/dune_model_5000.pth'),
                '-p', 'n115_output_dir:=' + temp,
                '-p', 'worker_health_topic:=/test_production_worker_health',
                '-p', 'cmd_vel_topic:=/test_production_observed_zero',
                '-p', 'worker_health_period_sec:=0.025',
            ])
            node = None
            executor = MultiThreadedExecutor(num_threads=3)
            try:
                node = N116Node()
                rows = []
                sub = node.create_subscription(String, '/test_production_worker_health',
                                               lambda msg: rows.append(json.loads(msg.data)), 10)
                executor.add_node(node)
                until = time.monotonic() + 2
                while not rows and time.monotonic() < until:
                    executor.spin_once(timeout_sec=.05)
                self.assertTrue(rows)
                self.assertEqual(rows[-1]['state'], 'STARTING')
                self.assertFalse(node._worker_recovery_enabled)
                self.assertIsNone(node.client.process)
                node._planner_scheduler_error(RuntimeError('isolated production-node injection'))
                until = time.monotonic() + 2
                while not any(r['state'] == 'FAULT_LATCHED' for r in rows) and time.monotonic() < until:
                    executor.spin_once(timeout_sec=.05)
                self.assertTrue(any(r['state'] == 'FAULT_LATCHED' for r in rows))
                self.assertTrue(node._bridge_failed)
                self.assertIsNone(node.client.process)
                self.assertFalse(any(name == '/cmd_vel' for name, _ in node.get_topic_names_and_types()))
                print('Production N116Node: STARTING heartbeat, recovery disabled, FATAL received; no worker spawned.')
            finally:
                if node is not None:
                    node.finalize()
                executor.shutdown(timeout_sec=2)
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()
                signal.signal(signal.SIGUSR1, old_signal)


if __name__ == '__main__':
    unittest.main()
