"""Real supervisor/Node methods with fake IPC, no ROS graph or physical driver."""
import concurrent.futures
import json
import pickle
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from r1_neupan_integration.worker_health import WorkerHealth
from r1_neupan_integration.n1_16_bridge import BridgeProtocolError, ProductionWholeCoreClient
from r1_neupan_integration.n1_16_node import N116Node


class WorkerHealthTests(unittest.TestCase):
    def test_unique_boots_and_starting_does_not_claim_attempt(self):
        a, b = WorkerHealth(), WorkerHealth()
        first = a.snapshot()
        self.assertNotEqual(first['boot_id'], b.snapshot()['boot_id'])
        self.assertEqual(first['state'], 'STARTING')
        self.assertIsNone(first['navigation_attempt_id'])

    def test_first_fatal_survives_context_ready_success_and_shutdown(self):
        health = WorkerHealth()
        health.update_context(7, 4, 5)
        fault = health.latch('worker_response_failure', 7, 12)
        health.latch('scheduler_exception', 7, 13)
        health.ready(8)
        health.success(99)
        health.update_context(8, 6, 7)
        health.shutting_down()
        state = health.snapshot()
        self.assertEqual(state['state'], 'FAULT_LATCHED')
        self.assertEqual(state['fault_id'], fault.fault_id)
        self.assertEqual(state['fault_code'], 'worker_response_failure')
        self.assertEqual(state['worker_generation'], 7)
        self.assertEqual(state['failed_request_id'], 12)
        self.assertEqual((state['path_version'], state['goal_version']), (4, 5))
        self.assertEqual(state['last_successful_request_id'], -1)
        json.dumps(state, allow_nan=False)

    def test_sequences_are_unique_under_concurrent_heartbeats(self):
        health = WorkerHealth()
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            rows = list(pool.map(lambda _: health.snapshot(), range(100)))
        self.assertEqual(sorted(r['sequence'] for r in rows), list(range(1, 101)))

    def test_ready_success_and_shutdown_without_fault(self):
        health = WorkerHealth()
        health.ready(7)
        health.success(12)
        self.assertEqual(health.snapshot()['last_successful_request_id'], 12)
        self.assertEqual(health.snapshot()['state'], 'RUNNING')
        health.shutting_down()
        self.assertEqual(health.snapshot()['state'], 'SHUTTING_DOWN')


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.client = ProductionWholeCoreClient(self.temp.name, 'cfg', 'model', {}, 1)
        self.client.health.update_context(self.client.worker_generation, 3, 4)

    def test_fatal_is_observable_while_cleanup_blocks(self):
        entered, release = threading.Event(), threading.Event()
        def cleanup(_):
            entered.set()
            if not release.wait(3):
                raise AssertionError('test cleanup was not released')
        self.client._quarantine_worker = cleanup
        def fail():
            with self.assertRaises(BridgeProtocolError):
                self.client._fail('worker_timeout', 'injected', request_id=2)
        thread = threading.Thread(target=fail)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            rows = [self.client.health.snapshot() for _ in range(10)]
            self.assertTrue(thread.is_alive())
            self.assertTrue(all(row['state'] == 'FAULT_LATCHED' for row in rows))
            self.assertEqual(len({row['fault_id'] for row in rows}), 1)
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())

    def test_fatal_precedes_disk_io_and_cleanup_even_when_both_raise(self):
        seen = []
        def broken_event(*args, **kwargs):
            seen.append(('disk', self.client.health.snapshot()['state']))
            raise OSError('disk unavailable')
        def broken_cleanup(_):
            seen.append(('cleanup', self.client.health.snapshot()['state']))
            raise OSError('cleanup unavailable')
        self.client._event = broken_event
        self.client._quarantine_worker = broken_cleanup
        with self.assertRaises(BridgeProtocolError) as error:
            self.client._fail('nan_inf', 'injected', request_id=2)
        self.assertEqual(error.exception.code, 'nan_inf')
        self.assertEqual(seen[:2], [('disk', 'FAULT_LATCHED'), ('cleanup', 'FAULT_LATCHED')])
        self.assertTrue(all(state == 'FAULT_LATCHED' for _, state in seen))

    def test_all_fatal_categories_latch(self):
        for code in ['worker_response_failure', 'worker_crash', 'pipe_broken',
                     'worker_timeout', 'malformed_response', 'nan_inf',
                     'generation_mismatch', 'request_id_mismatch']:
            with self.subTest(code=code), tempfile.TemporaryDirectory() as temp:
                client = ProductionWholeCoreClient(temp, 'cfg', 'model', {}, 1)
                client._quarantine_worker = Mock()
                with self.assertRaises(BridgeProtocolError):
                    client._fail(code, 'injected')
                self.assertEqual(client.health.snapshot()['fault_code'], code)

    def test_startup_failure_latches_before_logging(self):
        conn = Mock()
        conn.poll.return_value = True
        conn.recv_bytes.return_value = pickle.dumps({'status': 'READY', 'worker_generation': -9})
        proc = Mock()
        self.client.context = SimpleNamespace(Pipe=lambda **kw: (conn, Mock()), Process=lambda **kw: proc)
        states = []
        self.client._event = lambda *a, **kw: states.append(self.client.health.snapshot()['state'])
        self.client._quarantine_worker = Mock()
        with self.assertRaises(BridgeProtocolError):
            self.client.startup([[0]], [[0]], [[0]], 1)
        self.assertEqual(states, ['FAULT_LATCHED'])

    def test_fatal_cannot_be_restarted_by_legacy_recovery_method(self):
        self.client.health.latch('worker_crash', self.client.worker_generation)
        self.client._start = Mock()
        with self.assertRaises(BridgeProtocolError):
            self.client.recover_worker([[0]], [[0]], [[0]], 1)
        self.client._start.assert_not_called()


class NodeMethodTests(unittest.TestCase):
    def setUp(self):
        self.node = N116Node.__new__(N116Node)
        self.node.client = SimpleNamespace(health=WorkerHealth(), worker_generation=7)
        self.node._bridge_failed = False
        self.node._scheduler_fault = ''
        self.node._publish_failsafe_stop = Mock()
        self.node._status = Mock()
        self.node._recover_worker_from_cached_inputs = Mock()
        self.node.worker_health_pub = Mock()
        self.node._health_publish_lock = threading.Lock()

    def test_scheduler_failure_latches_before_broken_diagnostics_and_never_recovers(self):
        self.node._status.side_effect = OSError('diagnostics unavailable')
        self.node._planner_scheduler_error(RuntimeError('injected'))
        self.assertTrue(self.node._bridge_failed)
        self.assertEqual(self.node.client.health.snapshot()['fault_code'], 'scheduler_exception')
        self.node._recover_worker_from_cached_inputs.assert_not_called()
        self.node._publish_failsafe_stop.assert_called_once()
        row = json.loads(self.node.worker_health_pub.publish.call_args[0][0].data)
        self.assertEqual(row['state'], 'FAULT_LATCHED')

    def test_health_publish_failure_cannot_skip_stop_or_latch(self):
        self.node.worker_health_pub.publish.side_effect = RuntimeError('DDS unavailable')
        self.node._handle_fatal_error('bridge_exception', 'injected')
        self.assertTrue(self.node._bridge_failed)
        self.node._publish_failsafe_stop.assert_called_once()

    def test_timer_observes_client_fault_without_waiting_for_node_catch(self):
        self.node.client.health.latch('worker_timeout', 7, 9)
        self.node._publish_worker_health()
        row = json.loads(self.node.worker_health_pub.publish.call_args[0][0].data)
        self.assertEqual(row['fault_code'], 'worker_timeout')
        self.assertFalse(self.node._bridge_failed)  # catch may still be blocked in cleanup

    def test_start_of_run_rejects_fault_without_touching_planner(self):
        self.node._shutdown_requested = False
        self.node.client.health.latch('worker_crash', 7)
        self.node._consume_gear_transition = Mock()
        self.node.run()
        self.node._consume_gear_transition.assert_not_called()

    def test_nonfatal_superseded_path_drop_does_not_latch(self):
        self.node.state_lock = threading.Lock()
        self.node.path_version, self.node.goal_version = 2, 3
        accepted, reason = self.node._fresh_for_publish({'scan_counter': 4}, 4, 1, 3)
        self.assertFalse(accepted)
        self.assertEqual(reason, 'path_version_changed')
        self.assertIsNone(self.node.client.health.fault)

    def test_defaults_and_hw1_launch_and_yaml_disable_recovery(self):
        declared = {}
        fake = SimpleNamespace(declare_parameter=lambda key, value: declared.setdefault(key, value))
        N116Node.declare_client_parameters(fake)
        self.assertFalse(declared['worker_recovery_enabled'])
        self.assertGreater(declared['worker_health_period_sec'], 0)
        import ast
        import yaml
        packages = Path(__file__).resolve().parents[2]
        launch = packages / 'navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py'
        tree = ast.parse(launch.read_text())
        overrides = []
        for entry in ast.walk(tree):
            if isinstance(entry, ast.Dict):
                for key, value in zip(entry.keys, entry.values):
                    if isinstance(key, ast.Constant) and key.value == 'worker_recovery_enabled':
                        overrides.append(ast.literal_eval(value))
        self.assertEqual(overrides, [False])
        config = yaml.safe_load((packages / 'navigation_neupan_bridge/config/robot_r1_hw1.yaml').read_text())
        self.assertFalse(next(iter(config.values()))['ros__parameters']['worker_recovery_enabled'])


if __name__ == '__main__':
    unittest.main()
