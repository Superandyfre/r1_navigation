"""Production protocol and atomic Node callbacks with no solver/driver."""
import json
import math
import threading
import unittest
from pathlib import Path as FilePath
import pytest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from nav_msgs.msg import Path
from r1_neupan_integration.execution_protocol import (
    CONTEXT_SCHEMA, SOURCE_SCHEMA, ContextError, ExecutionContextStore, make_source, validate_source,
)
from r1_neupan_integration.n1_16_node import N116Node
from r1_neupan_integration.worker_health import WorkerHealth


def context(sequence=1, stamp=100., attempt='attempt-a', path=1, goal=0, state='ACTIVE'):
    return dict(schema=CONTEXT_SCHEMA, authority_boot_id='authority-a',
                controller_boot_id='controller-a', navigation_attempt_id=attempt,
                path_version=path, goal_version=goal, sequence=sequence, monotonic=stamp,
                state=state, frame_id='map', path=[[0., 0., 0.], [1., 0., 0.]],
                goal=[1., 0., 0.])


class ContextProtocolTests(unittest.TestCase):
    def setUp(self):
        self.store = ExecutionContextStore()

    def test_first_active_then_exact_heartbeat_does_not_reinstall(self):
        self.assertTrue(self.store.accept(context(), 100.))
        active = self.store.active
        self.assertFalse(self.store.accept(context(2, 100.1), 100.1))
        self.assertIs(self.store.active, active)

    def test_revisions_require_strict_versions_and_retire_old_context(self):
        self.store.accept(context(), 100.)
        first = self.store.active
        self.assertTrue(self.store.accept(context(2, 100.1, path=2, goal=1), 100.1))
        self.assertNotEqual(first, self.store.active)
        self.assertFalse(self.store.accept(context(3, 100.2), 100.2))
        with self.assertRaises(ContextError):
            self.store.accept(context(4, 100.2, path=3, goal=1), 100.2)

    def test_new_same_path_attempt_supersedes_old_and_old_cannot_return(self):
        self.store.accept(context(), 100.)
        self.store.accept(context(2, 100.1, attempt='attempt-b'), 100.1)
        self.assertFalse(self.store.accept(context(3, 100.2), 100.2))
        self.assertEqual(self.store.active.attempt, 'attempt-b')

    def test_end_before_begin_and_delayed_end_are_irreversible(self):
        self.assertFalse(self.store.accept(context(state='END'), 100.))
        self.assertFalse(self.store.accept(context(2, 100.1), 100.1))
        self.store.accept(context(3, 100.2, attempt='attempt-b'), 100.2)
        self.assertTrue(self.store.accept(context(1, 99., attempt='attempt-b', state='END'), 101.))
        self.assertIsNone(self.store.active)
        self.assertFalse(self.store.accept(context(4, 101., attempt='attempt-b'), 101.))

    def test_old_end_cannot_invalidate_current_attempt(self):
        self.store.accept(context(), 100.)
        self.store.accept(context(2, 100.1, attempt='attempt-b'), 100.1)
        self.assertFalse(self.store.accept(context(1, 100., state='END'), 100.2))
        self.assertEqual(self.store.active.attempt, 'attempt-b')

    def test_authority_or_controller_restart_requires_full_controlled_restart(self):
        for field in ('authority_boot_id', 'controller_boot_id'):
            store = ExecutionContextStore()
            store.accept(context(), 100.)
            changed = context(2, 100.1)
            changed[field] = 'other-boot'
            with self.assertRaises(ContextError) as error:
                store.accept(changed, 100.1)
            self.assertTrue(error.exception.fatal)

    def test_same_identity_with_different_path_is_fatal(self):
        self.store.accept(context(), 100.)
        changed = context(2, 100.1)
        changed['path'][0][0] = .2
        with self.assertRaises(ContextError) as error:
            self.store.accept(changed, 100.1)
        self.assertTrue(error.exception.fatal)

    def test_changed_payload_at_same_sequence_is_fatal(self):
        self.store.accept(context(), 100.)
        with self.assertRaises(ContextError):
            self.store.accept(context(1, 100.1), 100.1)

    def test_strict_identity_numbers_and_finite_geometry(self):
        for value in (True, 1., '1', -1, 2**63):
            payload = context()
            payload['path_version'] = value
            with self.assertRaises(ContextError):
                self.store.accept(payload, 100.)
        for value in (True, '0', math.nan, math.inf):
            payload = context()
            payload['path'][0][0] = value
            with self.assertRaises(ContextError):
                self.store.accept(payload, 100.)
        payload = context()
        payload['goal'] = [2., 0., 0.]
        with self.assertRaises(ContextError):
            self.store.accept(payload, 100.)

    def test_old_and_future_context_cannot_replace_active(self):
        self.store.accept(context(), 100.)
        self.assertFalse(self.store.accept(context(2, 99., attempt='attempt-b'), 100.))
        with self.assertRaises(ContextError):
            self.store.accept(context(2, 101., attempt='attempt-b'), 100.)
        self.assertEqual(self.store.active.attempt, 'attempt-a')

    def test_source_v2_preserves_original_scan_and_transport_times(self):
        self.store.accept(context(), 100.)
        kwargs = dict(worker_boot='worker-a', generation=7, request=4, cycle=4, scan=4,
                      scan_received=99., dispatched=99.1, solved=99.2, published=100.,
                      linear=.1, angular=.2)
        source = make_source(self.store.active, **kwargs)
        self.assertEqual(source['schema'], SOURCE_SCHEMA)
        self.assertEqual(source['scan_receive_monotonic'], 99.)
        self.assertEqual(source['navigation_attempt_id'], 'attempt-a')
        for field, value in [('scan_received', 99.3), ('dispatched', 100.1),
                             ('solved', math.nan), ('request', True), ('linear', math.inf)]:
            with self.assertRaises(ContextError):
                make_source(self.store.active, **dict(kwargs, **{field: value}))


class AtomicNodeTests(unittest.TestCase):
    def setUp(self):
        self.node = N116Node.__new__(N116Node)
        self.node._require_execution_context = True
        self.node._execution_store = ExecutionContextStore()
        self.node.client = SimpleNamespace(worker_generation=7, health=WorkerHealth())
        self.node.state_lock = threading.Lock()
        self.node.map_frame = 'map'
        self.node.include_path_direction = True
        self.node.active_gear = 1
        self.node.path_version = 0
        self.node.nav2_path_version = 0
        self.node.goal_version = -1
        self.node.path_matrix = None
        self.node._publish_worker_health = Mock()
        self.node._publish_context = Mock()
        self.node._status = Mock()
        self.node._handle_fatal_error = Mock()
        self.notifications = []
        def notified():
            # Taking the real lock proves notify occurs after atomic install.
            with self.node.state_lock:
                active = self.node._execution_store.active
                self.notifications.append((active.attempt, self.node.nav2_path_version,
                                           self.node.goal_version, self.node.path_matrix.copy(),
                                           self.node.client.health.snapshot()['navigation_attempt_id']))
        self.node._planner_input_updated = notified

    def send(self, payload):
        with patch('r1_neupan_integration.n1_16_node.time.monotonic', return_value=100.):
            self.node.execution_context_callback(SimpleNamespace(data=json.dumps(payload)))

    def test_atomic_install_precedes_scheduler_notification_and_ignores_legacy(self):
        self.send(context())
        row = self.notifications[0]
        self.assertEqual(row[:3], ('attempt-a', 1, 0))
        self.assertEqual(row[4], 'attempt-a')
        np.testing.assert_allclose(row[3], [[0., 1.], [0., 0.], [0., 0.], [1., 1.]])
        self.node.path_callback(Path())
        self.node.goal_callback(None)
        self.assertEqual(len(self.notifications), 1)

    def test_end_invalidates_inflight_result_without_changing_fault(self):
        self.send(context())
        old = self.node._execution_store.active
        path_epoch = self.node.path_version
        self.send(context(2, state='END'))
        accepted, reason = self.node._fresh_for_publish({'scan_counter': 4}, 4, path_epoch, 0, old)
        self.assertFalse(accepted)
        self.assertEqual(reason, 'execution_context_changed')
        self.assertIsNone(self.node.path_matrix)
        self.assertIsNone(self.node.client.health.snapshot()['navigation_attempt_id'])
        self.assertIsNone(self.node.client.health.fault)
        self.assertEqual(len(self.notifications), 1)

    def test_repeated_context_does_not_reset_versions_or_rewake_solver(self):
        self.send(context())
        epoch = self.node.path_version
        self.send(context(2))
        self.assertEqual(self.node.path_version, epoch)
        self.assertEqual(len(self.notifications), 1)

    def test_new_attempt_same_geometry_has_new_internal_epoch(self):
        self.send(context())
        self.send(context(2, attempt='attempt-b'))
        self.assertEqual(self.node.path_version, 2)
        self.assertEqual(self.notifications[-1][0], 'attempt-b')
        self.assertEqual(self.node.nav2_path_version, 1)

    def test_invalid_context_during_execution_latches_and_does_not_partial_install(self):
        self.send(context())
        old = self.node._execution_store.active
        invalid = context(2)
        invalid['goal'] = [20., 0., 0.]
        self.send(invalid)
        self.node._handle_fatal_error.assert_called_once()
        self.assertIs(self.node._execution_store.active, old)

    def test_end_racing_path_preparation_prevents_late_install(self):
        entered, release = threading.Event(), threading.Event()
        prepare = self.node._prepare_path
        def delayed(message):
            entered.set()
            self.assertTrue(release.wait(2.))
            return prepare(message)
        self.node._prepare_path = delayed
        thread = threading.Thread(target=lambda: self.send(context()))
        thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            self.send(context(2, state='END'))
        finally:
            release.set()
            thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertIsNone(self.node._execution_store.active)
        self.assertIsNone(self.node.path_matrix)
        self.assertFalse(self.notifications)

    def test_fatal_prevents_late_context_or_health_attempt_change(self):
        self.send(context())
        self.node.client.health.latch('worker_timeout', 7)
        self.send(context(2, attempt='attempt-b'))
        self.node.client.health.bind_execution('attempt-b', 8, 99, 99)
        self.assertEqual(self.node.client.health.snapshot()['navigation_attempt_id'], 'attempt-a')
        self.assertEqual(len(self.notifications), 1)



if __name__ == '__main__':
    unittest.main()


_vectors = json.loads((FilePath(__file__).resolve().parents[2] /
                       'r1_neupan_runtime/test/source_protocol_vectors.json').read_text())


@pytest.mark.parametrize('vector', _vectors, ids=[row['name'] for row in _vectors])
def test_shared_python_cpp_source_protocol_vectors(vector):
    if vector['valid']:
        validate_source(vector['source'], vector['received'])
    else:
        with pytest.raises(ContextError):
            validate_source(vector['source'], vector['received'])
