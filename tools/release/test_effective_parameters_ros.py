"""Only isolated parameter servers. No solver, navigation, drivers or motion."""
import json
import os
from pathlib import Path
import threading
import unittest

from effective_parameters import SCHEMA, sample


@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'isolated opt-in only')
class LiveParameterServices(unittest.TestCase):
    def test_fresh_values_mismatch_unpinned_missing_and_duplicate(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '192')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        context = Context()
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        server = Node('parameter_gate_fixture', context=context)
        probe = Node('parameter_gate_probe', context=context)
        server.declare_parameter('FollowPath.shadow_mode', False)
        server.declare_parameter('FollowPath.timeout', .5)
        server.declare_parameter('topics', ['/test_parameters/scan', '/test_parameters/odom'])
        executor.add_node(server)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        duplicate = None
        results = []
        pins = dict(schema=SCHEMA, nodes={'/parameter_gate_fixture': dict(
            parameters={'FollowPath.shadow_mode': dict(type=1, value=False),
                        'FollowPath.timeout': dict(type=3, value=.5),
                        'topics': dict(type=9, value=['/test_parameters/scan', '/test_parameters/odom'])},
            closed_prefixes=['FollowPath'])})
        try:
            result = sample(probe, pins, 5.)
            results.append(dict(case='matching', **result))
            self.assertTrue(result['ok'], result)
            server.set_parameters([Parameter('FollowPath.timeout', value=.75)])
            result = sample(probe, pins, 2.)
            results.append(dict(case='changed_live_value', **result))
            self.assertIn('/parameter_gate_fixture:FollowPath.timeout:mismatch', result['errors'])
            server.set_parameters([Parameter('FollowPath.timeout', value=.5)])
            server.declare_parameter('FollowPath.unreviewed', 1.)
            result = sample(probe, pins, 2.)
            results.append(dict(case='unpinned_namespace', **result))
            self.assertIn('/parameter_gate_fixture:FollowPath.unreviewed:unpinned', result['errors'])
            server.undeclare_parameter('FollowPath.unreviewed')
            server.undeclare_parameter('topics')
            result = sample(probe, pins, 2.)
            results.append(dict(case='missing_parameter', **result))
            self.assertFalse(result['ok'])
            duplicate = Node('parameter_gate_fixture', context=context)
            executor.add_node(duplicate)
            # Discovery is asynchronous; poll the actual graph before sampling.
            import time
            deadline = time.monotonic() + 3.
            while sum(n == 'parameter_gate_fixture' for n, _ in probe.get_node_names_and_namespaces()) < 2:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
            result = sample(probe, pins, 2.)
            results.append(dict(case='duplicate_node', **result))
            self.assertIn('/parameter_gate_fixture:duplicate_node_name', result['errors'])
            missing = dict(schema=SCHEMA, nodes={'/parameter_gate_absent': dict(parameters={
                'timeout': dict(type=3, value=.5)})})
            result = sample(probe, missing, .2)
            results.append(dict(case='absent_node', **result))
            self.assertFalse(result['ok'])
        finally:
            executor.shutdown(timeout_sec=3.)
            thread.join(timeout=3.)
            if duplicate:
                duplicate.destroy_node()
            server.destroy_node()
            probe.destroy_node()
            context.shutdown()
            report = os.environ.get('NEUPAN_PARAMETER_TEST_REPORT')
            if report:
                with Path(report).open('x') as stream:
                    json.dump(results, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
