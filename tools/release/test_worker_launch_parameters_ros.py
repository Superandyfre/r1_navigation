"""Actual installed N116 parent with full HW1 launch inputs; no scan/goal/solver."""
import json
import os
from pathlib import Path
import threading
import unittest

from effective_parameters import SCHEMA, sample


@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'isolated opt-in only')
class WorkerLaunchParameters(unittest.TestCase):
    def test_actual_worker_full_launch_parameters(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '192')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from r1_neupan_integration.n1_16_node import N116Node
        import r1_neupan_integration.n1_16_node as worker_module
        bundle = Path('/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-06')
        root = Path(os.environ['NEUPAN_WORKER_LAUNCH_REPORT'])
        root.mkdir(exist_ok=False)
        expected_root = Path(os.environ['NEUPAN_EXPECT_WORKER_ROOT']).resolve()
        self.assertTrue(Path(worker_module.__file__).resolve().is_relative_to(expected_root))
        (root / 'worker-source.json').write_text(json.dumps(dict(source=str(Path(worker_module.__file__).resolve()))))
        # Launch belongs to system Python; do not install its tooling into the
        # dedicated solver venv. Consume the independently generated draft.
        contract = json.loads(Path(os.environ['NEUPAN_WORKER_LAUNCH_CONTRACT']).read_text())
        self.assertEqual(contract['launch_arguments'], dict(scan_source='dual', command_profile='smoothed',
                         use_mesh_self_filter=True, collision_check_enabled=True))
        spec = contract['pins']['nodes']['/n1_16_production_bridge']
        self.assertEqual(len(spec['parameters']), 82)
        self.assertNotIn('robot_type', spec['parameters'])
        self.assertIn('robot_type', contract['informational_launch_inputs']['/n1_16_production_bridge'])
        parameters = {name: pin['value'] for name, pin in spec['parameters'].items()}
        parameters['n115_output_dir'] = str(root / 'worker')
        params_file = root / 'params.yaml'
        import yaml
        params_file.write_text(yaml.safe_dump({'n1_16_production_bridge': {'ros__parameters': parameters}}))
        rclpy.init(args=['--ros-args', '--params-file', str(params_file),
                         '-r', '/neupan_cmd_vel:=/test_worker_launch/diagnostic_cmd'])
        node = None
        executor = None
        thread = None
        probe = None
        try:
            node = N116Node()
            probe = Node('worker_launch_parameter_probe', use_global_arguments=False)
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin)
            thread.start()
            pins = dict(schema=SCHEMA, nodes={'/n1_16_production_bridge': spec})
            result = sample(probe, pins, 5.)
            (root / 'live-parameters.json').write_text(json.dumps(result, indent=2))
            self.assertTrue(result['ok'], result['errors'])
            attempts = []
            for name, value in [('worker_recovery_enabled', True), ('worker_health_period_sec', .25),
                                ('worker_health_topic', '/wrong_health'), ('n115_timeout_sec', 1.),
                                ('use_sim_time', True), ('execution_context_topic', '/wrong_context')]:
                before = node.get_parameter(name).value
                attempt = node.set_parameters([Parameter(name, value=value)])
                attempts.append(dict(name=name, successful=attempt[0].successful, reason=attempt[0].reason,
                                     before=before, after=node.get_parameter(name).value))
                self.assertFalse(attempt[0].successful, name)
                self.assertEqual(node.get_parameter(name).value, before)
            self.assertFalse(node.get_parameter('worker_recovery_enabled').value)
            (root / 'recovery-change-refused.json').write_text(json.dumps(attempts, indent=2))
            self.assertFalse(any(topic == '/cmd_vel' for topic, _ in probe.get_topic_names_and_types()))
            (root / 'contract.json').write_text(json.dumps(contract, indent=2))
        finally:
            if node:
                node.finalize()
            if executor:
                executor.shutdown(timeout_sec=5.)
            if thread:
                thread.join(timeout=5.)
            if node:
                node.destroy_node()
            if probe:
                probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
