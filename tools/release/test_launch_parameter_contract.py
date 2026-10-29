"""Evaluate installed launch builders only; prohibit any process execution."""
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from launch_parameter_contract import compile_contract, select_parameters, typed

BUNDLE = Path('/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-06')
CALIBRATION = Path('/home/rpp/rpp_ws/config/r1_hw1_calibration.json')


class PinEncoding(unittest.TestCase):
    def test_exact_types_and_unknown_yaml_rejected(self):
        self.assertEqual(typed(False), dict(type=1, value=False))
        self.assertEqual(typed(.5), dict(type=3, value=.5))
        with self.assertRaises(ValueError):
            typed([False, 0])
        with self.assertRaises(ValueError):
            select_parameters({'/**': {'ros__parameters': {'unsafe': True}}}, 'worker')
        self.assertEqual(select_parameters({'other': {'ros__parameters': {'unsafe': True}}}, 'worker'), {})


@unittest.skipUnless(os.environ.get('NEUPAN_LAUNCH_CONTRACT_TEST') == '1', 'selected bundle opt-in only')
class InstalledControlBuilders(unittest.TestCase):
    def test_direct_smoothed_mesh_and_collision_matrix(self):
        from launch.actions import ExecuteProcess, OpaqueFunction
        from launch_ros.actions import Node
        rows = []
        with patch.object(ExecuteProcess, 'execute', side_effect=AssertionError('process forbidden')), \
             patch.object(Node, 'execute', side_effect=AssertionError('node forbidden')), \
             patch.object(OpaqueFunction, 'execute', side_effect=AssertionError('opaque visitor forbidden')), \
             patch('subprocess.Popen', side_effect=AssertionError('subprocess forbidden')):
            for profile in ('direct', 'smoothed'):
                for mesh in (False, True):
                    for collision in (False, True):
                        data = compile_contract(BUNDLE, CALIBRATION, profile, mesh, collision)
                        nodes = data['pins']['nodes']
                        get = lambda n, k: nodes[n]['parameters'][k]['value']
                        self.assertEqual('/neupan_velocity_smoother' in nodes, profile == 'smoothed')
                        self.assertEqual(get('/controller_server', 'use_sim_time'), False)
                        self.assertEqual(get('/bt_navigator', 'use_sim_time'), False)
                        self.assertEqual(get('/controller_server', 'FollowPath.shadow_mode'), False)
                        self.assertEqual(get('/n1_16_production_bridge', 'worker_recovery_enabled'), False)
                        self.assertNotIn('robot_type', nodes['/n1_16_production_bridge']['parameters'])
                        self.assertIn('robot_type', data['informational_launch_inputs']['/n1_16_production_bridge'])
                        self.assertEqual(get('/n1_16_production_bridge', 'scan_topic'),
                                         '/scan_neupan_mesh_shadow' if mesh else '/scan_neupan')
                        self.assertEqual(get('/r1_collision_guard', 'collision_check_enabled'), collision)
                        self.assertEqual(get('/r1_collision_guard', 'cmd_vel_out_topic'), '/cmd_vel')
                        self.assertEqual(get('/r1_collision_guard', 'cmd_vel_in_topic'),
                                         '/neupan_nav2_smoothed_cmd_vel' if profile == 'smoothed' else '/neupan_nav2_cmd_vel')
                        self.assertEqual(data['remappings']['/controller_server'],
                                         [['cmd_vel', '/neupan_nav2_raw_cmd_vel' if profile == 'smoothed' else '/neupan_nav2_cmd_vel']])
                        self.assertFalse(data['review_complete'])
                        self.assertFalse(data['motion_authorized'])
                        rows.append(data)
        report = os.environ.get('NEUPAN_LAUNCH_CONTRACT_REPORT')
        if report:
            import json
            with Path(report).open('x') as stream:
                json.dump(rows, stream, indent=2)


if __name__ == '__main__':
    unittest.main()
