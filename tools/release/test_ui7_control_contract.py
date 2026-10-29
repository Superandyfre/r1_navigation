import json
import os
from pathlib import Path
import re
import subprocess
import unittest
from unittest.mock import patch

from ui7_control_contract import button_options, trace

BUNDLE = Path('/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-07')
CALIBRATION = Path('/home/rpp/rpp_ws/config/r1_hw1_calibration.json')


class ButtonEnvironment(unittest.TestCase):
    def test_matches_real_button_assignments_without_sourcing_or_launching(self):
        script = (BUNDLE / 'tools/starterbutton/start_navigation_high_sampling_neupan.sh').read_text()
        assignments = [line for line in script.splitlines()
                       if re.match(r'^(scan_source|mesh_self_filter|collision_check_enabled)=', line)]
        self.assertEqual(len(assignments), 3)
        # Execute only these three known assignments, no .rpprc/source/launch.
        command = '\n'.join(assignments) + '\nprintf "%s\\n" "$scan_source" "$mesh_self_filter" "$collision_check_enabled"'
        for environment in [{}, {'RPP_NEUPAN_SCAN_SOURCE': ''},
                            {'RPP_NEUPAN_USE_MESH_SELF_FILTER': ''},
                            {'RPP_NEUPAN_COLLISION_CHECK_DEFAULT': 'false'},
                            {'RPP_NEUPAN_COLLISION_CHECK_ENABLED': '', 'RPP_NEUPAN_COLLISION_CHECK_DEFAULT': 'false'},
                            {'RPP_NEUPAN_COLLISION_CHECK_ENABLED': 'true', 'RPP_NEUPAN_COLLISION_CHECK_DEFAULT': 'false'}]:
            values = subprocess.check_output(['/bin/bash', '-c', command], env=environment, text=True).splitlines()
            self.assertEqual(list(button_options(environment).values()), values)
        with self.assertRaises(ValueError):
            button_options({'RPP_NEUPAN_SCAN_SOURCE': 'unknown'})
        with self.assertRaises(ValueError):
            button_options({'RPP_NEUPAN_COLLISION_CHECK_ENABLED': '1'})
        with self.assertRaises(ValueError):
            button_options({'RPP_NEUPAN_COMMAND_PROFILE': 'direct'})


@unittest.skipUnless(os.environ.get('NEUPAN_UI7_PROPAGATION_TEST') == '1', 'selected bundle opt-in only')
class UI7Propagation(unittest.TestCase):
    def test_root_and_intermediate_control_inputs(self):
        from launch.actions import ExecuteProcess, OpaqueFunction
        from launch_ros.actions import Node
        rows = []
        with patch.object(ExecuteProcess, 'execute', side_effect=AssertionError('process forbidden')), \
             patch.object(Node, 'execute', side_effect=AssertionError('node forbidden')), \
             patch.object(OpaqueFunction, 'execute', side_effect=AssertionError('opaque visitor forbidden')), \
             patch('subprocess.Popen', side_effect=AssertionError('hardware probe forbidden')):
            for scan in ('dual', 'livox_only'):
                for mesh in ('false', 'true'):
                    for collision in ('false', 'true'):
                        environment = {'RPP_NEUPAN_SCAN_SOURCE': scan,
                                       'RPP_NEUPAN_USE_MESH_SELF_FILTER': mesh,
                                       'RPP_NEUPAN_COLLISION_CHECK_ENABLED': collision}
                        data = trace(BUNDLE, CALIBRATION, environment)
                        info = data['ui7_trace']
                        self.assertEqual(info['button_options'], dict(scan_source=scan,
                            use_mesh_self_filter=mesh, collision_check_enabled=collision))
                        self.assertEqual(info['nav_profile'], 'smoothed')
                        self.assertEqual(info['motion_profile'], 'smoothed')
                        nodes = data['pins']['nodes']
                        self.assertEqual(nodes['/r1_collision_guard']['parameters']['collision_check_enabled']['value'],
                                         collision == 'true')
                        self.assertEqual(nodes['/n1_16_production_bridge']['parameters']['scan_topic']['value'],
                                         '/scan_neupan_mesh_shadow' if mesh == 'true' else '/scan_neupan')
                        self.assertEqual(data['remappings']['/controller_server'],
                                         [['cmd_vel', '/neupan_nav2_raw_cmd_vel']])
                        rows.append(data)
            default = trace(BUNDLE, CALIBRATION, {})
            self.assertEqual(default['ui7_trace']['registered_environment']['RPP_NEUPAN_COLLISION_CHECK_DEFAULT'], 'false')
            self.assertEqual(default['pins']['nodes']['/r1_collision_guard']['parameters']['collision_check_enabled']['value'], False)
            self.assertFalse(default['ui7_trace']['registration_points_to_bundle'])
            rows.append(default)
        report = os.environ.get('NEUPAN_UI7_PROPAGATION_REPORT')
        if report:
            with Path(report).open('x') as stream:
                json.dump(rows, stream, indent=2)


if __name__ == '__main__':
    unittest.main()
