#!/usr/bin/env python3
"""Exercise the real shell argument construction without starting ROS nodes."""
import subprocess
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools/starterbutton/start_navigation_high_sampling_neupan.sh'

class ButtonArguments(unittest.TestCase):
    def args(self, attachment):
        script = SCRIPT.read_text()
        setup = script.split('extra_mesh_launch_args=()', 1)[1].split('/usr/bin/python3 "${script_dir}/launch_with_parent_death.py"', 1)[0]
        command = script.split('/usr/bin/python3 "${script_dir}/launch_with_parent_death.py"', 1)[1].split('  local_ip:="${arm_local_ip}" 9>&- &', 1)[0]
        shell = '''arm_extra_mesh_config="$1"
scan_source=dual
mesh_self_filter=true
mesh_filter_mode=3d
mesh_3d_projection_margin_m=0.0
left_arm_ip=192.168.11.60
right_arm_ip=192.168.11.61
arm_local_ip=192.168.11.11
extra_mesh_launch_args=()
''' + setup + '\nargs=( ' + command + ' local_ip:="${arm_local_ip}" )\nprintf "%s\\0" "${args[@]}"\n'
        result = subprocess.check_output(['/bin/bash', '-c', shell, 'test', attachment])
        return [a.decode() for a in result.split(b'\0') if a]

    def test_unconfigured_optional_value_is_omitted(self):
        args = self.args('')
        self.assertIn('mesh_filter_mode:=3d', args)
        self.assertIn('mesh_3d_projection_margin_m:=0.0', args)
        self.assertFalse(any(a.startswith('arm_extra_mesh_config:=') for a in args))
        self.assertFalse(any(a.endswith(':=') for a in args))

    def test_configured_value_is_one_argument(self):
        args = self.args('/tmp/held object.yaml')
        self.assertIn('arm_extra_mesh_config:=/tmp/held object.yaml', args)

if __name__ == '__main__':
    unittest.main()
