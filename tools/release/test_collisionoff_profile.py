import ast
import importlib.util
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]


class CollisionOffProfile(unittest.TestCase):
    def test_hw1_all_entry_defaults_disable_duplicate_collision_layer(self):
        for name in ['packages/navigation_2d/launch/local_and_nav_high_sampling_controller.launch.py',
                     'packages/navigation_2d/launch/local_and_nav_high_sampling_neupan.launch.py',
                     'packages/navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py']:
            tree = ast.parse((ROOT / name).read_text())
            calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                     and n.args and isinstance(n.args[0], ast.Constant)
                     and n.args[0].value == 'collision_check_enabled']
            self.assertEqual(len(calls), 1)
            values = {k.arg: ast.literal_eval(k.value) for k in calls[0].keywords}
            self.assertEqual(values['default_value'], 'false')
            self.assertEqual(values['choices'], ['false'])
        script = (ROOT / 'tools/starterbutton/start_navigation_high_sampling_neupan.sh').read_text()
        assignment = next(line for line in script.splitlines() if line.startswith('collision_check_enabled='))
        result = subprocess.check_output(['/bin/bash', '-c', assignment + '\nprintf "%s" "$collision_check_enabled"'], env={}, text=True)
        self.assertEqual(result, 'false')
        self.assertIn('if [[ "${collision_check_enabled}" != "false" ]]', script)

    def test_actual_hw1_builder_rejects_enable_and_detaches_guard_geometry(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        from launch_ros.utilities import evaluate_parameters
        path = ROOT / 'packages/navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py'
        spec = importlib.util.spec_from_file_location('collisionoff_source_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations['calibration_record'] = '/home/rpp/rpp_ws/config/r1_hw1_calibration.json'
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        nodes = module._build(context)
        guard = next(node for node in nodes if perform_substitutions(context,
            normalize_to_list_of_substitutions(node._Node__node_name)) == 'r1_collision_guard')
        override = evaluate_parameters(context, guard._Node__parameters)[-1]
        self.assertFalse(override['collision_check_enabled'])
        self.assertFalse(override['use_exact_mesh_components'])
        self.assertFalse(override['require_dynamic_footprint'])
        context.launch_configurations['collision_check_enabled'] = 'true'
        with self.assertRaisesRegex(RuntimeError, 'duplicate collision checks were removed'):
            module._build(context)


if __name__ == '__main__':
    unittest.main()
