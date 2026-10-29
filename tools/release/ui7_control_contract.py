#!/usr/bin/env python3
"""Trace UI7 control argument propagation only; never run processes or arm probes.

This intentionally evaluates the navigation-construction callback for a synthetic
successful arm-state gate. It neither runs nor certifies that hardware gate.
"""
import argparse
import hashlib
import importlib.util
import json
import shlex
from pathlib import Path
from types import SimpleNamespace

from launch_parameter_contract import compile_contract


def button_options(environment):
    allowed = {'RPP_NEUPAN_SCAN_SOURCE', 'RPP_NEUPAN_USE_MESH_SELF_FILTER',
               'RPP_NEUPAN_COLLISION_CHECK_ENABLED', 'RPP_NEUPAN_COLLISION_CHECK_DEFAULT'}
    if set(environment) - allowed or any(not isinstance(v, str) for v in environment.values()):
        raise ValueError('unsupported_UI7_environment_input')
    scan = environment.get('RPP_NEUPAN_SCAN_SOURCE') or 'dual'
    mesh = environment.get('RPP_NEUPAN_USE_MESH_SELF_FILTER') or 'true'
    collision = environment.get('RPP_NEUPAN_COLLISION_CHECK_ENABLED') or (
        environment.get('RPP_NEUPAN_COLLISION_CHECK_DEFAULT') or 'true')
    if scan not in ('dual', 'livox_only') or mesh not in ('true', 'false') or collision not in ('true', 'false'):
        raise ValueError('invalid_UI7_environment_override')
    return dict(scan_source=scan, use_mesh_self_filter=mesh, collision_check_enabled=collision)


def trace(bundle, calibration, environment):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    bundle = bundle.resolve(strict=True)
    manifest = json.loads((bundle / 'releases/a-plus/manifest.json').read_text())
    if manifest.get('excluded_components') != ['cognav']:
        raise ValueError('bundle_does_not_exclude_cognav')
    # Check before importing any of the bundle's Python launch source.
    for row in manifest['inventory']:
        path = (bundle / row['path']).resolve(strict=True)
        if not path.is_relative_to(bundle) or hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('bundle_inventory_changed:' + row['path'])
    navigation = Path(get_package_share_directory('navigation_2d')).resolve(strict=True)
    if not navigation.is_relative_to(bundle / 'install'):
        raise ValueError('selected_navigation_not_in_bundle')
    inputs = {}
    button = bundle / 'tools/starterbutton/start_navigation_high_sampling_neupan.sh'
    inputs[str(button)] = hashlib.sha256(button.read_bytes()).hexdigest()
    registration_path = bundle / 'tools/starterbutton/navigation7_button.json'
    inputs[str(registration_path)] = hashlib.sha256(registration_path.read_bytes()).hexdigest()
    registration = json.loads(registration_path.read_text())
    registered_environment = {}
    for assignment in shlex.split(registration['Additional_environment_variables']):
        name, separator, value = assignment.partition('=')
        if not separator or name in registered_environment:
            raise ValueError('unsupported_UI7_registration_assignment')
        registered_environment[name] = value
    button_options(registered_environment)
    def load(filename):
        path = navigation / 'launch' / filename
        inputs[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        spec = importlib.util.spec_from_file_location('_ui7_' + filename.replace('.', '_'), path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    def text(value, context):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    def declare(module, context):
        description = module.generate_launch_description()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return description
    def child_context(include, parent):
        child = LaunchContext()
        child.launch_configurations.update(parent.launch_configurations)
        child.launch_configurations.update({text(k, parent): text(v, parent)
                                            for k, v in include.launch_arguments})
        return child
    # Caller-supplied values represent explicitly reviewed effective overrides
    # after external startup configuration, not assumed UI application priority.
    button_options(environment)
    effective_environment = dict(registered_environment, **environment)
    options = button_options(effective_environment)
    context = LaunchContext()
    context.launch_configurations.update(options)
    context.launch_configurations.update(dict(local_controller_mode='neupan', start_arm_drivers='false',
                                             calibration_record=str(calibration.resolve(strict=True))))
    selector = load('local_and_nav_high_sampling_controller.launch.py')
    declare(selector, context)
    include, = selector._select(context)
    selected = Path(text(include.launch_description_source._LaunchDescriptionSource__location, context)).resolve()
    if selected != navigation / 'launch/local_and_nav_high_sampling_neupan.launch.py':
        raise ValueError('UI7_selected_unexpected_launch')
    context = child_context(include, context)
    intermediate = load('local_and_nav_high_sampling_neupan.launch.py')
    # Constructing the description is safe. Do not invoke its admit callback:
    # it probes real arm topics and may return hardware driver actions.
    description = declare(intermediate, context)
    opaque, = [a for a in description.entities if isinstance(a, OpaqueFunction)]
    admit = opaque._OpaqueFunction__function
    closure = dict(zip(admit.__code__.co_freevars, (c.cell_contents for c in admit.__closure__)))
    after_gate = closure['after_arm_gate']
    actions = after_gate(SimpleNamespace(returncode=0), context)
    targets = {}
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            location = action.launch_description_source._LaunchDescriptionSource__location
            # A PythonLaunchDescriptionSource normalizes its location lazily.
            location = text(location, context)
            name = Path(location).name
            if name in ('navigation_neupan.launch.py', 'r1_neupan_hw1_motion.launch.py'):
                targets[name] = child_context(action, context)
    if set(targets) != {'navigation_neupan.launch.py', 'r1_neupan_hw1_motion.launch.py'}:
        raise ValueError('missing_control_launch_include')
    nav = targets['navigation_neupan.launch.py'].launch_configurations
    motion = targets['r1_neupan_hw1_motion.launch.py'].launch_configurations
    if nav['command_profile'] != motion['command_profile'] or nav['mode'] != 'hw1':
        raise ValueError('inconsistent_UI7_control_profile')
    if motion['cmd_vel_out_topic'] != '/cmd_vel':
        raise ValueError('unexpected_UI7_final_output')
    actual = {key: motion[key] for key in options}
    if actual != options:
        raise ValueError('UI7_override_not_forwarded')
    contract = compile_contract(bundle, calibration, profile=motion['command_profile'],
                                mesh=motion['use_mesh_self_filter'] == 'true',
                                collision=motion['collision_check_enabled'] == 'true',
                                scan_source=motion['scan_source'])
    contract['inputs'].update(inputs)
    contract['ui7_trace'] = dict(button_options=options, nav_profile=nav['command_profile'],
                                motion_profile=motion['command_profile'], final_output=motion['cmd_vel_out_topic'],
                                arm_gate='synthetic success callback only; no arm probes/drivers run',
                                registered_environment=registered_environment,
                                reviewed_effective_overrides=environment,
                                registration_target=registration['bash_url'],
                                registration_points_to_bundle=Path(registration['bash_url']).expanduser().resolve().is_relative_to(bundle),
                                environment_scope='registered options plus reviewed effective overrides; .rpprc not executed')
    contract['scope'] = 'UI7 control propagation + control builders only; hardware admission/full process launch not certified'
    return contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--environment', type=Path, required=True, help='reviewed allowlisted UI7 environment JSON')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = trace(args.bundle, args.calibration, json.loads(args.environment.read_text()))
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result['ui7_trace']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
