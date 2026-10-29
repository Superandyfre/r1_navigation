#!/usr/bin/env python3
"""Evaluate pinned HW1 control launch inputs without executing ROS actions.

Output is a review draft, not approval. Only the two control launch builders
are evaluated: no full UI7 launch, arm probes, drivers, lifecycle transitions,
Node.execute or LaunchService. Humble's normalized Node fields are inspected.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import yaml

from effective_parameters import SCHEMA, validate
from runtime_topology import control_contract


def typed(value):
    kinds = {bool: 1, int: 2, float: 3, str: 4}
    if type(value) in kinds:
        return dict(type=kinds[type(value)], value=value)
    if isinstance(value, (list, tuple)) and value:
        if type(value[0]) not in kinds or any(type(v) is not type(value[0]) for v in value):
            raise ValueError('unsupported_mixed_parameter_array')
        return dict(type=kinds[type(value[0])] + 5, value=list(value))
    raise ValueError('unsupported_or_untyped_parameter_value:' + repr(value))


def flatten(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = typed(value)
    return result


def select_parameters(document, name):
    """This launch uses exact root names only; reject wildcard ambiguity."""
    if any('*' in key for key in document):
        raise ValueError('wildcard_yaml_requires_explicit_review')
    selected = document.get(name, document.get('/' + name, {}))
    return flatten(selected.get('ros__parameters', {}))


def compile_contract(bundle, calibration, profile='smoothed', mesh=True, collision=True, scan_source='dual'):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    bundle = bundle.resolve(strict=True)
    bridge = Path(get_package_share_directory('navigation_neupan_bridge')).resolve(strict=True)
    if not bridge.is_relative_to(bundle / 'install'):
        raise ValueError('selected_bridge_not_in_bundle')
    for package in ('r1_neupan_integration', 'r1_collision_guard'):
        if not Path(get_package_share_directory(package)).resolve(strict=True).is_relative_to(bundle / 'install'):
            raise ValueError('selected_package_not_in_bundle:' + package)
    manifest = json.loads((bundle / 'releases/a-plus/manifest.json').read_text())
    if manifest.get('excluded_components') != ['cognav']:
        raise ValueError('bundle_does_not_exclude_cognav')
    for row in manifest['inventory']:
        path = (bundle / row['path']).resolve(strict=True)
        if not path.is_relative_to(bundle) or hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('bundle_inventory_changed:' + row['path'])
    pins = dict(schema=SCHEMA, nodes={})
    evidence = {}
    remaps = {}
    informational_inputs = {}

    def record(path):
        path = Path(path).resolve(strict=True)
        evidence[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

    record(calibration)
    for filename, wanted, overrides in [
        ('r1_neupan_hw1_motion.launch.py', {'n1_16_production_bridge', 'r1_collision_guard'},
         dict(calibration_record=str(calibration.resolve(strict=True)), scan_source=scan_source,
              command_profile=profile, use_mesh_self_filter=str(mesh).lower(),
              collision_check_enabled=str(collision).lower())),
        ('navigation_neupan.launch.py', {'controller_server', 'neupan_velocity_smoother', 'bt_navigator'},
         dict(mode='hw1', command_profile=profile)),
    ]:
        path = bridge / 'launch' / filename
        record(path)
        spec = importlib.util.spec_from_file_location('_contract_' + filename.replace('.', '_'), path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        # Execute argument declarations only. Never visit OpaqueFunction,
        # includes, timers, event handlers, process actions or launch service.
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        for node in module._build(context):
            if not isinstance(node, Node):
                continue
            name = perform_substitutions(context, normalize_to_list_of_substitutions(node._Node__node_name or ''))
            if name not in wanted:
                continue
            parameters = {}
            for value in evaluate_parameters(context, node._Node__parameters):
                if isinstance(value, Path):
                    # RewrittenYaml files are temporary. Also pin the original
                    # input, not only the generated file's transient pathname.
                    if filename == 'navigation_neupan.launch.py':
                        record(bridge / 'config/nav2_params_r1_neupan_hw1.yaml')
                    else:
                        record(value)
                    parameters.update(select_parameters(yaml.safe_load(value.read_text()), name))
                elif isinstance(value, dict):
                    parameters.update(flatten(value))
                else:
                    raise ValueError('unsupported_evaluated_parameter_input')
            # These historical YAML entries describe the robot but are not
            # declared/read by N115/N116. Preserve them as informational input,
            # never represent them as effective runtime settings.
            if name == 'n1_16_production_bridge':
                informational_inputs['/' + name] = {
                    key: parameters.pop(key) for key in ('robot_type', 'robot_description')
                    if key in parameters}
            pins['nodes']['/' + name] = dict(parameters=parameters)
            remaps['/' + name] = [
                [perform_substitutions(context, left), perform_substitutions(context, right)]
                for left, right in node._Node__remappings or []]
    expected = {'/controller_server', '/r1_collision_guard', '/n1_16_production_bridge', '/bt_navigator'}
    if profile == 'smoothed':
        expected.add('/neupan_velocity_smoother')
    if set(pins['nodes']) != expected:
        raise ValueError('missing_expected_control_nodes')
    validate(pins)
    description = Path(get_package_share_directory('r1_description')) / 'config/r1_lidar_extrinsics.yaml'
    record(description)
    return dict(pins=pins, remappings=remaps, topology=control_contract(pins, remaps), inputs=evidence,
                informational_launch_inputs=informational_inputs,
                launch_arguments=dict(scan_source=scan_source, command_profile=profile,
                                      use_mesh_self_filter=mesh, collision_check_enabled=collision),
                build_source_sha=manifest['build_source_sha'], tooling_source_sha=manifest['tooling_source_sha'],
                review_complete=False, motion_authorized=False,
                scope='two control builders only; upstream UI7 propagation and runtime remaps not certified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--profile', choices=['direct', 'smoothed'], default='smoothed')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compile_contract(args.bundle, args.calibration, args.profile)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(nodes=list(result['pins']['nodes']), review_complete=False, motion_authorized=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
