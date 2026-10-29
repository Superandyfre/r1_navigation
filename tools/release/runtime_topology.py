#!/usr/bin/env python3
"""Read resolved ROS graph edges; never publishes, sets parameters or arms."""
import time


def validate(contract):
    if not isinstance(contract.get('nodes'), dict) or not contract['nodes']:
        raise ValueError('empty_topology_contract')
    for fqn, spec in contract['nodes'].items():
        if not fqn.startswith('/') or fqn.endswith('/') or '//' in fqn:
            raise ValueError('invalid_node_name')
        if not any(spec.get(k) for k in ('publishers', 'subscriptions')):
            raise ValueError('empty_node_edges')
        for key in ('publishers', 'subscriptions'):
            for topic, msgtype in spec.get(key, {}).items():
                if not topic.startswith('/') or '/msg/' not in msgtype:
                    raise ValueError('invalid_topic_or_message_type')
    if not contract.get('exclusive_publishers'):
        raise ValueError('empty_exclusive_publisher_contract')
    for topic, owner in contract['exclusive_publishers'].items():
        if owner not in contract['nodes'] or topic not in contract['nodes'][owner].get('publishers', {}):
            raise ValueError('exclusive_owner_not_pinned_publisher')


def check(node, contract):
    validate(contract)
    errors, observed = [], {}
    names = [ns.rstrip('/') + '/' + name for name, ns in node.get_node_names_and_namespaces()]
    for fqn, spec in contract['nodes'].items():
        if names.count(fqn) != 1:
            errors.append(fqn + ':node_missing_or_ambiguous')
            continue
        namespace, _, name = fqn.rpartition('/')
        try:
            pubs = dict(node.get_publisher_names_and_types_by_node(name, namespace or '/'))
            subs = dict(node.get_subscriber_names_and_types_by_node(name, namespace or '/'))
            observed[fqn] = dict(publishers=pubs, subscriptions=subs)
            for key, actual in [('publishers', pubs), ('subscriptions', subs)]:
                for topic, msgtype in spec.get(key, {}).items():
                    if topic not in actual or actual[topic] != [msgtype]:
                        errors.append(fqn + ':' + key + ':' + topic + ':missing_or_wrong_type')
        except Exception as exc:
            errors.append(fqn + ':graph_query:' + type(exc).__name__)
    exclusive = {}
    for topic, owner in contract.get('exclusive_publishers', {}).items():
        endpoints = node.get_publishers_info_by_topic(topic)
        owners = [info.node_namespace.rstrip('/') + '/' + info.node_name for info in endpoints]
        exclusive[topic] = owners
        if owners != [owner]:
            errors.append(topic + ':publisher_not_exclusive')
    return dict(ok=not errors, errors=errors, observed=observed, exclusive_publishers=exclusive,
                sampled_monotonic=time.monotonic(), motion_authorized=False,
                scope='discovered resolved endpoints only; not transport delivery or PID identity')


def control_contract(pins, remappings, smoother='/neupan_velocity_smoother'):
    """Expected graph derives from reviewed control inputs, never observed graph."""
    nodes = pins['nodes']
    get = lambda n, k: nodes[n]['parameters'][k]['value']
    controller, guard, worker = '/controller_server', '/r1_collision_guard', '/n1_16_production_bridge'
    string, twist = 'std_msgs/msg/String', 'geometry_msgs/msg/Twist'
    raw = dict(remappings[controller])['cmd_vel']
    final = get(guard, 'cmd_vel_out_topic')
    source = get(worker, 'planner_result_topic')
    health = get(worker, 'worker_health_topic')
    context = get(worker, 'execution_context_topic')
    grant = get(controller, 'FollowPath.motion_grant_topic')
    result = dict(nodes={
        controller: dict(publishers={raw: twist, grant: string, context: string},
                         subscriptions={source: string, health: string}),
        worker: dict(publishers={source: string, health: string}, subscriptions={context: string}),
        guard: dict(publishers={final: twist}, subscriptions={source: string, health: string,
                    grant: string, get(guard, 'cmd_vel_in_topic'): twist,
                    get(guard, 'source_cmd_vel_topic'): twist}),
    }, exclusive_publishers={final: guard, source: worker, health: worker, raw: controller})
    if smoother in nodes:
        mappings = dict(remappings[smoother])
        result['nodes'][smoother] = dict(publishers={mappings['cmd_vel_smoothed']: twist},
                                         subscriptions={mappings['cmd_vel']: twist})
        result['exclusive_publishers'][mappings['cmd_vel_smoothed']] = smoother
    return result


def main():
    import argparse
    import hashlib
    import json
    import math
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=3.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        parser.error('timeout must be finite and positive')
    raw = args.contract.read_bytes()
    data = json.loads(raw)
    contract = data.get('topology', data)
    validate(contract)
    import rclpy
    with args.output.open('x') as stream:
        rclpy.init(args=[])
        node = rclpy.create_node('neupan_topology_verifier')
        try:
            deadline = time.monotonic() + args.timeout_sec
            while True:
                result = check(node, contract)
                if result['ok'] or time.monotonic() >= deadline:
                    break
                rclpy.spin_once(node, timeout_sec=min(.02, max(0., deadline-time.monotonic())))
            result['contract_sha256'] = hashlib.sha256(raw).hexdigest()
            json.dump(result, stream, indent=2)
            stream.write('\n')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(dict(ok=result['ok'], errors=result['errors'], motion_authorized=False)))
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
