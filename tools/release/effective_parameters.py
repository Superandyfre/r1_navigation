#!/usr/bin/env python3
"""Read live ROS parameters against independent pins. Never sets, arms or launches.

This proves only the named parameter values during the sampling interval, not
topic remappings, loaded code, physical readiness or continuous authorization.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

SCHEMA = 'r1_neupan_parameter_pins_v1'
FIELDS = {1: 'bool_value', 2: 'integer_value', 3: 'double_value',
          4: 'string_value', 5: 'byte_array_value', 6: 'bool_array_value',
          7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value'}


def valid_value(kind, value):
    scalar = {1: bool, 2: int, 3: float, 4: str}
    if kind in scalar:
        return (type(value) is scalar[kind] and
                (kind != 3 or math.isfinite(value)) and
                (kind != 2 or -(2**63) <= value < 2**63))
    if type(value) is not list:
        return False
    if kind == 5:
        return all(type(v) is int and 0 <= v <= 255 for v in value)
    element = {6: 1, 7: 2, 8: 3, 9: 4}.get(kind)
    return element is not None and all(valid_value(element, v) for v in value)


def validate(pins):
    if pins.get('schema') != SCHEMA or not pins.get('nodes'):
        raise ValueError('invalid_parameter_pins')
    for name, spec in pins['nodes'].items():
        if not name.startswith('/') or name.endswith('/') or '//' in name:
            raise ValueError('node_name_must_be_fully_qualified:' + name)
        if not isinstance(spec.get('parameters'), dict) or not spec['parameters']:
            raise ValueError('empty_parameter_pins:' + name)
        for key, pin in spec['parameters'].items():
            if (not key or type(pin.get('type')) is not int or
                    not valid_value(pin['type'], pin.get('value'))):
                raise ValueError('invalid_typed_parameter:' + name + ':' + key)
        for prefix in spec.get('closed_prefixes', []):
            if not isinstance(prefix, str) or not prefix:
                raise ValueError('invalid_closed_prefix:' + name)
            if not any(k == prefix or k.startswith(prefix + '.') for k in spec['parameters']):
                raise ValueError('unpinned_closed_prefix:' + name + ':' + prefix)


def compare(pins, observed):
    validate(pins)
    errors = []
    for name, spec in pins['nodes'].items():
        actual = observed.get(name, {})
        parameters = actual.get('parameters', {})
        for key, pin in spec['parameters'].items():
            value = parameters.get(key)
            if value is None:
                errors.append(name + ':' + key + ':missing')
            elif (type(value.get('type')) is not int or value.get('type') != pin['type'] or
                  not valid_value(value.get('type'), value.get('value')) or
                  value['value'] != pin['value']):
                errors.append(name + ':' + key + ':mismatch')
        for prefix in spec.get('closed_prefixes', []):
            extra = {k for k in actual.get('names', [])
                     if k == prefix or k.startswith(prefix + '.')} - set(spec['parameters'])
            errors.extend(name + ':' + key + ':unpinned' for key in sorted(extra))
    return errors


def sample(node, pins, timeout_sec=5., pump=None):
    """Collect fresh responses; caller may pump its isolated sensor fixture."""
    from rcl_interfaces.srv import GetParameters, ListParameters
    validate(pins)
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('invalid_collection_timeout')
    started = time.monotonic()
    deadline = started + timeout_sec
    clients = []
    observed, errors = {}, []
    executor = None
    if pump is None:
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)

    def tick():
        if pump is None:
            executor.spin_once(timeout_sec=min(.02, max(0., deadline-time.monotonic())))
        else:
            pump()

    def call(client, request):
        if time.monotonic() >= deadline:
            raise TimeoutError('parameter_collection_deadline')
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError('parameter_service_unavailable')
            tick()
        future = client.call_async(request)
        try:
            while not future.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError('parameter_response_timeout')
                tick()
            if time.monotonic() >= deadline:
                raise TimeoutError('parameter_collection_deadline')
            return future.result()
        finally:
            if not future.done():
                client.remove_pending_request(future)

    try:
        # Duplicate fully-qualified node names make service replies ambiguous.
        for name, spec in pins['nodes'].items():
            try:
                while True:
                    names = [ns.rstrip('/') + '/' + n for n, ns in node.get_node_names_and_namespaces()]
                    count = names.count(name)
                    if count > 1:
                        raise ValueError('duplicate_node_name')
                    if count == 1:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('node_not_discovered')
                    tick()
                listing = node.create_client(ListParameters, name + '/list_parameters')
                getter = node.create_client(GetParameters, name + '/get_parameters')
                clients.extend([listing, getter])
                response = call(listing, ListParameters.Request(prefixes=[], depth=0))
                names = list(response.result.names)
                keys = sorted(spec['parameters'])
                response = call(getter, GetParameters.Request(names=keys))
                if len(response.values) != len(keys):
                    raise ValueError('parameter_response_length')
                values = {}
                for key, value in zip(keys, response.values):
                    kind = value.type
                    raw = getattr(value, FIELDS[kind]) if kind in FIELDS else None
                    if kind in FIELDS and kind >= 5:
                        raw = [v[0] if isinstance(v, bytes) else v for v in raw]
                    values[key] = dict(type=kind, value=raw)
                if any(key not in names for key in keys):
                    raise ValueError('pinned_parameter_not_listed')
                names_after = [ns.rstrip('/') + '/' + n for n, ns in node.get_node_names_and_namespaces()]
                if names_after.count(name) != 1:
                    raise ValueError('node_identity_ambiguous_after_sample')
                observed[name] = dict(names=names, parameters=values,
                                      received_monotonic=time.monotonic())
            except (TimeoutError, ValueError, RuntimeError) as exc:
                errors.append(name + ':' + str(exc))
        errors.extend(compare(pins, observed))
    finally:
        for client in clients:
            node.destroy_client(client)
        if executor is not None:
            executor.remove_node(node)
            executor.shutdown()
    return dict(ok=not errors, errors=errors, observed=observed,
                started_monotonic=started, finished_monotonic=time.monotonic(),
                scope='named live parameter values only; not remaps/code/arm/physical readiness',
                motion_authorized=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pins', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=5.)
    args = parser.parse_args()
    # Validate independent pins and reserve a new report before contacting ROS.
    raw = args.pins.read_bytes()
    pins = json.loads(raw)
    validate(pins)
    import rclpy
    with args.output.open('x') as stream:
        rclpy.init(args=[])
        node = rclpy.create_node('neupan_parameter_verifier')
        try:
            result = sample(node, pins, args.timeout_sec)
            result['pins_sha256'] = hashlib.sha256(raw).hexdigest()
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(dict(ok=result['ok'], errors=result['errors'], motion_authorized=False)))
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
