#!/usr/bin/env python3
"""Compose read-only release/maps/parameter/graph evidence. Never arms.

An OK result means only these bounded checks agreed. Worker live import/PID
binding, full dependency closure, physical inhibit and continuous admission
are separate requirements; this tool cannot approve deployment or arm motion.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import effective_parameters
import preflight
import runtime_maps
import runtime_topology


def input_hashes(contract):
    if not isinstance(contract.get('inputs'), dict) or not contract['inputs']:
        raise ValueError('missing_independent_input_hashes')
    result = {}
    for name, expected in contract['inputs'].items():
        path = Path(name)
        if not path.is_absolute():
            raise ValueError('input_path_not_absolute')
        resolved = path.resolve(strict=True)
        before = resolved.stat()
        digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
        after = resolved.stat()
        if ((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) !=
                (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns) or
                path.resolve(strict=True) != resolved):
            raise ValueError('input_changed_during_hash:' + name)
        if digest != expected:
            raise ValueError('input_hash_mismatch:' + name)
        result[name] = dict(resolved=str(resolved), sha256=digest)
    return result


def audit(node, manifest, bindings, contract, timeout_sec=5., pump=None):
    started = time.monotonic()
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError('invalid_audit_timeout')
    # JSON round-trip freezes caller-owned mutable input objects for this audit.
    frozen = json.dumps(dict(manifest=manifest, bindings=bindings, contract=contract),
                        sort_keys=True, allow_nan=False)
    pinned = json.loads(frozen)
    manifest, bindings, contract = (pinned[k] for k in ('manifest', 'bindings', 'contract'))
    checks = {}
    errors = []

    def collect(name, function):
        try:
            result = function()
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            result = dict(ok=False, errors=[type(exc).__name__ + ':' + str(exc)])
        checks[name] = result
        if not result['ok']:
            errors.extend(name + ':' + error for error in result['errors'])
        return result['ok']

    def inputs():
        effective_parameters.validate(contract['pins'])
        runtime_topology.validate(contract['topology'])
        return dict(ok=True, errors=[], observed=input_hashes(contract))

    # Reject static/mapped identity failures before querying ROS. No unbounded
    # retries and no lifecycle/arm calls are made by this tool.
    if collect('inputs_before', inputs):
        if collect('release_before', lambda: preflight.check(manifest, compatibility_only=True)):
            if collect('maps_before', lambda: runtime_maps.check(manifest, bindings)):
                def parameters():
                    remaining = timeout_sec - (time.monotonic() - started)
                    if remaining <= 0:
                        raise ValueError('audit_deadline_before_parameters')
                    return effective_parameters.sample(node, contract['pins'], remaining, pump=pump)
                if collect('parameters', parameters):
                    collect('topology', lambda: runtime_topology.check(node, contract['topology']))
                collect('maps_after', lambda: runtime_maps.check(manifest, bindings))
                collect('inputs_after', inputs)
                collect('release_after', lambda: preflight.check(manifest, compatibility_only=True))
    finished = time.monotonic()
    if finished - started > timeout_sec:
        errors.append('audit_deadline_exceeded')
    return dict(ok=not errors, errors=errors, checks=checks,
                started_monotonic=started, finished_monotonic=finished,
                audit_input_sha256=hashlib.sha256(frozen.encode()).hexdigest(),
                deployment_approved=False, motion_authorized=False, coverage_complete=False,
                remaining=['live Worker import/PID binding', 'full external dependency closure',
                           'production launch inputs/defaults review', 'physical inhibit/stop evidence',
                           'continuous admission and production arm integration'],
                scope='bounded read-only consistency audit; not a reusable arm permit')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--bindings', type=Path, required=True)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=5.)
    args = parser.parse_args()
    data = {name: json.loads(getattr(args, name).read_text())
            for name in ('manifest', 'bindings', 'contract')}
    import rclpy
    with args.output.open('x') as stream:
        rclpy.init(args=[])
        node = rclpy.create_node('neupan_runtime_admission_auditor')
        try:
            result = audit(node, **data, timeout_sec=args.timeout_sec)
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(dict(ok=result['ok'], errors=result['errors'], motion_authorized=False)))
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
