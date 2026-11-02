#!/usr/bin/env python3
"""Stable UI7 entry: read and verify an immutable release on every invocation."""
import argparse
import hashlib
import json
import os
from pathlib import Path

SUPPORTED = {'A+context1-source2-grant1-health1-owner1',
             'A+context1-source2-grant1-health2-owner1'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def resolve(pointer):
    pin=json.loads(pointer.read_text())
    if pin.get('schema') != 'r1_ui7_release_pointer_v1':raise ValueError('invalid_pointer_schema')
    bundle=Path(pin['bundle']).resolve(strict=True)
    manifest_path=bundle/'releases/a-plus/manifest.json'
    if digest(manifest_path) != pin['manifest_sha256']:raise ValueError('manifest_hash_mismatch')
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('protocol_set') not in SUPPORTED:raise ValueError('unsupported_protocol_set')
    if Path(manifest['inventory_root']).resolve(strict=True) != bundle:raise ValueError('inventory_root_mismatch')
    inventory=manifest['inventory']
    if not inventory:raise ValueError('missing_inventory')
    paths=set()
    for row in inventory:
        path=(bundle/row['path']).resolve(strict=True)
        if not path.is_relative_to(bundle) or digest(path) != row['sha256']:
            raise ValueError('inventory_mismatch:'+row['path'])
        paths.add(path)
    entry=bundle/'tools/starterbutton/start_navigation_high_sampling_neupan.sh'
    if entry.resolve(strict=True) not in paths:raise ValueError('entry_not_in_inventory')
    for role,row in manifest['artifacts'].items():
        path=Path(row['path']).resolve(strict=True)
        if path not in paths or digest(path) != row['sha256']:
            raise ValueError('artifact_mismatch:'+role)
    return entry,dict(bundle=str(bundle),build_source_sha=manifest['build_source_sha'],
                      tooling_source_sha=manifest['tooling_source_sha'],protocol_set=manifest['protocol_set'],
                      manifest_sha256=pin['manifest_sha256'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pointer',type=Path,default=Path(__file__).resolve().parents[2]/'deployment/profiles/ui7-current.json')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    try:entry,info=resolve(args.pointer)
    except (ValueError,KeyError,OSError,TypeError) as exc:
        print('UI7 release refused: '+str(exc),flush=True);return 2
    print(json.dumps(info,indent=2),flush=True)
    if not args.check_only:os.execv('/bin/bash',['bash',str(entry)])
    return 0


if __name__=='__main__':raise SystemExit(main())
