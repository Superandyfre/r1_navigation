#!/usr/bin/env python3
"""Read-only compatibility gate against an independently reviewed pinned manifest.

No discovery command writes a manifest or approves a release. The caller must
protect/review the manifest: hashes prove consistency, not safety or provenance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

# All are mandatory, preventing a Python-only or Guard-only release snapshot.
ROLES = {
 'controller_server': ('nav2_controller','lib/nav2_controller/controller_server'),
 'controller_core': ('library','libcontroller_server_core.so'),
 'controller_plugin': ('library','libnavigation_neupan_bridge.so'),
 'guard': ('r1_collision_guard','lib/r1_collision_guard/r1_collision_guard_node'),
 'worker_node': ('python','r1_neupan_integration/n1_16_node.py'),
 'worker_bridge': ('python','r1_neupan_integration/n1_16_bridge.py'),
 'worker_context': ('python','r1_neupan_integration/execution_protocol.py'),
 'bt_plugin': ('library','libneupan_wait_condition_bt_node.so'),
 'bt_pose': ('navigation_neupan_bridge','share/navigation_neupan_bridge/config/navigate_w_replanning_neupan.xml'),
 'bt_poses': ('navigation_neupan_bridge','share/navigation_neupan_bridge/config/navigate_through_poses_neupan.xml'),
 'nav_config': ('navigation_neupan_bridge','share/navigation_neupan_bridge/config/nav2_params_r1_neupan_hw1.yaml'),
 'launch': ('navigation_neupan_bridge','share/navigation_neupan_bridge/launch/r1_neupan_hw1_motion.launch.py'),
}


def resolve(role, env, python_path):
    kind, relative = ROLES[role]
    if kind == 'library':
        roots=[Path(p) for p in env.get('LD_LIBRARY_PATH','').split(':') if p]
    elif kind == 'python':
        roots=[Path(p or os.getcwd()) for p in python_path]
    else:
        roots=[]
        for value in env.get('AMENT_PREFIX_PATH','').split(':'):
            if not value:continue
            prefix=Path(value)
            if (prefix/'share/ament_index/resource_index/packages'/kind).is_file():
                roots=[prefix];break
    for root in roots:
        path=root/relative
        if kind == 'python' and (root/'r1_neupan_integration/__init__.py').is_file():
            if path.is_file():return path.resolve()
            raise ValueError('module_missing_in_selected_package '+role)
        if kind != 'python' and path.is_file():return path.resolve()
    raise ValueError('unresolved '+role)


def check(manifest, env=None, python_path=None, compatibility_only=False):
    env=os.environ if env is None else env
    python_path=sys.path if python_path is None else python_path
    errors=[];observed={}
    if manifest.get('schema')!='r1_neupan_release_v1':errors.append('invalid_manifest_schema')
    if manifest.get('protocol_set')!='A+context1-source2-grant1-health1-owner1':errors.append('incompatible_protocol_set')
    artifacts=manifest.get('artifacts',{})
    if not isinstance(artifacts,dict):artifacts={};errors.append('invalid_artifacts')
    for role in ROLES:
        try:
            pin=artifacts[role]
            actual=resolve(role,env,python_path)
            expected=Path(pin['path']).resolve(strict=True)
            digest=hashlib.sha256(actual.read_bytes()).hexdigest()
            observed[role]=dict(path=str(actual),sha256=digest)
            if actual!=expected:errors.append(role+': resolved_path_mismatch')
            if digest!=pin['sha256']:errors.append(role+': hash_mismatch')
        except (KeyError,TypeError,ValueError,OSError) as exc:
            errors.append(role+': '+str(exc))
    inventory=manifest.get('inventory', [])
    if inventory:
        try:
            root=Path(manifest['inventory_root']).resolve(strict=True)
            for row in inventory:
                path=(root/row['path']).resolve(strict=True)
                if not path.is_relative_to(root):raise ValueError('inventory_path_escape')
                if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
                    errors.append('inventory_hash_mismatch:'+row['path'])
        except (KeyError,TypeError,ValueError,OSError) as exc:errors.append('inventory:'+str(exc))
    if not compatibility_only:
        if not inventory:errors.append('release_inventory_missing')
        if manifest.get('deployment_approved') is not True:errors.append('release_not_approved')
        if manifest.get('build_testing') is not False:errors.append('production_requires_BUILD_TESTING_OFF')
        for gate in ['full_no_motion_acceptance','physical_stop_acceptance','rollback_rehearsal']:
            if manifest.get('gates',{}).get(gate) is not True:errors.append('gate_incomplete:'+gate)
    return dict(ok=not errors,mode='compatibility_only' if compatibility_only else 'deployment',
                errors=errors,observed=observed,
                runtime_loaded_maps_verified=False,physical_stop_verified_by_this_tool=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--compatibility-only',action='store_true')
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    try:result=check(json.loads(args.manifest.read_text()),compatibility_only=args.compatibility_only)
    except (OSError,ValueError,TypeError) as exc:result=dict(ok=False,errors=[str(exc)])
    if args.command and args.compatibility_only:
        result['ok']=False;result.setdefault('errors',[]).append('diagnostic_mode_cannot_launch')
    print(json.dumps(result,indent=2),flush=True)
    if not result['ok']:return 2
    command=args.command
    if command and command[0]=='--':command=command[1:]
    if command:return subprocess.call(command)
    return 0

if __name__=='__main__':sys.exit(main())
