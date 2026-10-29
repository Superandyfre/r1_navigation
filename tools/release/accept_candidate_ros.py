#!/usr/bin/env python3
"""Run existing no-driver acceptance against a pinned candidate install.

Source candidate/install/setup.bash first. This test runner never approves a
release and refuses any install file changed since candidate inventory creation.
"""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import unittest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',required=True,type=Path)
    parser.add_argument('--suite',choices=['chain','navigator','cognav','restart','startup-tf','revoke-loss','handshake-loss','source-recovery','owner-loss','ack-delay','all'],default='all')
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='192'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST')=='1'
    assert os.environ.get('NEUPAN_CHAIN_WORKER')=='actual-solver'
    candidate=args.candidate.resolve(strict=True);install=candidate/'install'
    inventory=json.loads((candidate/'candidate-inventory.json').read_text())
    for row in inventory['files']:
        path=install/row['path']
        assert path.resolve().is_relative_to(install),path
        assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256'],path
    assert inventory['build_testing_off'] and all(inventory['build_testing_off'].values())
    assert not list(install.rglob('*same_uuid*'))
    assert not list(install.rglob('*action_stall*'))
    assert not list(install.rglob('*controlled_current*'))
    repo=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(repo/'packages/navigation_neupan_bridge/test'))
    import failstop_chain_ros as chain
    chain.CONTROLLER=install/'nav2_controller/lib/nav2_controller/controller_server'
    chain.GUARD=install/'r1_collision_guard/lib/r1_collision_guard/r1_collision_guard_node'
    base=chain.FailstopChainROS
    base.expected_plugin_root=str(install/'navigation_neupan_bridge/lib')
    base.expected_bt_root=str(install/'navigation_neupan_bridge/lib')
    base.worker_source_root=install/'r1_neupan_integration/lib/python3.10/site-packages/r1_neupan_integration'
    base.candidate_configdir=install/'navigation_neupan_bridge/share/navigation_neupan_bridge/config'
    base.candidate_checkpoint=install/'r1_neupan_integration/share/r1_neupan_integration/config/models/dune_model_5000.pth'
    import r1_neupan_integration.n1_16_node as worker
    assert Path(worker.__file__).resolve().is_relative_to(install),worker.__file__
    print(json.dumps(dict(candidate=str(candidate),build_source_sha=inventory['source_sha'],
        worker_import=worker.__file__,inventory_verified_files=len(inventory['files']),suite=args.suite)),flush=True)
    modules=dict(chain='failstop_chain_ros',navigator='failstop_navigator_ros',
                 cognav='failstop_cognav_ros',restart='failstop_full_restart_ros',
                 **{'startup-tf':'failstop_startup_tf_ros','revoke-loss':'failstop_revoke_loss_ros',
                    'handshake-loss':'failstop_handshake_loss_ros',
                    'source-recovery':'failstop_source_recovery_ros',
                    'owner-loss':'failstop_owner_loss_ros','ack-delay':'failstop_navigator_ack_delay_ros'})
    selected=list(modules) if args.suite=='all' else [args.suite]
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(importlib.import_module(modules[name])) for name in selected)
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1

if __name__=='__main__':sys.exit(main())
