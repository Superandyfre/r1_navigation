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


def nested_values(value, key):
    if isinstance(value,dict):
        if key in value:yield value[key]
        for child in value.values():yield from nested_values(child,key)
    elif isinstance(value,list):
        for child in value:yield from nested_values(child,key)


def verify_solver_provenance(report, bundle):
    """Check actual parent/child summaries, including restarted Workers."""
    roots=[];errors=[]
    for summary in report.glob('**/summary.json'):
        data=json.loads(summary.read_text())
        for value in nested_values(data,'neupan_source_root'):
            roots.append(value)
            if Path(value).resolve()!=bundle/'third_party/NeuPAN':
                errors.append('wrong_solver:'+str(summary)+':'+value)
        for value in nested_values(data,'integration_source'):
            if not Path(value).resolve().is_relative_to(bundle/'install'):
                errors.append('wrong_worker:'+str(summary)+':'+value)
    if not roots:
        # A successful pending-ACK halt may prevent execution before the lazy
        # solver is started. Require its explicit result and independent trace;
        # a failed setup/test cannot obtain this scoped no-solver exception.
        result_path=report/'pending-halt-result.json'
        allowed=False
        if result_path.is_file():
            outcome=json.loads(result_path.read_text())
            trace=json.loads((report/'trace.json').read_text())
            allowed=(outcome.get('solver_not_expected') is True and
                outcome.get('top_status')=='CANCELED' and outcome.get('next_goal')=='ABORTED' and
                not trace['source'] and not any(j['state']=='ACTIVE' for _,j in trace['context']) and
                not any(j['state']=='COMMIT' for _,j in trace['grant']) and
                bool(trace['output']) and all(v==0. and w==0. for _,v,w in trace['output']))
        if not allowed:errors.append('missing_solver_provenance:'+str(report))
    return errors


class AcceptanceResult(unittest.TextTestResult):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.reports=[]

    def stopTest(self,test):
        if hasattr(test,'report'):self.reports.append(test.report)
        super().stopTest(test)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',required=True,type=Path)
    parser.add_argument('--bundle',type=Path,help='Verify and run the relocated bundle instead of candidate/install')
    parser.add_argument('--suite',choices=['chain','navigator','restart','startup-tf','revoke-loss','handshake-loss','source-recovery','owner-loss','owner-protocol','navigator-owner','navigator-freeze','navigator-tick-loss','navigator-configure-churn','pending-halt','ack-delay','effective-parameters','all'],default='all')
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='192'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST')=='1'
    assert os.environ.get('NEUPAN_CHAIN_WORKER')=='actual-solver'
    candidate=args.candidate.resolve(strict=True);install=candidate/'install'
    inventory=json.loads((candidate/'candidate-inventory.json').read_text())
    bundle=args.bundle.resolve(strict=True) if args.bundle else None
    if bundle:
        manifest=json.loads((bundle/'releases/a-plus/manifest.json').read_text())
        assert manifest['build_source_sha']==inventory['source_sha']
        assert Path(manifest['inventory_root']).resolve()==bundle
        assert manifest['inventory'] and manifest['build_testing'] is False
        for row in manifest['inventory']:
            path=(bundle/row['path']).resolve(strict=True)
            assert path.is_relative_to(bundle),path
            assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256'],path
        install=bundle/'install'
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
    if bundle:
        base.worker_environment=dict(os.environ)
        base.worker_environment['PYTHONPATH']=str(bundle/'third_party/NeuPAN')+os.pathsep+os.environ.get('PYTHONPATH','')
    import r1_neupan_integration.n1_16_node as worker
    assert Path(worker.__file__).resolve().is_relative_to(install),worker.__file__
    print(json.dumps(dict(candidate=str(candidate),build_source_sha=inventory['source_sha'],
        bundle=str(bundle) if bundle else None,
        worker_import=worker.__file__,inventory_verified_files=len(inventory['files']),suite=args.suite)),flush=True)
    modules=dict(chain='failstop_chain_ros',navigator='failstop_navigator_ros',
                 restart='failstop_full_restart_ros',
                 **{'startup-tf':'failstop_startup_tf_ros','revoke-loss':'failstop_revoke_loss_ros',
                    'handshake-loss':'failstop_handshake_loss_ros',
                    'source-recovery':'failstop_source_recovery_ros',
                    'owner-loss':'failstop_owner_loss_ros','owner-protocol':'failstop_owner_protocol_ros',
                    'navigator-owner':'failstop_navigator_owner_ros','pending-halt':'failstop_navigator_halt_pending_ros',
                    'navigator-freeze':'failstop_navigator_freeze_ros',
                    'ack-delay':'failstop_navigator_ack_delay_ros'})
    # This extra fixture needs a manually compiled, test-only decorator. Keep
    # the BUILD_TESTING=OFF candidate's standard all43 independent of adapters.
    if args.suite=='navigator-tick-loss':
        modules['navigator-tick-loss']='failstop_navigator_tick_loss_ros'
    if args.suite=='navigator-configure-churn':
        modules['navigator-configure-churn']='failstop_navigator_configure_churn_ros'
    if args.suite=='effective-parameters':
        modules['effective-parameters']='failstop_effective_parameters_ros'
    # User excludes CogNav from further work; do not import its fixtures in all.
    selected=list(modules) if args.suite=='all' else [args.suite]
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(importlib.import_module(modules[name])) for name in selected)
    result=unittest.TextTestRunner(verbosity=2,resultclass=AcceptanceResult).run(suite)
    provenance_errors=[]
    if bundle and args.suite=='navigator-configure-churn':
        # Explicit no-goal diagnostic, excluded from all43. It cannot establish
        # solver provenance because no lazy solver should have started.
        print(json.dumps(dict(diagnostic_scope='configure-only; no solver provenance claimed',
                              scenes_checked=len(result.reports))),flush=True)
    elif bundle:
        for report in result.reports:
            provenance_errors.extend(verify_solver_provenance(report,bundle))
        print(json.dumps(dict(bundle_provenance_errors=provenance_errors,
                              scenes_checked=len(result.reports))),flush=True)
    return 0 if result.wasSuccessful() and not provenance_errors else 1

if __name__=='__main__':sys.exit(main())
