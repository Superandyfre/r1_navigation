#!/usr/bin/env python3
"""Scoped old/new binary combinations, maps verified; no driver."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

OLD_CONTROLLER=Path('/home/rpp/rpp_ws/install/nav2_controller/lib/nav2_controller/controller_server')
OLD_BRIDGE=Path('/home/rpp/r1_navigation/install/navigation_neupan_bridge')

class MixedVersion(chain.FailstopChainROS):
    def setUp(self):
        self.legacy_worker=self._testMethodName=='test_old_python_new_server_plugin_guard'
        if self.legacy_worker:
            self.worker_helper=Path(__file__).with_name('legacy_worker_peer.py')
            self.worker_environment=os.environ.copy()
            root='/home/rpp/rpp_ws/src/application/neupan_migration/r1_neupan_integration'
            self.worker_source_root=Path(root)/'r1_neupan_integration'
            self.worker_environment['PYTHONPATH']=root+':'+self.worker_environment.get('PYTHONPATH','')
        self.old_plugin=self._testMethodName=='test_new_server_old_plugin_new_guard'
        if self.old_plugin:self.expected_plugin_root=str((OLD_BRIDGE/'lib/libnavigation_neupan_bridge.so').resolve().parent)
        super().setUp()
        proc=next(p for p in self.procs if Path(p.args[0]).name=='controller_server')
        maps=(Path('/proc')/str(proc.pid)/'maps').read_text()
        oldcore='/home/rpp/rpp_ws/install/nav2_controller/lib/libcontroller_server_core.so'
        # symlink installs map the build target, so compare canonical paths.
        expected=str((chain.WS/'install/nav2_controller/lib/libcontroller_server_core.so').resolve()) if self.old_plugin or self.legacy_worker else str(Path(oldcore).resolve())
        self.assertIn(expected,maps,'must prove loaded ControllerServer core identity')
        (self.report/'mixed-controller-maps.txt').write_text(maps)
        (self.report/'mixed-core-hash.json').write_text(json.dumps(dict(path=expected,
            sha256=hashlib.sha256(Path(expected).read_bytes()).hexdigest()),indent=2))

    def start(self,exe,params,remaps):
        if exe!=chain.CONTROLLER or self.legacy_worker:return super().start(exe,params,remaps)
        executable=exe if self.old_plugin else OLD_CONTROLLER
        env=os.environ.copy()
        if self.old_plugin:
            env['AMENT_PREFIX_PATH']=str(OLD_BRIDGE)+':'+env.get('AMENT_PREFIX_PATH','')
            env['LD_LIBRARY_PATH']=str(OLD_BRIDGE/'lib')+':'+env.get('LD_LIBRARY_PATH','')
        else:env['LD_LIBRARY_PATH']=str(OLD_CONTROLLER.parents[1])+':'+env.get('LD_LIBRARY_PATH','')
        self.artifacts.append(dict(path=str(executable),resolved=str(executable.resolve()),
            sha256=hashlib.sha256(executable.read_bytes()).hexdigest()))
        log=open(self.report/'controller_server.log','w');self.logs.append(log)
        args=[str(executable),'--ros-args','--params-file',str(params)]
        for remap in remaps:args+=['-r',remap]
        self.procs.append(subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT))

    def arm_runtime(self):
        if not self.old_plugin:return super().arm_runtime()
        # Do not invent a successful arm for the legacy plugin. It has no A+
        # service and the new Guard remains disarmed throughout this scene.
        (self.report/'preflight-arm.json').write_text(json.dumps(dict(skipped=True,
            reason='legacy plugin has no A+ arm service; not deployment compatible')))

    def test_new_server_old_plugin_new_guard(self):
        handle=self.goal();result=handle.get_result_async()
        start=time.monotonic();self.until(lambda:time.monotonic()-start>1.)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs))
        self.assertFalse(any(j['state']=='COMMIT' for t,j in self.grants))
        self.assertTrue(all(j['authorization_state']=='DISARMED' for t,j in self.statuses))
        self.assertFalse(any(j['state']=='ACTIVE' for t,j in self.contexts))
        if not result.done():self.future(handle.cancel_goal_async())
        terminal=self.future(result)
        self.assertIn(terminal.status,(GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED))
        (self.report/'mixed-result.json').write_text(json.dumps(dict(old_server=False,new_plugin=False,
            new_guard=True,result_status=terminal.status,any_final_nonzero=False,physical_motion=False,
            a_plus_compatible=False,automatic_failure_semantics_proven=False),indent=2))

    def test_old_python_new_server_plugin_guard(self):
        handle=self.goal()
        self.assertEqual(self.future(handle.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        settled=time.monotonic();self.until(lambda:time.monotonic()-settled>.35)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs))
        self.assertFalse(any(j['state']=='COMMIT' for t,j in self.grants))
        import_file=self.report/'worker/legacy-import.json'
        self.until(import_file.is_file)
        imported=json.loads(import_file.read_text())
        self.assertFalse(imported['has_health_publisher'])
        self.assertFalse(self.healths)
        (self.report/'mixed-result.json').write_text(json.dumps(dict(old_python=imported,
            new_server=True,new_plugin=True,new_guard=True,result='ABORTED',any_final_nonzero=False,
            physical_motion=False),indent=2))

    def test_old_server_new_plugin_new_guard(self):
        handle=self.goal()
        self.assertEqual(self.future(handle.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        settled=time.monotonic();self.until(lambda:time.monotonic()-settled>.35)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs))
        self.assertFalse(any(j['state']=='COMMIT' for t,j in self.grants))
        self.assertFalse(any(j['state']=='ACTIVE' for t,j in self.contexts))
        (self.report/'mixed-result.json').write_text(json.dumps(dict(old_server=True,new_plugin=True,
            new_guard=True,result='ABORTED',any_final_nonzero=False,physical_motion=False),indent=2))

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(MixedVersion(name) for name in ['test_old_server_new_plugin_new_guard',
        'test_new_server_old_plugin_new_guard','test_old_python_new_server_plugin_guard'])
if __name__=='__main__':unittest.main()
