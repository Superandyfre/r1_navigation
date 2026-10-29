#!/usr/bin/env python3
"""Partial process restarts in isolated actual component chain; no driver."""
import json
from pathlib import Path
import signal
import subprocess
import time
import unittest
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState, ChangeState
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
import failstop_chain_ros as chain

class PartialRestart(chain.FailstopChainROS):
    def process(self,role):
        if role=='worker':return next(p for p in self.procs if 'chain_worker_peer.py' in ' '.join(p.args))
        names={'guard':chain.GUARD.name,'controller':chain.CONTROLLER.name,'smoother':chain.SMOOTHER.name}
        return next(p for p in self.procs if Path(p.args[0]).name==names[role])

    def restart(self,role):
        proc=self.process(role);args=list(proc.args)
        # Keep warmup/solver artifacts from the original Worker intact.
        args=[('n115_output_dir:='+str(self.report/'restarted_worker'))
              if a.startswith('n115_output_dir:=') else a for a in args]
        child_pids=set()
        for task in (Path('/proc')/str(proc.pid)/'task').glob('*'):
            try:child_pids.update(int(p) for p in (task/'children').read_text().split())
            except FileNotFoundError:pass
        proc.kill();proc.wait(timeout=3);self.procs.remove(proc)
        if role=='worker':
            for pid in child_pids:
                try:
                    import os
                    os.kill(pid,signal.SIGKILL)
                except ProcessLookupError:pass
        log=open(self.report/(role+'-restarted.log'),'w');self.logs.append(log)
        fresh=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT);self.procs.append(fresh)
        (self.report/(role+'-restart-process.json')).write_text(json.dumps(dict(old_pid=proc.pid,
            new_pid=fresh.pid,children_of_old=sorted(child_pids),args=args),indent=2))
        return fresh

    def activate_restarted(self,name):
        # service_is_ready may still refer to the killed server. Confirm the
        # replacement's actual lifecycle state before sending a transition.
        state=self.node.create_client(GetState,'/'+name+'/get_state')
        change=self.node.create_client(ChangeState,'/'+name+'/change_state')
        deadline=time.monotonic()+5.
        observed=False
        while time.monotonic()<deadline and not observed:
            request=state.call_async(GetState.Request())
            response_deadline=time.monotonic()+.25
            while not request.done() and time.monotonic()<response_deadline:self.tick()
            if request.done():observed=request.result().current_state.id==1
            else:state.remove_pending_request(request)
        self.assertTrue(observed,'replacement must be unconfigured before activation')
        settle=time.monotonic()+.25
        self.until(lambda:time.monotonic()>settle)
        for transition in [1,3]:
            request=ChangeState.Request();request.transition.id=transition
            self.assertTrue(self.future(change.call_async(request)).success)
        self.node.destroy_client(state);self.node.destroy_client(change)

    def exercise_restart(self,role):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for t,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done());self.assertTrue(any(v>.04 for t,v,w in self.outputs))
        old_guard=self.statuses[-1][1]['guard_boot_id']
        old_worker=self.healths[-1][1]['boot_id']
        old_controller=next(j['controller_boot_id'] for t,j in reversed(self.runtime_statuses)
                            if j.get('schema')=='r1_neupan_execution_status_v1')
        marker=time.monotonic();fresh=self.restart(role)
        if role=='controller':
            self.activate_restarted('controller_server')
            self.until(lambda:any(j.get('schema')=='r1_neupan_execution_status_v1' and
                j['controller_boot_id']!=old_controller for t,j in self.runtime_statuses if t>marker))
            # A crashed Action server cannot send a terminal result. Do not
            # manufacture ABORTED for the lost goal; only the new goal is checked.
            self.action.destroy();self.action=ActionClient(self.node,FollowPath,'/follow_path')
            self.until(self.action.server_is_ready)
        elif role=='guard':
            self.until(lambda:any(j['guard_boot_id']!=old_guard for t,j in self.statuses if t>marker))
        elif role=='worker':
            self.until(lambda:any(j['boot_id']!=old_worker for t,j in self.healths if t>marker))
        else:
            # Loss of the smoother is a command-stream outage, not a Worker
            # FATAL. Require Guard zero during loss; a fresh smoother may
            # serve the still-valid Action after explicit lifecycle activation.
            outage=time.monotonic()+.4
            self.until(lambda:time.monotonic()>outage)
            self.assertTrue(any(t>marker+.25 and j['reason']=='command_stale'
                                for t,j in self.statuses))
            self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.25))
            self.assertFalse(result.done())
            self.activate_restarted('velocity_smoother')
            resumed=time.monotonic()
            self.until(lambda:any(t>resumed and v>.04 for t,v,w in self.outputs) or result.done())
            self.assertFalse(result.done())
            self.future(handle.cancel_goal_async())
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
            (self.report/'restart-result.json').write_text(json.dumps(dict(role=role,
                outage_zero=True,worker_fatal=False,healthy_action_resumed=True,
                action_status='CANCELED',physical_motion=False,worker=self.worker_mode),indent=2))
            return
        if role!='controller':
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:any(t>marker+.8 and v==0. and w==0. for t,v,w in self.outputs))
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.8))
        retry=self.goal()
        self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        arm=self.node.create_client(Trigger,chain.PREFIX+'arm');self.until(arm.service_is_ready)
        self.assertFalse(self.future(arm.call_async(Trigger.Request())).success)
        settled=time.monotonic();self.until(lambda:time.monotonic()-settled>.35)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.8))
        maps=Path('/proc')/str(fresh.pid)/'maps'
        (self.report/(role+'-restarted-maps.txt')).write_text(maps.read_text())
        (self.report/'restart-result.json').write_text(json.dumps(dict(role=role,old_guard=old_guard,
            old_worker=old_worker,old_controller=old_controller,old_action_terminal_proven=role!='controller',
            old_action_status=None if role=='controller' else 'ABORTED',new_goal='ABORTED',arm_denied=True,
            physical_motion=False,worker=self.worker_mode),indent=2))

    def test_guard_restart(self):self.exercise_restart('guard')
    def test_worker_parent_restart(self):self.exercise_restart('worker')
    def test_controller_restart(self):self.exercise_restart('controller')
    def test_smoother_restart(self):self.exercise_restart('smoother')

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(PartialRestart(name) for name in ['test_guard_restart','test_worker_parent_restart',
        'test_controller_restart','test_smoother_restart'])
if __name__=='__main__':unittest.main()
