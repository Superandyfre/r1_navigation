#!/usr/bin/env python3
"""Drop actual Controller REVOKE messages to real Guard; no robot driver."""
import json
from pathlib import Path
import time
import unittest
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import failstop_chain_ros as chain

class RevokeLoss(chain.FailstopChainROS):
    def setUp(self):
        self.relay_events=[];self.drop_revoke=True
        if self._testMethodName=='test_ordinary_failure_revoke_lost':self.progress_allowance=8.
        super().setUp()

    def start(self,exe,params,remaps):
        if exe==chain.CONTROLLER:
            config=yaml.safe_load(params.read_text())
            config['controller_server']['ros__parameters']['FollowPath']['motion_grant_topic']=chain.PREFIX+'grant_out'
            params=self.report/'controller-relay-params.yaml';params.write_text(yaml.safe_dump(config))
        super().start(exe,params,remaps)

    def arm_runtime(self):
        self.relay_pub=self.node.create_publisher(String,chain.PREFIX+'grant',20)
        def relay(message):
            payload=json.loads(message.data)
            drop=self.drop_revoke and payload['state']=='REVOKE'
            self.relay_events.append(dict(monotonic=time.monotonic(),payload=payload,dropped=drop))
            if not drop:self.relay_pub.publish(message)
        self.relay_sub=self.node.create_subscription(String,chain.PREFIX+'grant_out',relay,20)
        super().arm_runtime()

    def shutdown(self):
        (self.report/'grant-relay-events.json').write_text(json.dumps(self.relay_events,indent=2))
        super().shutdown()

    def exercise_loss(self,mode):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        old_source=dict(self.sources[-1][1])
        old_commit=next(dict(e['payload']) for e in reversed(self.relay_events) if e['payload']['state']=='COMMIT')
        marker=time.monotonic()
        if mode=='cancel':self.future(handle.cancel_goal_async())
        elif mode=='success':self.position=4.
        elif mode=='fatal':self.fatal=True
        expected={'cancel':GoalStatus.STATUS_CANCELED,'success':GoalStatus.STATUS_SUCCEEDED,
                  'ordinary_failure':GoalStatus.STATUS_ABORTED,'fatal':GoalStatus.STATUS_ABORTED}[mode]
        self.assertEqual(self.future(result,12).status,expected)
        marker=time.monotonic()
        self.until(lambda:any(e['dropped'] for e in self.relay_events))
        self.assertFalse(any(j['state']=='REVOKE' for t,j in self.grants))
        self.until(lambda:time.monotonic()>marker+.75)
        reason='worker_fault_latched' if mode=='fatal' else 'navigation_authority_stale'
        self.until(lambda:any(j['authorization_state']=='FAULT_LATCHED' and
                             j['authorization_reason']==reason
                             for t,j in self.statuses if t>marker))
        if mode=='fatal':
            self.assertTrue(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        settled=time.monotonic()
        raw=self.node.create_publisher(Twist,chain.PREFIX+'raw',20)
        smooth=self.node.create_publisher(Twist,chain.PREFIX+'smooth',20)
        # Captured complete valid old data, with original stamps/identity. Fresh
        # command traffic must not make the already expired authority valid.
        command=Twist();command.linear.x=.15
        for _ in range(25):
            self.relay_pub.publish(String(data=json.dumps(old_commit)))
            self.publish('source',old_source)
            raw.publish(command);smooth.publish(command);self.tick()
        self.until(lambda:time.monotonic()>settled+.4)
        rows=[r for r in self.outputs if r[0]>marker+.75]
        self.assertGreater(len(rows),15)
        self.assertTrue(all(v==0. and w==0. for _,v,w in rows))
        self.position=0.
        retry=self.goal();self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        (self.report/'revoke-loss-result.json').write_text(json.dumps(dict(mode=mode,
            original_action={'cancel':'CANCELED','success':'SUCCEEDED','ordinary_failure':'ABORTED','fatal':'ABORTED'}[mode],
            dropped_revokes=sum(e['dropped'] for e in self.relay_events),
            guard_reason=reason,authority_timeout_seconds=.5,
            zero_assertion_after_terminal_seconds=.75,zero_samples=len(rows),
            replayed_old_grant_source_with_fresh_commands=True,new_goal='ABORTED',
            physical_motion=False,worker=self.worker_mode),indent=2))

    def test_cancel_revoke_lost(self):self.exercise_loss('cancel')
    def test_success_revoke_lost(self):self.exercise_loss('success')
    def test_ordinary_failure_revoke_lost(self):self.exercise_loss('ordinary_failure')
    def test_fatal_revoke_lost(self):self.exercise_loss('fatal')

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(RevokeLoss(name) for name in ['test_cancel_revoke_lost','test_success_revoke_lost','test_ordinary_failure_revoke_lost','test_fatal_revoke_lost'])
if __name__=='__main__':unittest.main()
