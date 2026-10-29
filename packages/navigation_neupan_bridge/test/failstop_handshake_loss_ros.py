#!/usr/bin/env python3
"""Drop real PREPARE/COMMIT on the isolated Controller -> Guard wire."""
import json
import time
import unittest
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import failstop_chain_ros as chain
from failstop_revoke_loss_ros import RevokeLoss

class HandshakeLoss(RevokeLoss):
    def setUp(self):
        self.drop_state='PREPARE' if self._testMethodName=='test_prepare_lost' else 'COMMIT'
        super().setUp()

    def arm_runtime(self):
        self.relay_pub=self.node.create_publisher(String,chain.PREFIX+'grant',20)
        def relay(message):
            payload=json.loads(message.data)
            drop=payload['state']==self.drop_state
            self.relay_events.append(dict(monotonic=time.monotonic(),payload=payload,dropped=drop))
            if not drop:self.relay_pub.publish(message)
        self.relay_sub=self.node.create_subscription(String,chain.PREFIX+'grant_out',relay,20)
        chain.FailstopChainROS.arm_runtime(self)

    def exercise_handshake(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(e['dropped'] for e in self.relay_events) or result.done(),50)
        self.assertFalse(result.done(),'ended before intended handshake loss')
        # PREPARE may precede first Source: DDS callbacks are independent.
        # Observe actual Source arrival instead of assuming callback order.
        self.until(lambda:bool(self.sources) or result.done(),50)
        self.assertFalse(result.done(),'ended before first actual Source')
        self.assertTrue(self.sources)
        marker=time.monotonic()
        self.until(lambda:time.monotonic()>marker+.8)
        if self.drop_state=='PREPARE':
            self.assertFalse(result.done())
            self.assertFalse(any(e['payload']['state']=='COMMIT' for e in self.relay_events))
            self.future(handle.cancel_goal_async())
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
            reason='revoke retires Attempt before any delivered PREPARE'
        else:
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
            self.assertTrue(any(j['authorization_reason']=='navigation_authority_stale'
                                for _,j in self.statuses))
            reason='navigation_authority_stale'
        self.until(lambda:any(j['state']=='REVOKE' for _,j in self.grants))
        self.assertFalse(any(j['state']==self.drop_state for _,j in self.grants))
        # First barrier: no motion at any point, even with real fresh Sources.
        self.assertFalse(any(v!=0. or w!=0. for _,v,w in self.outputs))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        captured=[e['payload'] for e in self.relay_events if e['payload']['state'] in ('PREPARE','COMMIT')]
        old_source=dict(self.sources[-1][1])
        raw=self.node.create_publisher(Twist,chain.PREFIX+'raw',20)
        smooth=self.node.create_publisher(Twist,chain.PREFIX+'smooth',20)
        command=Twist();command.linear.x=.15
        replay=time.monotonic()
        for _ in range(30):
            for payload in captured[-3:]:self.relay_pub.publish(String(data=json.dumps(payload)))
            self.publish('source',old_source);raw.publish(command);smooth.publish(command);self.tick()
        self.until(lambda:time.monotonic()>replay+.5)
        rows=[r for r in self.outputs if r[0]>replay]
        self.assertGreater(len(rows),15)
        self.assertTrue(all(v==0. and w==0. for _,v,w in rows))
        (self.report/'handshake-loss-result.json').write_text(json.dumps(dict(
            dropped_state=self.drop_state,dropped_messages=sum(e['dropped'] for e in self.relay_events),
            result='CANCELED' if self.drop_state=='PREPARE' else 'ABORTED',guard_reason=reason,
            zero_samples_after_replay=len(rows),all_outputs_zero=True,worker_fatal=False,
            replay_original_messages=True,fresh_nonzero_commands=True,physical_motion=False),indent=2))

    def test_prepare_lost(self):self.exercise_handshake()
    def test_commit_lost(self):self.exercise_handshake()

class AckLoss(chain.FailstopChainROS):
    def setUp(self):
        self.ack_events=[]
        self.wrong_context=self._testMethodName=='test_wrong_context_ack'
        super().setUp()

    def start(self,exe,params,remaps):
        if exe==chain.CONTROLLER:
            config=yaml.safe_load(params.read_text())
            config['controller_server']['ros__parameters']['FollowPath']['guard_status_topic']=chain.PREFIX+'status_relay'
            params=self.report/'controller-ack-params.yaml';params.write_text(yaml.safe_dump(config))
        super().start(exe,params,remaps)

    def arm_runtime(self):
        self.ack_pub=self.node.create_publisher(String,chain.PREFIX+'status_relay',20)
        def relay(message):
            payload=json.loads(message.data)
            prepared=payload['authorization_state']=='PREPARED'
            delivered=dict(payload)
            if prepared and self.wrong_context:
                delivered['navigation_attempt_id']='wrong-attempt'
            drop=prepared and not self.wrong_context
            self.ack_events.append(dict(monotonic=time.monotonic(),original=payload,
                delivered=None if drop else delivered))
            if not drop:self.ack_pub.publish(String(data=json.dumps(delivered)))
        self.ack_sub=self.node.create_subscription(String,chain.PREFIX+'status',relay,20)
        super().arm_runtime()

    def shutdown(self):
        (self.report/'ack-relay-events.json').write_text(json.dumps(self.ack_events,indent=2))
        super().shutdown()

    def exercise_ack(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(e['original']['authorization_state']=='PREPARED' for e in self.ack_events)
                   or result.done(),50)
        self.assertFalse(result.done())
        marker=time.monotonic()
        self.until(lambda:time.monotonic()>marker+.8)
        if self.wrong_context:
            self.assertFalse(result.done())
            self.future(handle.cancel_goal_async())
            expected=GoalStatus.STATUS_CANCELED
        else:
            expected=GoalStatus.STATUS_ABORTED
        self.assertEqual(self.future(result).status,expected)
        self.assertFalse(any(j['state']=='COMMIT' for _,j in self.grants))
        self.until(lambda:time.monotonic()>marker+1.2)
        self.assertFalse(any(v!=0. or w!=0. for _,v,w in self.outputs))
        self.assertTrue(self.sources)
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        (self.report/'ack-loss-result.json').write_text(json.dumps(dict(
            mode='wrong-context' if self.wrong_context else 'prepared-ack-dropped',
            action='CANCELED' if self.wrong_context else 'ABORTED',
            all_outputs_zero=True,commit_count=0,source_count=len(self.sources),
            changed_monotonic_or_sequence=False,worker_fatal=False,physical_motion=False),indent=2))

    def test_prepared_ack_lost(self):self.exercise_ack()
    def test_wrong_context_ack(self):self.exercise_ack()

def load_tests(loader,tests,pattern):
    return unittest.TestSuite([*(HandshakeLoss(n) for n in ['test_prepare_lost','test_commit_lost']),
        *(AckLoss(n) for n in ['test_prepared_ack_lost','test_wrong_context_ack'])])
if __name__=='__main__':unittest.main()
