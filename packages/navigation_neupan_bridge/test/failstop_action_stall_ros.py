#!/usr/bin/env python3
"""Actual Action-thread stall with independent real plugin runtime timers."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from std_msgs.msg import Empty,String
import failstop_chain_ros as chain

class ActionStall(chain.FailstopChainROS):
    controller_plugin='navigation_neupan_bridge/ActionStallController'
    def test_action_thread_stall(self):
        events=[]
        sub=self.node.create_subscription(String,chain.PREFIX+'action_stall_ack',
            lambda msg:events.append((time.monotonic(),msg.data)),10)
        pub=self.node.create_publisher(Empty,chain.PREFIX+'action_stall',10)
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        self.until(lambda:pub.get_subscription_count()>0)
        pub.publish(Empty());self.until(lambda:any(v=='paused' for _,v in events))
        paused=next(t for t,v in events if v=='paused')
        self.until(lambda:time.monotonic()>paused+1.)
        self.assertFalse(any(v=='resumed' for _,v in events))
        self.assertFalse(result.done(),'Action compute thread must still be inside injected stall')
        runtime=[j for t,j in self.runtime_statuses if t>paused+.1 and
                 j.get('schema')=='r1_neupan_execution_status_v1']
        self.assertGreater(len(runtime),8,'background timer must continue during Action stall')
        self.assertTrue(any(j['fault_reason']=='action_loop_stale' for j in runtime))
        self.assertFalse(any(j['state']=='COMMIT' and t>paused+.65 for t,j in self.grants))
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>paused+.8))
        health=[j for t,j in self.healths if paused<t<paused+1.]
        self.assertGreater(len(health),5)
        self.assertTrue(all(j['fault_id'] is None for j in health))
        self.assertEqual(self.future(result,3).status,GoalStatus.STATUS_ABORTED)
        self.assertTrue(any(v=='resumed' for _,v in events))
        ended=time.monotonic();self.until(lambda:time.monotonic()>ended+.4)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>paused+.8))
        retry=self.goal();self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        (self.report/'action-stall-result.json').write_text(json.dumps(dict(
            events=events,background_statuses_during_stall=len(runtime),health_during_stall=len(health),
            fault='action_loop_stale',action='ABORTED',terminal_after_pause_seconds=ended-paused,
            zero_observed_from_seconds=.8,new_goal='ABORTED',physical_motion=False,
            adapter='test-only subclass; compute thread pause before real base compute'),indent=2))

def load_tests(loader,tests,pattern):return unittest.TestSuite([ActionStall('test_action_thread_stall')])
if __name__=='__main__':unittest.main()
