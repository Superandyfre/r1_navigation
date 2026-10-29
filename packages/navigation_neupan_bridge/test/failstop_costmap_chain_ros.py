#!/usr/bin/env python3
"""Real NeuPAN full chain while actual Costmap2DROS remains noncurrent."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
import failstop_chain_ros as chain

class CostmapChain(chain.FailstopChainROS):
    noncurrent_test_layer=True
    def exercise_wait(self,mode):
        acknowledgments=[]
        sub=self.node.create_subscription(Bool,chain.PREFIX+'costmap_noncurrent_ack',
            lambda msg:acknowledgments.append((time.monotonic(),msg.data)),10)
        pub=self.node.create_publisher(Bool,chain.PREFIX+'costmap_noncurrent',10)
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        self.until(lambda:pub.get_subscription_count()>0)
        request=time.monotonic();pub.publish(Bool(data=True))
        self.until(lambda:any(t>request and v for t,v in acknowledgments))
        current_false=next(t for t,v in acknowledgments if t>request and v)
        self.until(lambda:time.monotonic()>current_false+.45)
        self.assertFalse(result.done(),'must inject before the 2s costmap wait deadline')
        # No compute output while stuck in actual server costmap wait.
        self.assertFalse(any(abs(v)>.001 or abs(w)>.001 for t,v,w in self.raw if t>current_false+.15))
        marker=time.monotonic()
        if mode=='fatal':self.fatal=True
        elif mode=='cancel':self.future(handle.cancel_goal_async())
        expected=GoalStatus.STATUS_CANCELED if mode=='cancel' else GoalStatus.STATUS_ABORTED
        terminal=self.future(result,3)
        ended=time.monotonic();self.assertEqual(terminal.status,expected)
        if mode in ('fatal','cancel'):self.assertLess(ended-marker,.7)
        else:
            self.assertGreater(ended-current_false,1.8)
            self.assertLess(ended-current_false,2.5)
        self.assertTrue(all(value for t,value in acknowledgments if current_false<t<ended))
        settle=time.monotonic()+.4;self.until(lambda:time.monotonic()>settle)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>current_false+.4))
        self.assertTrue(any(j['state']=='END' for t,j in self.contexts if t>marker))
        if mode=='fatal':
            retry=self.goal();self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        (self.report/'costmap-chain-result.json').write_text(json.dumps(dict(
            mode=mode,costmap_noncurrent_ack=current_false,injected_at=marker,
            action_status=terminal.status,terminal_after_injection_seconds=ended-marker,
            terminal_after_noncurrent_seconds=ended-current_false,wait_limit_seconds=2.,
            costmap_stayed_noncurrent=True,final_zero_from_noncurrent_seconds=.4,
            physical_motion=False,worker=self.worker_mode,
            layer='test-only current flag; actual Costmap2DROS and actual NeuPANController'),indent=2))

    def test_fatal_during_noncurrent(self):self.exercise_wait('fatal')
    def test_cancel_during_noncurrent(self):self.exercise_wait('cancel')
    def test_costmap_wait_deadline(self):self.exercise_wait('deadline')

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(CostmapChain(n) for n in ['test_fatal_during_noncurrent','test_cancel_during_noncurrent','test_costmap_wait_deadline'])
if __name__=='__main__':unittest.main()
