#!/usr/bin/env python3
"""Delay only test Worker's robot TF consumer; no driver or global TF change."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class StartupTF(chain.FailstopChainROS):
    def set_tf_hold(self,hold):
        marker=time.monotonic()
        self.publish('inject',{'hold_robot_tf':hold})
        self.until(lambda:any(t>marker and j.get('hold_robot_tf') is hold for t,j in self.inject_acks))

    def test_delayed_initial_tf(self):
        self.assertEqual(self.worker_mode,'actual-solver')
        self.set_tf_hold(True)
        handle=self.goal();result=handle.get_result_async()
        marker=time.monotonic()
        self.until(lambda:time.monotonic()>marker+1.)
        self.assertFalse(result.done())
        self.assertFalse(self.sources)
        self.assertFalse(any(v!=0 or w!=0 for _,v,w in self.outputs))
        self.set_tf_hold(False)
        released=time.monotonic()
        deadline=released+12.
        self.until(lambda:any(t>released and v>.04 for t,v,w in self.outputs) or
                   result.done() or time.monotonic()>deadline,13.)
        moving=any(t>released and v>.04 for t,v,w in self.outputs)
        (self.report/'startup-tf-result.json').write_text(json.dumps(dict(
            hold_seconds=1.,post_release_observation=time.monotonic()-released,
            reached_nonzero=moving,action_ended=result.done(),source_count=len(self.sources),
            physical_motion=False),indent=2))
        self.assertTrue(moving,'TF consumer released but scans did not start solver; '+str(self.report))
        self.assertFalse(result.done())
        self.position=4.
        self.assertEqual(self.future(result,10).status,GoalStatus.STATUS_SUCCEEDED)

def load_tests(loader,tests,pattern):
    return unittest.TestSuite([StartupTF('test_delayed_initial_tf')])
if __name__=='__main__':unittest.main()
