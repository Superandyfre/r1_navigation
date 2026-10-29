#!/usr/bin/env python3
"""Actual client owner lease stops while Worker and Action loop remain live."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class OwnerLoss(chain.FailstopChainROS):
    def test_no_owner_confirmation_never_moves(self):
        self.owner_enabled=False
        handle=self.goal();result=handle.get_result_async()
        self.assertEqual(self.future(result,3).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:any(j.get('fault_reason')=='owner_ack_timeout' for _,j in self.runtime_statuses))
        self.assertFalse(any(v!=0. or w!=0. for _,v,w in self.outputs))
        self.assertFalse(any(j['state']=='COMMIT' for _,j in self.grants))
        (self.report/'owner-loss-result.json').write_text(json.dumps(dict(
            reason='owner_ack_timeout',action='ABORTED',all_outputs_zero=True,
            commit_count=0,physical_motion=False),indent=2))

    def test_owner_lease_expiry(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        self.owner_enabled=False;marker=time.monotonic()
        self.assertEqual(self.future(result,3).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:any(j.get('fault_reason')=='owner_lease_expired' for _,j in self.runtime_statuses))
        terminal=time.monotonic()
        self.until(lambda:time.monotonic()>terminal+.6)
        self.owner_enabled=True
        retry=self.goal();self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>terminal+.15))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        (self.report/'owner-loss-result.json').write_text(json.dumps(dict(
            reason='owner_lease_expired',action='ABORTED',fault_observed_after=terminal-marker,
            new_goal='ABORTED',worker_fatal=False,physical_motion=False),indent=2))

def load_tests(loader,tests,pattern):return unittest.TestSuite(OwnerLoss(n) for n in
    ['test_no_owner_confirmation_never_moves','test_owner_lease_expiry'])
if __name__=='__main__':unittest.main()
