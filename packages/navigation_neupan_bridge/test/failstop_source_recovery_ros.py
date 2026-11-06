#!/usr/bin/env python3
"""Actual solver Source pause/recovery, preserving original protocol times."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class SourceRecovery(chain.FailstopChainROS):
    def source_hold(self,hold):
        nonce=str(time.time_ns())
        self.publish('inject',dict(hold_source=hold,nonce=nonce))
        self.until(lambda:any(j.get('nonce')==nonce for _,j in self.inject_acks))

    def test_short_source_hold_recovers(self):
        self.assertEqual(self.worker_mode,'actual-solver')
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        attempt=self.context['navigation_attempt_id']
        grant=next(j['grant_id'] for _,j in reversed(self.grants) if j['state']=='COMMIT')
        rounds=[]
        for _ in range(2):
            self.source_hold(True)
            start=time.monotonic();last=dict(self.sources[-1][1])
            self.until(lambda:any(t>start and j['authorization_state']=='SOURCE_STALE_HOLD'
                                 for t,j in self.statuses),2.)
            held=time.monotonic()
            # Observe actual hold output before permitting fresh real results.
            self.until(lambda:any(t>held and v==0. and w==0. for t,v,w in self.outputs),.3)
            self.assertFalse(result.done())
            self.source_hold(False)
            release=time.monotonic()
            self.until(lambda:any(t>release and v>.04 for t,v,w in self.outputs) or result.done(),2.)
            self.assertFalse(result.done())
            fresh=self.sources[-1][1]
            self.assertEqual(fresh['navigation_attempt_id'],attempt)
            self.assertEqual(self.context['navigation_attempt_id'],attempt)
            self.assertTrue(all(j['grant_id']==grant for t,j in self.grants if t>start and j['state']=='COMMIT'))
            self.assertGreater(fresh['request_id'],last['request_id'])
            self.assertFalse(any(j['authorization_state']=='FAULT_LATCHED' for t,j in self.statuses if t>start))
            self.assertFalse(any(j.get('fault_latched') for t,j in self.runtime_statuses if t>start))
            rounds.append(dict(hold_start=start,stale_hold_seen=held,release=release,
                resumed=time.monotonic(),old_request=last['request_id'],new_request=fresh['request_id']))
        self.position=4.
        self.assertEqual(self.future(result,10).status,GoalStatus.STATUS_SUCCEEDED)
        (self.report/'source-recovery-result.json').write_text(json.dumps(dict(
            rounds=rounds,same_attempt=attempt,same_grant=grant,action='SUCCEEDED',
            healthy_worker=True,physical_motion=False),indent=2))

    def test_expired_source_hold_aborts_only_current_attempt(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        old_attempt=self.context["navigation_attempt_id"]
        self.source_hold(True)
        start=time.monotonic()
        self.assertEqual(self.future(result,8).status,GoalStatus.STATUS_ABORTED)
        ended=time.monotonic()
        self.until(lambda:time.monotonic()>ended+.5)
        self.assertTrue(all(v==0. and w==0. for t,v,w in self.outputs if t>ended+.1))
        self.assertTrue(any(j.get("authorization_reason")=="planner_result_recovery_timeout"
                            for t,j in self.statuses if t>start))
        self.assertFalse(any(j.get("fault_latched") for t,j in self.runtime_statuses if t>start))
        self.source_hold(False)
        retry=self.goal();next_result=retry.get_result_async()
        marker=time.monotonic()
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or next_result.done(),50)
        self.assertFalse(next_result.done())
        self.assertNotEqual(self.context["navigation_attempt_id"],old_attempt)
        self.assertTrue(any(t>marker and v>.04 for t,v,w in self.outputs))
        self.future(retry.cancel_goal_async())
        (self.report/"source-recovery-result.json").write_text(json.dumps(dict(
            mode="expired-hold",action="ABORTED",new_goal="resumed",
            terminal_after_pause=ended-start,root_reason="planner_result_recovery_timeout",
            healthy_worker=True,physical_motion=False),indent=2))

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(SourceRecovery(n) for n in
        ['test_short_source_hold_recovers','test_expired_source_hold_aborts_only_current_attempt'])
if __name__=='__main__':unittest.main()
