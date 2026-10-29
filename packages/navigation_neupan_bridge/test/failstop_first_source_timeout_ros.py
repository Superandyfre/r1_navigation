#!/usr/bin/env python3
"""Full unchanged 40-second first Source budget, actual solver, no driver."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class FirstSourceTimeout(chain.FailstopChainROS):
    def test_full_initial_source_deadline(self):
        self.assertEqual(self.worker_mode,'actual-solver')
        nonce='initial-source-'+str(time.time_ns())
        request=dict(hold_source=True,nonce=nonce)
        self.publish('inject',request)
        self.until(lambda:any(j.get('nonce')==nonce for _,j in self.inject_acks))
        sent=time.monotonic();handle=self.goal();result=handle.get_result_async()
        self.until(lambda:self.context is not None and self.context['state']=='ACTIVE')
        started=self.context['monotonic']
        self.until(lambda:any(j['state']=='RUNNING' for _,j in self.healths),40)
        # Repeated ACTIVE context/health/scan do not renew the first-source
        # deadline. Keep normal external stimuli alive for the full budget.
        self.until(lambda:time.monotonic()>started+39.,42)
        self.assertFalse(result.done(),'must not spend recovery budget before first Source')
        self.assertFalse(self.sources)
        self.assertFalse(any(v!=0. or w!=0. for _,v,w in self.outputs))
        self.assertFalse(any(j['state']=='COMMIT' for _,j in self.grants))
        self.assertEqual(self.future(result,4).status,GoalStatus.STATUS_ABORTED)
        ended=time.monotonic()
        self.assertGreaterEqual(ended-started,40.)
        self.assertLess(ended-started,41.)
        self.until(lambda:any(j.get('fault_reason')=='initial_source_timeout' for _,j in self.runtime_statuses))
        active_health=[j for t,j in self.healths if started+5<t<ended]
        self.assertGreater(len(active_health),250)
        self.assertTrue(all(j['state']=='RUNNING' and j['fault_id'] is None for j in active_health))
        nonce='resume-source-'+str(time.time_ns())
        self.publish('inject',dict(hold_source=False,nonce=nonce))
        self.until(lambda:any(j.get('nonce')==nonce for _,j in self.inject_acks))
        settle=time.monotonic()+.5;self.until(lambda:time.monotonic()>settle)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>ended))
        retry=self.goal();self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        (self.report/'first-source-timeout-result.json').write_text(json.dumps(dict(
            action='ABORTED',reason='initial_source_timeout',budget_seconds=40.,
            terminal_after_context_seconds=ended-started,goal_send_monotonic=sent,
            first_context_monotonic=started,parent_health_running_samples=len(active_health),
            sources_before_terminal=0,commit_before_terminal=False,continuous_final_zero=True,
            new_goal='ABORTED',physical_motion=False,worker=self.worker_mode),indent=2))

def load_tests(loader,tests,pattern):return unittest.TestSuite([FirstSourceTimeout('test_full_initial_source_deadline')])
if __name__=='__main__':unittest.main()
