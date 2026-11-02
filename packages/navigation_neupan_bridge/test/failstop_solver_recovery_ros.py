#!/usr/bin/env python3
"""Classified-failure injection over actual solver/Controller/Guard, no driver."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain


class SolverRecovery(chain.FailstopChainROS):
    guard_parameters_override = dict(collision_check_enabled=False)

    def fail_solves(self, count):
        nonce=str(time.time_ns())
        self.publish('inject',dict(solver_failures=count,nonce=nonce))
        self.until(lambda:any(row.get('nonce')==nonce for _,row in self.inject_acks))

    def begin_motion(self):
        handle=self.goal();result=handle.get_result_async()
        marker=time.monotonic()
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        return handle,result

    def test_one_failure_recovers_without_latching(self):
        handle,result=self.begin_motion()
        start=time.monotonic();attempt=self.context['navigation_attempt_id']
        self.fail_solves(1)
        self.until(lambda:any(t>start and j.get('planner_state')=='retry' for t,j in self.healths))
        failed=next(j for t,j in self.healths if t>start and j.get('planner_state')=='retry')
        self.until(lambda:any(t>start and v==0 and w==0 for t,v,w in self.outputs))
        self.until(lambda:any(t>start and j['request_id']>failed['planner_failed_request_id'] for t,j in self.sources))
        resumed=time.monotonic()
        self.until(lambda:any(t>resumed and v>.04 for t,v,w in self.outputs) or result.done())
        self.assertFalse(result.done())
        self.assertEqual(attempt,self.context['navigation_attempt_id'])
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        self.assertFalse(any(j.get('fault_latched') for _,j in self.runtime_statuses))
        self.future(handle.cancel_goal_async())
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
        (self.report/'solver-recovery-result.json').write_text(json.dumps(dict(
            mode='single',same_attempt=attempt,recovered=True,physical_motion=False,
            failure_injection='classified failure after real IPC solve'),indent=2))

    def test_exhausted_attempt_new_goal_can_retry(self):
        handle,result=self.begin_motion()
        old=self.context['navigation_attempt_id'];start=time.monotonic()
        self.fail_solves(2)
        self.assertEqual(self.future(result,4).status,GoalStatus.STATUS_ABORTED)
        ended=time.monotonic()
        self.until(lambda:time.monotonic()>ended+.4)
        self.assertTrue(all(v==0 and w==0 for t,v,w in self.outputs if t>ended+.1))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        self.assertFalse(any(j.get('fault_latched') for _,j in self.runtime_statuses))
        new,new_result=self.begin_motion()
        self.assertNotEqual(old,self.context['navigation_attempt_id'])
        self.future(new.cancel_goal_async())
        self.assertEqual(self.future(new_result).status,GoalStatus.STATUS_CANCELED)
        (self.report/'solver-recovery-result.json').write_text(json.dumps(dict(
            mode='exhausted',failed_attempt=old,new_attempt=self.context['navigation_attempt_id'],
            terminal_seconds=ended-start,new_goal_recovered=True,physical_motion=False,
            failure_injection='classified failures after real IPC solve'),indent=2))


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(SolverRecovery(name) for name in
        ['test_one_failure_recovers_without_latching','test_exhausted_attempt_new_goal_can_retry'])
