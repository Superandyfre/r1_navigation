"""Real chain: diagnostic publication failure cannot latch motion authority."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from failstop_solver_recovery_ros import SolverRecovery

class DiagnosticIsolation(SolverRecovery):
    def test_diagnostic_failure_keeps_fresh_sources_and_new_goal(self):
        self.worker_statuses=[]
        self.subs.append(self.node.create_subscription(String,'/neupan_bridge_status',
            lambda m:self.worker_statuses.append((time.monotonic(),json.loads(m.data))),20))
        handle,result=self.begin_motion()
        marker=time.monotonic();nonce=str(time.time_ns())
        request_id=max(j['request_id'] for _,j in self.sources)
        self.publish('inject',dict(diagnostic_failures=8,nonce=nonce))
        self.until(lambda:any(j.get('nonce')==nonce for _,j in self.inject_acks))
        self.until(lambda:any(t>marker and j.get('code')=='diagnostic_publication_failed'
                             for t,j in self.worker_statuses))
        self.until(lambda:any(t>marker and j['request_id']>request_id+10 for t,j in self.sources))
        self.assertFalse(result.done())
        self.until(lambda:any(t>marker+.5 and v>.04 for t,v,w in self.outputs) or result.done())
        self.assertFalse(result.done())
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        self.assertFalse(any(j.get('fault_latched') for _,j in self.runtime_statuses))
        self.future(handle.cancel_goal_async())
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
        fresh,fresh_result=self.begin_motion()
        self.future(fresh.cancel_goal_async())
        self.assertEqual(self.future(fresh_result).status,GoalStatus.STATUS_CANCELED)
        (self.report/'diagnostic-isolation-result.json').write_text(json.dumps(dict(
            diagnostic_failures=8,continued_fresh_sources=True,new_goal_works=True,
            physical_motion=False),indent=2))

def load_tests(loader,tests,pattern):
    return unittest.TestSuite([DiagnosticIsolation('test_diagnostic_failure_keeps_fresh_sources_and_new_goal')])
