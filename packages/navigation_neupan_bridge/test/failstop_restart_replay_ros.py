#!/usr/bin/env python3
"""Captured old protocol messages around a complete controlled restart."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
import failstop_chain_ros as chain
from failstop_full_restart_ros import FullRestart


class RestartReplay(FullRestart):
    def setUp(self):
        super().setUp()
        self.replay_pubs = {key:self.node.create_publisher(String,chain.PREFIX+key,20)
                            for key in ['grant','context']}
        self.replay_events = []

    def replay(self, key, messages):
        for message in messages:
            self.replay_pubs[key].publish(String(data=json.dumps(message)))
            self.replay_events.append(dict(topic=key,payload=message,monotonic=time.monotonic()))
            self.tick()

    def before_restart_arm(self):
        # These are byte-equivalent JSON payloads captured before shutdown;
        # never refresh their monotonic stamps or substitute the new boot.
        old_boot = self.contexts[0][1]['controller_boot_id']
        self.old_contexts = [dict(j) for _,j in self.contexts if j['controller_boot_id']==old_boot]
        self.old_grants = [dict(j) for _,j in self.grants if j['controller_boot_id']==old_boot]
        self.assertTrue(any(j['state']=='COMMIT' for j in self.old_grants))
        self.assertTrue(any(j['state']=='REVOKE' for j in self.old_grants))
        self.assertTrue(any(j['state']=='ACTIVE' for j in self.old_contexts))
        self.assertTrue(any(j['state']=='END' for j in self.old_contexts))
        marker=time.monotonic()
        for _ in range(3):
            self.replay('grant',list(reversed(self.old_grants)))
            self.replay('context',list(reversed(self.old_contexts)))
        self.until(lambda:time.monotonic()>marker+.4)
        rows=[r for r in self.outputs if r[0]>marker]
        self.assertGreater(len(rows),10)
        self.assertTrue(all(v==0. and w==0. for _,v,w in rows))

    def after_restart_commit(self, result):
        marker=time.monotonic()
        # Old Guard boot is rejected even while the new chain is executable.
        self.replay('grant',self.old_grants)
        self.until(lambda:time.monotonic()>marker+.35)
        self.assertFalse(result.done(),'captured grants must not alter the new Guard epoch')
        self.assertTrue(any(t>marker and v>.04 for t,v,w in self.outputs))
        abort = self._testMethodName=='test_active_old_context'
        if abort:
            marker=time.monotonic()
            self.replay('context',self.old_contexts)
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
            self.until(lambda:any(t>marker and j.get('fault_code')=='execution_protocol_fault'
                                 for t,j in self.healths))
            # A foreign authority is an immutable fault after a new context is
            # bound. Safe denial is expected; it must never adopt the old task.
            self.assertFalse(any(t>marker and j.get('navigation_attempt_id')==
                self.old_contexts[0]['navigation_attempt_id'] for t,j in self.sources))
        (self.report/'replay-events.json').write_text(json.dumps(self.replay_events,indent=2))
        (self.report/'replay-result.json').write_text(json.dumps(dict(
            old_grants_before_arm_zero=True,old_context_before_arm_zero=True,
            old_grants_after_commit_ignored=True,old_context_after_commit_tested=abort,
            old_context_after_commit_outcome='ABORTED/execution_protocol_fault' if abort else None,
            physical_motion=False,worker=self.worker_mode),indent=2))
        return abort

    def test_prearm_old_protocol(self):self.exercise_full_restart()
    def test_active_old_context(self):self.exercise_full_restart()


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(RestartReplay(name) for name in
                             ['test_prearm_old_protocol','test_active_old_context'])

if __name__=='__main__':unittest.main()
