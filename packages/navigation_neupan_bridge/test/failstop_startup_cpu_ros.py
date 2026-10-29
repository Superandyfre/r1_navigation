#!/usr/bin/env python3
"""Bound only owned Worker CPU affinity; compare actual heartbeat send/receive."""
import json
import os
from pathlib import Path
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class StartupCPU(chain.FailstopChainROS):
    def exercise_cpu(self,count):
        self.assertEqual(self.worker_mode,'actual-solver')
        worker=next(p for p in self.procs if 'chain_worker_peer.py' in ' '.join(p.args))
        original=sorted(os.sched_getaffinity(worker.pid))
        self.assertGreaterEqual(len(original),count)
        os.sched_setaffinity(worker.pid,set(original[:count]))
        # New spawn solver inherits only this process's restricted affinity.
        marker=time.monotonic();handle=self.goal();result=handle.get_result_async()
        try:
            self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or result.done(),50)
            health=[(t,j) for t,j in self.healths if t>marker]
            (self.report/'startup-cpu-result.json').write_text(json.dumps(dict(
                worker_pid=worker.pid,worker_cpus=original[:count],original_cpus=original,
                reached_nonzero=any(t>marker and v>.04 for t,v,w in self.outputs),
                ended_before_nonzero=result.done(),received_health=len(health),physical_motion=False),indent=2))
            self.assertFalse(result.done(),'startup ended before nonzero; retain send/receive trace')
            self.position=4.
            self.assertEqual(self.future(result,10).status,GoalStatus.STATUS_SUCCEEDED)
        finally:
            if worker.poll() is None:os.sched_setaffinity(worker.pid,set(original))

    def test_worker_one_cpu(self):self.exercise_cpu(1)
    def test_worker_two_cpus(self):self.exercise_cpu(2)

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(StartupCPU(name) for name in ['test_worker_one_cpu','test_worker_two_cpus'])
if __name__=='__main__':unittest.main()
