#!/usr/bin/env python3
"""Distinguish parent health loss from slow lazy solver startup; no driver."""
import json
import os
from pathlib import Path
import signal
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain


class StartupStall(chain.FailstopChainROS):
    def setUp(self):
        super().setUp()
        self.assertEqual(self.worker_mode,'actual-solver')

    def worker(self):
        return next(p for p in self.procs if 'chain_worker_peer.py' in ' '.join(p.args))

    def children(self):
        result=set()
        for task in (Path('/proc')/str(self.worker().pid)/'task').glob('*'):
            try:result.update(int(v) for v in (task/'children').read_text().split())
            except FileNotFoundError:pass
        # multiprocessing resource_tracker is not the solver.
        return [p for p in result if 'spawn_main' in (Path('/proc')/str(p)/'cmdline').read_text()]

    def test_parent_health_loss_during_startup(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:bool(self.children()))
        self.assertFalse(self.sources,'must inject before first source')
        pid=self.worker().pid;marker=time.monotonic()
        os.kill(pid,signal.SIGSTOP)
        try:
            self.assertEqual(self.future(result,3).status,GoalStatus.STATUS_ABORTED)
            elapsed=time.monotonic()-marker
            self.assertLess(elapsed,1.2)
            self.until(lambda:any(t>marker and j.get('fault_reason')=='worker_health_stale'
                                 for t,j in self.runtime_statuses))
        finally:os.kill(pid,signal.SIGCONT)
        end=time.monotonic()+.5;self.until(lambda:time.monotonic()>end)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker))
        (self.report/'startup-stall-result.json').write_text(json.dumps(dict(
            mode='parent-stopped',pid=pid,action='ABORTED',reason='worker_health_stale',
            terminal_seconds=elapsed,zero=True,physical_motion=False),indent=2))

    def test_solver_startup_wait_keeps_health(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:bool(self.children()))
        self.assertFalse(self.sources,'must inject before first source')
        pid=self.children()[0];marker=time.monotonic()
        os.kill(pid,signal.SIGSTOP)
        try:
            self.until(lambda:time.monotonic()>marker+1.)
            self.assertFalse(result.done(),'healthy parent must permit bounded lazy startup')
            health=[(t,j) for t,j in self.healths if t>marker]
            self.assertGreaterEqual(len(health),6)
            self.assertTrue(all(j['state']=='STARTING' for t,j in health))
            gaps=[b[0]-a[0] for a,b in zip(health,health[1:])]
            self.assertLess(max(gaps),.5)
            self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker))
        finally:os.kill(pid,signal.SIGCONT)
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        self.position=4.
        self.assertEqual(self.future(result,10).status,GoalStatus.STATUS_SUCCEEDED)
        (self.report/'startup-stall-result.json').write_text(json.dumps(dict(
            mode='solver-stopped-parent-alive',pid=pid,held_seconds=1.,health_samples=len(health),
            max_health_receipt_gap=max(gaps),waiting_zero=True,action='SUCCEEDED',physical_motion=False),indent=2))


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(StartupStall(n) for n in ['test_parent_health_loss_during_startup','test_solver_startup_wait_keeps_health'])
if __name__=='__main__':unittest.main()
