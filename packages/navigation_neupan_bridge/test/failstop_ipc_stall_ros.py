#!/usr/bin/env python3
"""Stop only an owned live solver after motion; parent heartbeat stays alive."""
import json
import os
from pathlib import Path
import signal
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class IPCStall(chain.FailstopChainROS):
    def test_live_solver_stall(self):
        self.assertEqual(self.worker_mode,'actual-solver')
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        worker=next(p for p in self.procs if 'chain_worker_peer.py' in ' '.join(p.args))
        children=set()
        for task in (Path('/proc')/str(worker.pid)/'task').glob('*'):
            try:children.update(int(v) for v in (task/'children').read_text().split())
            except FileNotFoundError:pass
        solvers=[p for p in children if 'spawn_main' in (Path('/proc')/str(p)/'cmdline').read_text()]
        self.assertEqual(len(solvers),1);pid=solvers[0]
        marker=time.monotonic();os.kill(pid,signal.SIGSTOP)
        try:
            self.assertEqual(self.future(result,4).status,GoalStatus.STATUS_ABORTED)
            terminal=time.monotonic()
            # The independent source watchdog must act long before the 30s
            # IPC timeout, while the supervisor is healthy and generation fixed.
            self.assertLess(terminal-marker,3.)
            health=[j for t,j in self.healths if marker<t<terminal]
            self.assertGreater(len(health),5)
            self.assertTrue(all(j['state']=='RUNNING' and j['fault_id'] is None for j in health))
            self.assertIsNone(worker.poll())
            self.assertTrue((Path('/proc')/str(pid)).exists())
            tail=time.monotonic()+.4;self.until(lambda:time.monotonic()>tail)
            self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.8))
        finally:os.kill(pid,signal.SIGCONT)
        resumed=time.monotonic();self.until(lambda:time.monotonic()>resumed+.6)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.8))
        retry=self.goal();self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        (self.report/'ipc-stall-result.json').write_text(json.dumps(dict(solver_pid=pid,
            parent_pid=worker.pid,parent_health_running=True,worker_fatal_before_action_terminal=False,
            action='ABORTED',terminal_seconds=terminal-marker,zero_after_seconds=.8,
            resumed_solver_cannot_restore_motion=True,new_goal='ABORTED',physical_motion=False),indent=2))

def load_tests(loader,tests,pattern):return unittest.TestSuite([IPCStall('test_live_solver_stall')])
if __name__=='__main__':unittest.main()
