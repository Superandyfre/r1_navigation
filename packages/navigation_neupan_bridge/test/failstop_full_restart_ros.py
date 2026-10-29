#!/usr/bin/env python3
"""Full component stop/rebuild with explicit arm; isolated synthetic sensors only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
import failstop_chain_ros as chain
from failstop_restart_ros import PartialRestart


class FullRestart(PartialRestart):
    def setUp(self):
        # Set ledger before the initial Worker process is created. Its directory
        # is independent of the harness report, which is allocated by setUp.
        import tempfile
        self.ledger_dir = Path(tempfile.mkdtemp(prefix='neupan-restart-ledger-'))
        self.ledger = self.ledger_dir/'context.json'
        self.worker_environment = dict(os.environ)
        self.worker_environment.pop('N116_CTX_LEDGER', None)
        mode = self._testMethodName.removeprefix('test_')
        if mode != 'disabled':
            self.worker_environment['N116_CTX_LEDGER'] = str(self.ledger)
        if mode == 'stale':
            self.ledger.write_text(json.dumps(dict(path_version=900001, goal_version=800001)))
        elif mode == 'corrupt':
            self.ledger.write_text('{broken ledger\n')
        self.ledger_before = self.ledger.read_bytes() if self.ledger.exists() else None
        super().setUp()
        self.assertTrue(self.real_node, 'requires an actual N116Node')

    def before_restart_arm(self):
        pass

    def after_restart_commit(self, result):
        return False

    def exercise_full_restart(self):
        handle = self.goal(); result = handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done(), 'initial chain never became executable')
        old_context = dict(self.context)
        old_source = dict(self.sources[-1][1])
        old_worker = self.healths[-1][1]['boot_id']
        old_guard = self.statuses[-1][1]['guard_boot_id']
        self.fatal = True
        self.assertEqual(self.future(result).status, GoalStatus.STATUS_ABORTED)
        self.until(lambda:any(j.get('state')=='FAULT_LATCHED' for _,j in self.healths))
        # An arm request on the old faulted chain cannot reset its FATAL.
        from std_srvs.srv import Trigger
        arm = self.node.create_client(Trigger, chain.PREFIX+'arm')
        self.until(arm.service_is_ready)
        self.assertFalse(self.future(arm.call_async(Trigger.Request())).success)
        self.node.destroy_client(arm)
        self.fatal = False
        specs = [(list(proc.args), 'chain_worker_peer.py' in ' '.join(proc.args)) for proc in self.procs]
        old_pids = [proc.pid for proc in self.procs]
        for proc in reversed(self.procs):
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait(timeout=3)
        self.procs.clear()
        self.action.destroy()
        marker = time.monotonic()
        for index, (args, worker) in enumerate(specs):
            args = [('n115_output_dir:='+str(self.report/'new-worker'))
                    if arg.startswith('n115_output_dir:=') else arg for arg in args]
            log = open(self.report/('full-restart-'+str(index)+'.log'), 'w')
            self.logs.append(log)
            self.procs.append(subprocess.Popen(args, env=self.worker_environment if worker else None,
                                              stdout=log, stderr=subprocess.STDOUT))
        self.activate_restarted('velocity_smoother')
        self.activate_restarted('controller_server')
        self.action = ActionClient(self.node, FollowPath, '/follow_path')
        self.until(self.action.server_is_ready)
        self.until(lambda:any(t>marker and j['guard_boot_id']!=old_guard for t,j in self.statuses))
        self.until(lambda:any(t>marker and j['boot_id']!=old_worker for t,j in self.healths))
        # A captured old Source cannot independently restore any permission.
        for _ in range(5):
            self.publish('source', old_source); self.tick()
        settle = time.monotonic()+.5
        self.until(lambda:time.monotonic()>settle)
        prearm = [row for row in self.outputs if row[0]>marker+.25]
        self.assertGreater(len(prearm), 10)
        self.assertTrue(all(v==0. and w==0. for _,v,w in prearm))
        self.before_restart_arm()
        # arm is a separate explicit operator action; a new Action is still
        # required. Do not send a rejected goal first: that may itself latch.
        self.arm_runtime()
        armed_at = time.monotonic()
        self.until(lambda:time.monotonic()>armed_at+.3)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>armed_at))
        fresh = self.goal(); fresh_result = fresh.get_result_async()
        self.until(lambda:any(t>armed_at and v>.04 for t,v,w in self.outputs) or fresh_result.done(),50)
        self.assertFalse(fresh_result.done(), 'new chain did not recover after explicit arm')
        new_context = dict(self.context)
        self.assertNotEqual(new_context['authority_boot_id'], old_context['authority_boot_id'])
        self.assertNotEqual(new_context['controller_boot_id'], old_context['controller_boot_id'])
        self.assertNotEqual(new_context['navigation_attempt_id'], old_context['navigation_attempt_id'])
        self.assertEqual(new_context['navigation_attempt_id'], bytes(fresh.goal_id.uuid).hex())
        self.assertLess(new_context['path_version'], 900001)
        self.assertTrue(any(t>armed_at and j.get('navigation_attempt_id')==new_context['navigation_attempt_id']
                            for t,j in self.sources))
        # Synthetic pose reaches the target: verify real FollowPath success,
        # rather than claiming recovery from a single nonzero sample.
        replay_aborted = self.after_restart_commit(fresh_result)
        if not replay_aborted:
            self.position = 4.
        expected = GoalStatus.STATUS_ABORTED if replay_aborted else GoalStatus.STATUS_SUCCEEDED
        self.assertEqual(self.future(fresh_result,10).status, expected)
        terminal = time.monotonic()
        self.until(lambda:time.monotonic()>terminal+.5)
        zeros = [r for r in self.outputs if r[0]>terminal+.1]
        self.assertGreater(len(zeros), 10)
        self.assertTrue(all(v==0. and w==0. for _,v,w in zeros))
        # Authoritative context mode neither restores nor overwrites the legacy
        # diagnostic ledger, including deliberately corrupt/stale contents.
        self.assertEqual(self.ledger.read_bytes() if self.ledger.exists() else None, self.ledger_before)
        (self.report/'full-restart-result.json').write_text(json.dumps(dict(
            ledger_mode=self._testMethodName, ledger_path=str(self.ledger), ledger_unchanged=True,
            old_pids=old_pids, new_pids=[p.pid for p in self.procs], old_context=old_context,
            new_context=new_context, old_action='ABORTED',
            new_action='ABORTED' if replay_aborted else 'SUCCEEDED',
            old_arm_denied=True, prearm_zero_samples=len(prearm), armed_without_goal_zero=True,
            old_source_replayed_before_arm=True, stale_grant_context_replay_tested=False,
            physical_motion=False, worker=self.worker_mode), indent=2))

    def test_disabled(self): self.exercise_full_restart()
    def test_enabled(self): self.exercise_full_restart()
    def test_stale(self): self.exercise_full_restart()
    def test_corrupt(self): self.exercise_full_restart()


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(FullRestart(name) for name in
                             ['test_disabled','test_enabled','test_stale','test_corrupt'])

if __name__ == '__main__': unittest.main()
