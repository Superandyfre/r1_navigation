#!/usr/bin/env python3
"""Parent authority loss must stop actual CogNav owner renewal independently."""
import json
import time
import unittest
from failstop_cognav_ros import CogNavFailstop


class CogNavOwner(CogNavFailstop):
    def exercise_parent_loss(self, inputs=False):
        self.submit(1)
        self.until(lambda:self.bridge.counts['goals_sent']==1,8.)
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs),50.)
        # Isolate owner renewal's own checks from the separate cancel watchdog.
        # Actual Controller, Worker, Guard, robot inputs and ROS callbacks stay live.
        self.bridge._execution_watchdog=lambda:None
        marker=time.monotonic()
        if inputs:
            timers=[timer for timer in self.bridge.timers if timer.callback==self.bridge.write_motion]
            self.assertEqual(len(timers),1);timers[0].cancel()
        else:
            self.plan=None  # stop writing parent driver lease, keep ROS inputs fresh
        self.until(lambda:not self.bridge.lifecycle.busy(),3.)
        self.assertEqual(self.bridge.lifecycle.current['state'],'aborted')
        self.assertFalse(self.bridge.lifecycle.current['cancel_requested'])
        self.until(lambda:any(j.get('fault_reason')=='owner_lease_expired' for _,j in self.runtime_statuses))
        terminal=time.monotonic()
        self.until(lambda:time.monotonic()>terminal+.6)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>terminal+.15))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        self.submit(2)
        ack=self.bridge.root/'ack/000002.json';self.until(ack.is_file)
        self.assertFalse(json.loads(ack.read_text())['sent'])
        self.assertEqual(self.bridge.counts['goals_sent'],1)
        (self.report/'cognav-owner-result.json').write_text(json.dumps(dict(
            scenario='bridge_input_stale' if inputs else 'parent_driver_lease_lost',
            cancel_watchdog_disabled=True,action='ABORTED',fault='owner_lease_expired',
            terminal_after=terminal-marker,new_plan_refused=True,physical_motion=False),indent=2))

    def test_parent_driver_lease_loss(self):self.exercise_parent_loss()
    def test_bridge_input_freshness_loss(self):self.exercise_parent_loss(True)


def load_tests(loader,tests,pattern):return unittest.TestSuite(CogNavOwner(name) for name in (
    'test_parent_driver_lease_loss','test_bridge_input_freshness_loss'))


if __name__=='__main__':unittest.main()
