#!/usr/bin/env python3
"""Freeze the actual Navigator process while downstream components stay live."""
import json
import signal
import time
import unittest
from action_msgs.msg import GoalStatus
from failstop_navigator_owner_ros import NavigatorOwner


class NavigatorFreeze(NavigatorOwner):
    def test_navigator_process_freeze(self):
        client,handle,result=self.send_top()
        attempt=self.moving_attempt(result)
        navigator=self.procs[-1]
        self.assertTrue(str(navigator.args[0]).endswith('/bt_navigator'))
        marker=time.monotonic()
        navigator.send_signal(signal.SIGSTOP)
        try:
            self.until(lambda:any(t>marker and j.get('fault_reason')=='owner_lease_expired'
                and j.get('fault_latched') for t,j in self.runtime_statuses),3.)
            fault_at=next(t for t,j in self.runtime_statuses if t>marker and
                j.get('fault_reason')=='owner_lease_expired' and j.get('fault_latched'))
            self.until(lambda:any(t>marker and j['state']=='END' and
                j['navigation_attempt_id']==attempt for t,j in self.contexts),2.)
            self.until(lambda:time.monotonic()>fault_at+.8,2.)
            self.assert_owner_retired(attempt,marker)
            self.assertTrue(any(t>fault_at+.15 for t,_,_ in self.outputs))
            self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>fault_at+.15))
            self.assertTrue(any(t>fault_at+.4 and j['state']=='RUNNING' for t,j in self.healths))
            self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        finally:
            # Always resume our own test child so teardown cannot strand it.
            navigator.send_signal(signal.SIGCONT)
        self.assertEqual(self.future(result,4.).status,GoalStatus.STATUS_ABORTED)
        resumed=time.monotonic()
        newclient,newhandle,newresult=self.send_top(6.)
        self.assertEqual(self.future(newresult,4.).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:time.monotonic()>resumed+.5,2.)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>fault_at+.15))
        (self.report/'navigator-freeze-result.json').write_text(json.dumps(dict(
            attempt=attempt,freeze_at=marker,fault_at=fault_at,
            fault='owner_lease_expired',worker_healthy=True,downstream_ended=True,
            resumed_top='ABORTED',new_goal='ABORTED',physical_motion=False),indent=2))
        client.destroy();newclient.destroy()


def load_tests(loader,tests,pattern):
    return unittest.TestSuite([NavigatorFreeze('test_navigator_process_freeze')])


if __name__=='__main__':unittest.main()
