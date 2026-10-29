#!/usr/bin/env python3
"""Cancel top goal while FollowPath acceptance is still hidden by ACK proxy."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import failstop_navigator_ros as nav
from failstop_navigator_ack_delay_ros import NavigatorAckDelay


class PendingHalt(NavigatorAckDelay):
    def start(self,exe,params,remaps):
        if exe==nav.NAVIGATOR and self._testMethodName.endswith('revoke_unavailable'):
            remaps=[*remaps,'/neupan_controller/revoke_uncertain_execution:=/test_failstop_ack/missing_revoke']
        super().start(exe,params,remaps)

    def exercise_halt(self,wait_accepted=False):
        client=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.until(client.server_is_ready)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=4.;goal.pose.pose.orientation.w=1.
        handle=self.future(client.send_goal_async(goal));self.assertTrue(handle.accepted)
        result=handle.get_result_async();eventfile=self.report/'ack-peer/events.json'
        def request_pending():
            if not eventfile.is_file():return False
            events=json.loads(eventfile.read_text())
            return bool(events and 'delayed_response' not in events[0] and
                (not wait_accepted or events[0].get('accepted') is True))
        self.until(request_pending,3.)
        self.assertFalse(result.done(),'top-level already failed before cancellation')
        marker=time.monotonic()
        self.assertTrue(self.future(handle.cancel_goal_async()).goals_canceling)
        self.assertEqual(self.future(result,4.).status,GoalStatus.STATUS_CANCELED)
        terminal=time.monotonic()
        self.until(lambda:time.monotonic()>terminal+3.,4.)
        events=json.loads(eventfile.read_text());first=events[0]
        self.assertTrue(first['accepted'])
        self.assertLess(marker,first['delayed_response'])
        self.assertGreater(first['delayed_response']-first['actual_response'],1.9)
        self.assertFalse(any(v!=0. or w!=0. for _,v,w in self.outputs))
        self.assertFalse(self.owners,'pending ACK never establishes accepted BT ownership')
        self.assertTrue(any(j.get('fault_latched') and j.get('fault_reason') in (
            'follow_path_execution_uncertain','owner_ack_timeout') for _,j in self.runtime_statuses))
        retry=self.future(client.send_goal_async(goal));self.assertTrue(retry.accepted)
        self.assertEqual(self.future(retry.get_result_async(),4.).status,GoalStatus.STATUS_ABORTED)
        (self.report/'pending-halt-result.json').write_text(json.dumps(dict(
            top_status='CANCELED',cancelled_before_ack_delivery=True,
            proxy_events=events,owner_count=0,all_outputs_zero=True,
            downstream_ack_observed_before_cancel=first['actual_response']<marker,
            cancel_requested_at=marker,waited_for_downstream_ack=wait_accepted,
            solver_not_expected=not any(j['state']=='ACTIVE' for _,j in self.contexts),
            next_goal='ABORTED',physical_motion=False),indent=2))
        client.destroy()

    def test_halt_pending_ack(self):self.exercise_halt()
    def test_halt_pending_revoke_unavailable(self):self.exercise_halt()
    def test_halt_after_accept_before_ack(self):self.exercise_halt(True)
    def test_halt_after_accept_revoke_unavailable(self):self.exercise_halt(True)


def load_tests(loader,tests,pattern):return unittest.TestSuite(PendingHalt(name) for name in (
    'test_halt_pending_ack','test_halt_pending_revoke_unavailable',
    'test_halt_after_accept_before_ack','test_halt_after_accept_revoke_unavailable'))


if __name__=='__main__':unittest.main()
