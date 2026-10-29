#!/usr/bin/env python3
"""No downstream motion after top-level ACK timeout; no driver."""
import json
from pathlib import Path
import subprocess
import sys
import time
import unittest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import failstop_navigator_ros as nav

class NavigatorAckDelay(nav.NavigatorFailstop):
    def start(self,exe,params,remaps):
        if exe==nav.NAVIGATOR:
            helper=Path(__file__).with_name('chain_goal_ack_peer.py')
            log=open(self.report/'goal-ack-peer.log','w');self.logs.append(log)
            self.procs.append(subprocess.Popen([sys.executable,str(helper),str(self.report/'ack-peer')],stdout=log,stderr=subprocess.STDOUT))
            remaps=[*remaps,'/follow_path/_action/send_goal:=/test_failstop_ack/follow_path/_action/send_goal']
        super().start(exe,params,remaps)

    def test_ack_timeout_downstream_observation(self):
        client=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.until(client.server_is_ready)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=4.;goal.pose.pose.orientation.w=1.
        handle=self.future(client.send_goal_async(goal));self.assertTrue(handle.accepted)
        result=self.future(handle.get_result_async(),10)
        terminal=time.monotonic()
        self.assertEqual(result.status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:time.monotonic()>terminal+6.,7.)
        events=json.loads((self.report/'ack-peer/events.json').read_text())
        self.assertTrue(events and events[0]['accepted'])
        self.assertGreater(events[0]['delayed_response']-events[0]['actual_response'],1.9)
        self.assertTrue(any(j['navigation_attempt_id']==events[0]['uuid'] for _,j in self.contexts))
        later=[row for row in self.outputs if row[0]>terminal+.1 and (row[1]!=0. or row[2]!=0.)]
        record=dict(top_status='ABORTED',terminal=terminal,observation_seconds=6.,
            later_nonzero_samples=len(later),proxy_events=events,
            physical_motion=False,diagnostic_exposure_test=False)
        (self.report/'ack-delay-result.json').write_text(json.dumps(record,indent=2))
        self.assertEqual(len(later),0,'downstream output after top-level failure')
        self.assertTrue(any(j.get('fault_reason')=='follow_path_execution_uncertain'
                            and j.get('fault_latched') for _,j in self.runtime_statuses))
        retry=self.future(client.send_goal_async(goal));self.assertTrue(retry.accepted)
        self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        client.destroy()

def load_tests(loader,tests,pattern):
    return unittest.TestSuite([NavigatorAckDelay('test_ack_timeout_downstream_observation')])
if __name__=='__main__':unittest.main()
