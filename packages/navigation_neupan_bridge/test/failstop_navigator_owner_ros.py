#!/usr/bin/env python3
"""Actual Navigator halt/preemption must retire the downstream owner identity."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from failstop_navigator_ros import NavigatorFailstop


class NavigatorOwner(NavigatorFailstop):
    def send_top(self,x=4.):
        client=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.until(client.server_is_ready)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=x;goal.pose.pose.orientation.w=1.
        handle=self.future(client.send_goal_async(goal));self.assertTrue(handle.accepted)
        return client,handle,handle.get_result_async()

    def moving_attempt(self,result,after=0.,old=None):
        self.until(lambda:(self.context and self.context['state']=='ACTIVE' and
            self.context['navigation_attempt_id']!=old and
            any(t>after and v>.04 for t,v,_ in self.outputs)) or result.done(),50.)
        self.assertFalse(result.done(),'top-level execution failed before intended injection')
        attempt=self.context['navigation_attempt_id']
        self.until(lambda:any(j['navigation_attempt_id']==attempt for _,j in self.owners),2.)
        self.until(lambda:any(t>after and j.get('navigation_attempt_id')==attempt and
            j.get('authorization_state')=='ARMED' for t,j in self.statuses),5.)
        armed_at=next(t for t,j in self.statuses if t>after and
            j.get('navigation_attempt_id')==attempt and j.get('authorization_state')=='ARMED')
        self.until(lambda:any(t>armed_at+.05 and v>.04 for t,v,_ in self.outputs) or result.done(),3.)
        self.assertFalse(result.done())
        return attempt

    def cancel_top(self,handle,result):
        response=self.future(handle.cancel_goal_async())
        self.assertTrue(response.goals_canceling)
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
        terminal=time.monotonic()
        self.until(lambda:time.monotonic()>terminal+.7)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>terminal+.15))
        return terminal

    def assert_owner_retired(self,attempt,since):
        self.assertFalse(any(t>since+.15 and j['navigation_attempt_id']==attempt for t,j in self.owners))

    def test_cancel_halts_owner_and_allows_fresh_goal(self):
        client,handle,result=self.send_top()
        old=self.moving_attempt(result)
        terminal=self.cancel_top(handle,result)
        self.assert_owner_retired(old,terminal)
        marker=time.monotonic()
        newclient,newhandle,newresult=self.send_top(6.)
        new=self.moving_attempt(newresult,marker,old)
        self.assertNotEqual(new,old)
        self.assertFalse(any(j.get('fault_latched') for _,j in self.runtime_statuses))
        self.cancel_top(newhandle,newresult)
        (self.report/'navigator-owner-result.json').write_text(json.dumps(dict(
            scenario='cancel_then_new_goal',old=old,new=new,first_status='CANCELED',
            old_owner_retired=True,new_goal_moved=True,physical_motion=False),indent=2))
        client.destroy();newclient.destroy()

    def test_preemption_rejects_old_owner_replay(self):
        client,handle,result=self.send_top()
        old=self.moving_attempt(result)
        old_owner=dict(next(j for _,j in reversed(self.owners) if j['navigation_attempt_id']==old))
        marker=time.monotonic()
        newclient,newhandle,newresult=self.send_top(6.)
        new=self.moving_attempt(newresult,marker,old)
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
        settled=time.monotonic()
        while time.monotonic()<settled+.7:
            self.owner_pub.publish(String(data=json.dumps(old_owner)));self.tick()
        self.assertFalse(newresult.done())
        self.assertEqual(self.context['navigation_attempt_id'],new)
        self.assertTrue(any(t>settled+.4 and v>.04 for t,v,_ in self.outputs))
        # Our replay is intentionally still observable; distinguish the actual
        # BT publisher by timestamp renewal, not just transport receipt time.
        self.assertFalse(any(t>settled+.15 and j['navigation_attempt_id']==old and
            j['monotonic']>old_owner['monotonic'] for t,j in self.owners))
        self.assertFalse(any(j.get('fault_latched') for _,j in self.runtime_statuses))
        self.cancel_top(newhandle,newresult)
        (self.report/'navigator-owner-result.json').write_text(json.dumps(dict(
            scenario='preempt_and_old_owner_replay',old=old,new=new,old_status='ABORTED',
            new_goal_moved=True,old_owner_cannot_affect_new=True,physical_motion=False),indent=2))
        client.destroy();newclient.destroy()


def load_tests(loader,tests,pattern):return unittest.TestSuite(NavigatorOwner(name) for name in (
    'test_cancel_halts_owner_and_allows_fresh_goal','test_preemption_rejects_old_owner_replay'))


if __name__=='__main__':unittest.main()
