"""Actual Navigator must not replenish a parent task's retry budget."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from failstop_navigator_ros import NavigatorFailstop


class SolverBTRecovery(NavigatorFailstop):
    guard_parameters_override=dict(collision_check_enabled=False)

    def test_parent_task_stops_after_one_replan_and_new_goal_works(self):
        client=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.until(client.server_is_ready)
        goal=NavigateToPose.Goal();goal.pose=PoseStamped();goal.pose.header.frame_id='map'
        goal.pose.header.stamp=self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x=4.;goal.pose.pose.orientation.w=1.
        handle=self.future(client.send_goal_async(goal));result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        first=self.context['navigation_attempt_id'];start=time.monotonic()
        nonce=str(time.time_ns())
        self.publish('inject',dict(solver_failures=4,nonce=nonce))
        self.until(lambda:any(j.get('nonce')==nonce for _,j in self.inject_acks))
        self.assertEqual(self.future(result,8).status,GoalStatus.STATUS_ABORTED)
        attempts={j['navigation_attempt_id'] for t,j in self.contexts if j['state']=='ACTIVE'}
        self.assertEqual(len(attempts),2)
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        goal.pose.header.stamp=self.node.get_clock().now().to_msg()
        marker=time.monotonic()
        fresh=self.future(client.send_goal_async(goal));fresh_result=fresh.get_result_async()
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or fresh_result.done(),10)
        self.assertFalse(fresh_result.done())
        self.future(fresh.cancel_goal_async());self.future(fresh_result)
        client.destroy()
        (self.report/'solver-bt-recovery-result.json').write_text(json.dumps(dict(
            attempts=sorted(attempts),initial_attempt=first,terminal_seconds=marker-start,
            new_goal_recovered=True,physical_motion=False),indent=2))


def load_tests(loader,tests,pattern):
    return unittest.TestSuite([SolverBTRecovery('test_parent_task_stops_after_one_replan_and_new_goal_works')])
