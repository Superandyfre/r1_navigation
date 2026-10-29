"""Actual Navigator/Controller route revisions in isolated domain 192, no driver."""
import copy
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer
from r1_neupan_runtime.srv import UpdatePath
from failstop_navigator_owner_ros import NavigatorOwner


class RouteUpdateNavigator(NavigatorOwner):
    def setUp(self):
        super().setUp()
        self.allow_bend = False
        self.planners[0].destroy()
        def compute(handle):
            self.planner_calls.append(time.monotonic())
            result = ComputePathToPose.Result()
            result.path.header.frame_id = 'map'
            result.path.header.stamp = self.node.get_clock().now().to_msg()
            target = handle.request.goal
            for x, y in [(0., 0.), (1., .25 if self.allow_bend else 0.),
                         (2., 0.), (target.pose.position.x, 0.)]:
                pose = PoseStamped(); pose.header = result.path.header
                pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.orientation.w = 1.
                result.path.poses.append(pose)
            handle.succeed()
            return result
        self.planners[0] = ActionServer(self.node, ComputePathToPose, '/compute_path_to_pose', compute)

    def test_replanning_keeps_uuid_and_updates_material_route(self):
        client, handle, result = self.send_top()
        attempt = self.moving_attempt(result)
        marker = time.monotonic()
        self.until(lambda: time.monotonic() >= marker+3.2 or result.done(), 8.)
        self.assertFalse(result.done())
        steady = [(v, w) for t, v, w in self.outputs if marker+.2 < t < time.monotonic()-.1]
        self.assertGreater(len(steady), 20)
        self.assertLess(sum(abs(v) < .005 and abs(w) < .005 for v, w in steady)/len(steady), .05)
        initial_path = self.context['path_version']
        self.allow_bend = True
        self.until(lambda: self.context['path_version'] > initial_path or result.done(), 8.)
        self.assertFalse(result.done())
        attempts = {j['navigation_attempt_id'] for _, j in self.contexts if j['state'] == 'ACTIVE'}
        self.assertEqual(attempts, {attempt})
        self.assertFalse(any(j['state'] == 'END' for _, j in self.contexts))
        self.assertFalse(any(j.get('fault_latched') for _, j in self.runtime_statuses))
        (self.report/'route-update-result.json').write_text(json.dumps(dict(
            attempt=attempt, planner_calls=len(self.planner_calls),
            zero_fraction=sum(abs(v)<.005 and abs(w)<.005 for v,w in steady)/len(steady),
            initial_path_version=initial_path, final_path_version=self.context['path_version'],
            worker=self.worker_mode, physical_motion=False), indent=2))
        self.cancel_top(handle, result)
        client.destroy()

    def test_update_rejects_wrong_execution_terminal_and_retired_goal(self):
        client, handle, result = self.send_top()
        attempt = self.moving_attempt(result)
        update = self.node.create_client(UpdatePath, '/update_follow_path')
        self.until(update.service_is_ready)
        path = Path(); path.header.frame_id = 'map'
        for x in (0., 2., 4.):
            pose = PoseStamped(); pose.header.frame_id = 'map'
            pose.pose.position.x = x; pose.pose.orientation.w = 1.; path.poses.append(pose)
        request = UpdatePath.Request(authority_boot_id=self.context['authority_boot_id'],
                                     navigation_attempt_id=attempt, path=path)
        for field, value in [('authority_boot_id', 'old-boot'), ('navigation_attempt_id', 'old-attempt')]:
            wrong = copy.deepcopy(request); setattr(wrong, field, value)
            self.assertFalse(self.future(update.call_async(wrong)).accepted)
        wrong = copy.deepcopy(request); wrong.path.poses[-1].pose.position.x = 5.
        self.assertFalse(self.future(update.call_async(wrong)).accepted)
        wrong = copy.deepcopy(request); wrong.path.poses[1].pose.position.x = float('nan')
        self.assertFalse(self.future(update.call_async(wrong)).accepted)
        self.assertTrue(self.future(update.call_async(request)).accepted)
        self.assertFalse(result.done())
        self.cancel_top(handle, result)
        self.assertFalse(self.future(update.call_async(request)).accepted)
        self.assertFalse(any(j.get('fault_latched') for _, j in self.runtime_statuses))
        client.destroy(); self.node.destroy_client(update)


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(RouteUpdateNavigator(name) for name in (
        'test_replanning_keeps_uuid_and_updates_material_route',
        'test_update_rejects_wrong_execution_terminal_and_retired_goal',
        'test_preemption_rejects_old_owner_replay'))
