#!/usr/bin/env python3
"""Real Navigator/BT + failstop chain; planner alone is synthetic, no driver."""
import hashlib
import json
import os
import time
import unittest
from pathlib import Path
import yaml
from rclpy.action import ActionClient, ActionServer
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses, NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

NAVIGATOR = Path('/home/rpp/rpp_ws/install/nav2_bt_navigator/lib/nav2_bt_navigator/bt_navigator')

class NavigatorFailstop(chain.FailstopChainROS):
    def setUp(self):
        if self._testMethodName == 'test_ordinary_failure_retries_once':
            self.progress_allowance = 8. if os.environ.get("NEUPAN_CHAIN_WORKER") == "actual-solver" else 2.
        super().setUp()
        self.planner_calls = []
        def compute(handle):
            self.planner_calls.append(time.monotonic())
            # Follow the exact terminal goal; synthetic clear global route.
            result = ComputePathToPose.Result() if hasattr(handle.request, 'goal') else ComputePathThroughPoses.Result()
            result.path = NavPath(); result.path.header.frame_id = 'map'
            target = handle.request.goal if hasattr(handle.request, 'goal') else handle.request.goals[-1]
            for x in [0., 2., target.pose.position.x]:
                pose = PoseStamped(); pose.header.frame_id = 'map'
                pose.pose.position.x = x; pose.pose.orientation.w = 1.
                result.path.poses.append(pose)
            handle.succeed(); return result
        self.planners = [ActionServer(self.node, ComputePathToPose, '/compute_path_to_pose', compute),
                         ActionServer(self.node, ComputePathThroughPoses, '/compute_path_through_poses', compute)]
        configdir = chain.WS/'src/r1_navigation/packages/navigation_neupan_bridge/config'
        params = {'bt_navigator': {'ros__parameters': dict(use_sim_time=False, global_frame='map',
            robot_base_frame='base_link', odom_topic=chain.PREFIX+'odom', bt_loop_duration=10,
            default_server_timeout=1000, wait_for_service_timeout=1000,
            default_nav_to_pose_bt_xml=str(configdir/'navigate_w_replanning_neupan.xml'),
            default_nav_through_poses_bt_xml=str(configdir/'navigate_through_poses_neupan.xml'),
            plugin_lib_names=['nav2_compute_path_to_pose_action_bt_node',
                'nav2_compute_path_through_poses_action_bt_node','nav2_follow_path_action_bt_node',
                'nav2_rate_controller_bt_node','nav2_pipeline_sequence_bt_node',
                'nav2_recovery_node_bt_node','neupan_wait_condition_bt_node'])}}
        # Production BT uses the default service name. Remap to the isolated
        # Controller instance; neither service arms or resets the runtime.
        paramsfile=self.report/'navigator-params.yaml';paramsfile.write_text(yaml.safe_dump(params))
        for name in ['navigate_w_replanning_neupan.xml','navigate_through_poses_neupan.xml']:
            source=configdir/name
            self.artifacts.append(dict(bt_xml=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
        self.start(NAVIGATOR,paramsfile,['/neupan_controller/can_retry:='+chain.PREFIX+'retry'])
        self.lifecycle('bt_navigator',1);self.lifecycle('bt_navigator',3)
        maps=(Path('/proc')/str(self.procs[-1].pid)/'maps').read_text()
        (self.report/'navigator-maps.txt').write_text(maps)
        self.assertTrue(any('neupan_failstop_ws' in line and 'libneupan_wait_condition_bt_node.so' in line
                            for line in maps.splitlines()), 'Navigator loaded wrong retry plugin')
        loaded=set()
        for line in maps.splitlines():
            path=Path(line.split()[-1])
            if path.is_file() and ('neupan_wait_condition' in path.name or 'bt_node' in path.name):
                loaded.add(path.resolve())
        for path in sorted(loaded):
            self.artifacts.append(dict(loaded_bt_library=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        (self.report/'binaries.json').write_text(json.dumps(self.artifacts,indent=2))

    def exercise_navigator(self, through):
        action_type=NavigateThroughPoses if through else NavigateToPose
        client=ActionClient(self.node,action_type,'/navigate_through_poses' if through else '/navigate_to_pose')
        self.until(client.server_is_ready)
        goal=action_type.Goal();pose=PoseStamped();pose.header.frame_id='map'
        pose.pose.position.x=4.;pose.pose.orientation.w=1.
        if through:goal.poses=[pose]
        else:goal.pose=pose
        handle=self.future(client.send_goal_async(goal));self.assertTrue(handle.accepted)
        result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),seconds=45)
        self.assertTrue(any(v>.04 for _,v,_ in self.outputs),'no initial output: '+str(self.report))
        marker=time.monotonic();self.fatal=True;self.publish('inject',{'fault':'worker_crash'})
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
        settled=time.monotonic();self.until(lambda:time.monotonic()-settled>.35)
        attempts={j['navigation_attempt_id'] for t,j in self.contexts if j['state']=='ACTIVE'}
        self.assertEqual(len(attempts),1,'FATAL must not produce BT second FollowPath attempt')
        self.assertTrue(any(t>marker and v==0. and w==0. for t,v,w in self.outputs))
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.2))
        # Same terminal target is a new top-level goal, still denied by latch.
        retry=self.future(client.send_goal_async(goal));self.assertTrue(retry.accepted)
        self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>marker+.2))
        (self.report/'navigator-result.json').write_text(json.dumps(dict(through=through,
            attempts_before_retry=sorted(attempts),planner_calls=len(self.planner_calls),
            fatal_to_terminal_sec=settled-marker,physical_motion=False,worker=self.worker_mode),indent=2))
        client.destroy()

    def test_ordinary_failure_retries_once(self):
        client=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.until(client.server_is_ready)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=4.;goal.pose.pose.orientation.w=1.
        handle=self.future(client.send_goal_async(goal));self.assertTrue(handle.accepted)
        result=self.future(handle.get_result_async(),seconds=30)
        self.assertEqual(result.status,GoalStatus.STATUS_ABORTED)
        attempts={j['navigation_attempt_id'] for t,j in self.contexts if j['state']=='ACTIVE'}
        self.assertEqual(len(attempts),2,'ordinary progress failure should retain exactly one retry')
        self.assertTrue(any(v>.04 for _,v,_ in self.outputs))
        (self.report/'navigator-result.json').write_text(json.dumps(dict(ordinary_failure=True,
            attempts=sorted(attempts),planner_calls=len(self.planner_calls),physical_motion=False,
            worker=self.worker_mode),indent=2))
        client.destroy()

    def test_navigate_to_pose_fatal(self):self.exercise_navigator(False)
    def test_navigate_through_poses_fatal(self):self.exercise_navigator(True)

def load_tests(loader, tests, pattern):
    return unittest.TestSuite(NavigatorFailstop(name) for name in
        ['test_navigate_to_pose_fatal','test_navigate_through_poses_fatal','test_ordinary_failure_retries_once'])

if __name__=='__main__':unittest.main()
