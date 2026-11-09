#!/usr/bin/env python3
"""Isolated real NeuPAN/Guard: a newly observed rear wall revokes old reverse motion."""
import json,math,os,time,unittest
from pathlib import Path
from unittest.mock import Mock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from rclpy.qos import qos_profile_sensor_data
import failstop_chain_ros as chain

chain.CONTROLLER=Path(os.environ['NEUPAN_TEST_CONTROLLER'])
chain.GUARD=Path(os.environ['NEUPAN_TEST_GUARD'])
chain.SMOOTHER=Path(os.environ['NEUPAN_TEST_SMOOTHER'])

class RearWallContinuation(chain.FailstopChainROS):
    worker_parameters_override=dict(auto_gear_transition_enabled=False,continuation_enabled=True,
                                    allow_reverse=True,independent_reference_direction=True)
    controller_parameters_override=dict(continuation_enabled=True,max_linear=.5,max_reverse=.5,allow_reverse=True)
    guard_parameters_override=dict(continuation_enabled=True,max_linear_mps=.5,max_reverse_mps=.5)
    smoother_parameters_override=dict(max_velocity=[.5,0.,.4],min_velocity=[-.5,0.,-.4])
    expected_plugin_root=os.environ.get('NEUPAN_TEST_ROUTE_PLUGIN_ROOT','/tmp/continuation-wiring-install')
    def setUp(self):
        self.wall=False;self.next_scene=0.;self.proofs=[]
        super().setUp()
        self.pubs['worker_scan']=Mock()
        self.scene_pub=self.node.create_publisher(LaserScan,chain.PREFIX+'worker_scan',qos_profile_sensor_data)
        self.proof_sub=self.node.create_subscription(String,'/neupan_trajectory_proof',
            lambda m:self.proofs.append((time.monotonic(),json.loads(m.data))),20)
    def tick(self):
        super().tick()
        if not hasattr(self,'scene_pub') or time.monotonic()<self.next_scene:return
        self.next_scene=time.monotonic()+.1
        scan=LaserScan();scan.header.stamp=self.node.get_clock().now().to_msg();scan.header.frame_id='base_link'
        scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/360
        scan.range_min=.05;scan.range_max=8.;scan.ranges=[7.]*721
        if self.wall:
            for i in range(721):
                a=-math.pi+i*math.pi/360
                if math.cos(a)<0:
                    r=-.43/math.cos(a)
                    if abs(r*math.sin(a))<.45:scan.ranges[i]=r
        self.scene_pub.publish(scan)
    def goal(self):
        goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
        goal.path.header.frame_id='map'
        for x in [0.,-2.,-4.]:
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.orientation.w=1.
            goal.path.poses.append(p)
        handle=self.future(self.action.send_goal_async(goal));self.assertTrue(handle.accepted)
        self.owned_attempts.add(bytes(handle.goal_id.uuid).hex());return handle
    def test_new_rear_wall_revokes_continuation(self):
        self.assertEqual(self.worker_mode,'actual-solver')
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v<-.04 for _,v,_ in self.outputs) or result.done(),55.)
        self.assertFalse(result.done(),'reverse trajectory did not start')
        self.assertTrue(any(v<-.04 for _,v,_ in self.outputs))
        self.until(lambda:any(p.get('valid') for _,p in self.proofs) or result.done(),10.)
        self.assertFalse(result.done());self.assertTrue(any(p.get('valid') for _,p in self.proofs))
        self.wall=True;marker=time.monotonic()
        self.until(lambda:any(t>marker and p.get('valid') is False for t,p in self.proofs) or result.done(),5.)
        rejected=[t for t,p in self.proofs if t>marker and p.get('valid') is False]
        self.assertTrue(rejected,'new rear wall did not produce a denial')
        self.until(lambda:any(t>rejected[0] and s.get('reason')=='planner_result_stale'
                              for t,s in self.statuses) or result.done(),3.)
        self.assertTrue(any(t>rejected[0] and s.get('reason')=='planner_result_stale' for t,s in self.statuses),
                        'final guard did not revoke the old reverse source')
        if not result.done():self.future(handle.cancel_goal_async());self.future(result)
        print('new rear wall caused trajectory denial and guard revocation; no driver was run')

if __name__=='__main__':unittest.main()
