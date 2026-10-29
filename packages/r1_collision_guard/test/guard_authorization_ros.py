#!/usr/bin/env python3
"""Real Guard process, isolated DDS, synthetic inputs, test-only velocity topics.

Not a hardware/ControllerServer/solver test. Requires the explicit isolated
flag; starts no hardware driver. Each case uses its own guard process/boot.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

EXECUTABLE = Path(os.environ.get('NEUPAN_GUARD_EXECUTABLE', '/home/rpp/neupan_failstop_ws/install/r1_collision_guard/lib/r1_collision_guard/r1_collision_guard_node'))


@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'isolated opt-in only')
class GuardAuthorizationROS(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '188')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.assertTrue(str(EXECUTABLE.resolve()).startswith('/home/rpp/neupan_failstop_ws/'))
        rclpy.init()
        self.addCleanup(self.shutdown)
        self.node = Node('guard_authorization_test')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.log = open(Path(self.temp.name)/'guard.log', 'w+')
        params = {
            'footprint_points': '[0.3, 0.2, 0.3, -0.2, -0.3, -0.2, -0.3, 0.2]',
            'require_arms_stowed': 'false', 'require_localization_ready': 'false',
            'require_dynamic_footprint': 'true' if 'footprint' in self._testMethodName else 'false',
            'dynamic_footprint_topic': '/test_wp2/footprint', 'collision_check_enabled': 'false',
            'cmd_vel_in_topic': '/test_wp2/input', 'source_cmd_vel_topic': '/test_wp2/raw',
            'cmd_vel_out_topic': '/test_wp2/output', 'status_topic': '/test_wp2/status',
            'scan_topic': '/test_wp2/scan', 'odom_topic': '/test_wp2/odom',
            'worker_health_topic': '/test_wp2/health', 'planner_source_topic': '/test_wp2/source',
            'motion_grant_topic': '/test_wp2/grant',
            'worker_health_timeout_sec': '0.30', 'navigation_authority_timeout_sec': '0.30',
            'planner_source_timeout_sec': '0.30', 'planner_input_timeout_sec': '0.40',
            'planner_recovery_timeout_sec': '1.0',
        }
        args = [str(EXECUTABLE), '--ros-args']
        for key,value in params.items(): args += ['-p', key+':='+value]
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)
        self.outputs, self.statuses = [], []
        self.subs = [
            self.node.create_subscription(Twist, '/test_wp2/output', lambda msg:self.outputs.append((time.monotonic(),msg.linear.x,msg.angular.z)),20),
            self.node.create_subscription(String, '/test_wp2/status', lambda msg:self.statuses.append(json.loads(msg.data)),20),
        ]
        self.pubs = {k:self.node.create_publisher(t,'/test_wp2/'+k,20) for k,t in [
            ('input',Twist),('raw',Twist),('scan',LaserScan),('odom',Odometry),
            ('health',String),('source',String),('grant',String),('footprint',PolygonStamped)]}
        self.health_seq = self.source_seq = self.grant_seq = 0
        self.next_input=0
        self.guard_boot = None
        self.grant_state = None
        self.health_enabled = self.source_enabled = self.odom_enabled = True
        self.worker_boot='worker-a'
        self.generation=7
        self.input_enabled=self.footprint_enabled=True
        self.input_age=.01
        self.nonfinite_command=False
        self.command_value=0.
        self.old_source=None
        self.duplicate=False
        self.corrupt_duplicate=False
        self.arm()

    def shutdown(self):
        if hasattr(self,'proc'):
            self.proc.send_signal(signal.SIGINT)
            try: self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill();self.proc.wait(timeout=3)
        if hasattr(self,'node'):self.node.destroy_node()
        rclpy.shutdown()
        if hasattr(self,'log'):
            self.log.seek(0)
            if self.proc.returncode not in (0,-2): print(self.log.read())
            self.log.close()

    def context(self):
        return dict(authority_boot_id='authority-a', controller_boot_id='controller-a',
                    navigation_attempt_id='attempt-a', worker_boot_id=self.worker_boot,
                    worker_generation=self.generation,path_version=getattr(self,'route',1),goal_version=getattr(self,'route',1)-1)

    def publish_json(self,key,payload):
        self.pubs[key].publish(String(data=json.dumps(payload, allow_nan=False)))

    def tick(self):
        if self.proc.poll() is not None:
            self.log.seek(0);self.fail('Guard exited: '+self.log.read())
        now=time.monotonic()
        if now >= self.next_input:
            self.next_input=now+.025
            stamp=self.node.get_clock().now().to_msg()
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom'
            odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
            if self.odom_enabled:self.pubs['odom'].publish(odom)
            if self.footprint_enabled:
                footprint=PolygonStamped();footprint.header.stamp=stamp;footprint.header.frame_id='base_link'
                footprint.polygon.points=[Point32(x=x,y=y,z=0.) for x,y in [(0.3,0.2),(0.3,-0.2),(-0.3,-0.2),(-0.3,0.2)]]
                self.pubs['footprint'].publish(footprint)
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link'
            scan.angle_min=-3.14;scan.angle_increment=.02;scan.angle_max=3.14
            scan.range_min=.05;scan.range_max=8.;scan.ranges=[7.]*315
            self.pubs['scan'].publish(scan)
            if self.health_enabled:
                self.health_seq+=1
                self.publish_json('health',dict(schema='r1_neupan_worker_health_v1',boot_id=self.worker_boot,
                    sequence=self.health_seq,state='RUNNING',worker_generation=self.generation,
                    navigation_attempt_id='attempt-a',monotonic=now,fault_id=None,fault_code=None))
            if self.source_enabled:
                if not self.duplicate or self.old_source is None:
                    self.source_seq+=1
                    source=dict(self.context(),schema='r1_neupan_planner_source_v2',request_id=self.source_seq,
                        cycle_id=self.source_seq,scan_counter=self.source_seq,finite=True,
                        scan_receive_monotonic=now-self.input_age,request_dispatched_monotonic=now-.008,
                        worker_solve_end_monotonic=now-.003,planner_result_monotonic=now,
                        command=dict(linear_x=.1,angular_z=.0))
                    self.old_source=source
                source=json.loads(json.dumps(self.old_source))
                if self.corrupt_duplicate:source['command']['linear_x']=.15
                self.publish_json('source',source)
            if self.grant_state:
                self.grant_seq+=1
                self.publish_json('grant',dict(self.context(),schema='r1_neupan_motion_grant_v1',
                    guard_boot_id=self.guard_boot,grant_id=getattr(self,'grant_id','grant-a'),sequence=self.grant_seq,
                    state=self.grant_state,monotonic=now))
            cmd=Twist();cmd.linear.x=self.command_value
            if self.nonfinite_command:cmd.linear.x=float('nan')
            if self.input_enabled:self.pubs['input'].publish(cmd)
            self.pubs['raw'].publish(cmd)
        rclpy.spin_once(self.node,timeout_sec=.01)

    def until(self,predicate,seconds=3):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:self.tick()
        self.assertTrue(predicate(),self.statuses[-1] if self.statuses else 'no Guard status')

    def arm(self):
        self.until(lambda:bool(self.statuses))
        self.guard_boot=self.statuses[-1]['guard_boot_id']
        self.assertFalse(any(abs(row[1])>1e-8 for row in self.outputs))
        self.grant_state='PREPARE'
        self.until(lambda:self.statuses[-1]['authorization_state']=='PREPARED')
        self.assertFalse(any(abs(row[1])>1e-8 for row in self.outputs))
        self.until(lambda:self.statuses[-1].get('command_barrier_ready',False))
        self.grant_state='COMMIT'
        self.until(lambda:self.statuses[-1]['authorization_state']=='ARMED')
        self.command_value=.1
        self.until(lambda:any(abs(row[1])>.05 for row in self.outputs))
        self.assertEqual(self.statuses[-1]['authorization_state'],'ARMED')
        self.assertFalse(any(name=='/cmd_vel' for name,_ in self.node.get_topic_names_and_types()))

    def assert_stopped(self,reason):
        start=time.monotonic()
        self.until(lambda:self.statuses[-1]['reason']==reason)
        settled=time.monotonic()+.04
        while time.monotonic()<settled:self.tick()
        mark=time.monotonic()
        while time.monotonic()<mark+.18:self.tick()
        rows=[r for r in self.outputs if r[0]>=mark]
        self.assertGreaterEqual(len(rows),4)
        self.assertTrue(all(abs(r[1])<1e-8 and abs(r[2])<1e-8 for r in rows), rows)
        print(json.dumps({'scenario':self._testMethodName,'stop_reason':reason,
                          'observed_window_sec':time.monotonic()-start,'continuous_zero_samples':len(rows)}))

    def test_route_commit_cannot_reuse_nonzero_smoother_tail(self):
        self.route=2
        self.grant_id='grant-route-2'
        self.grant_state='PREPARE'
        self.until(lambda:self.statuses[-1].get('grant_id')==self.grant_id)
        # Model an out-of-order/over-eager controller while both untagged
        # command streams still contain the previous route's nonzero tail.
        self.grant_state='COMMIT'
        start=time.monotonic()
        while time.monotonic()<start+.18:self.tick()
        rows=[r for r in self.outputs if r[0]>start+.04]
        self.assertTrue(rows)
        self.assertTrue(all(abs(v)<1e-8 and abs(w)<1e-8 for _,v,w in rows),rows)
        self.assertNotEqual(self.statuses[-1]['authorization_state'],'ARMED')
        self.command_value=0.
        self.until(lambda:self.statuses[-1]['authorization_state']=='ARMED')
        cutoff=time.monotonic()
        self.command_value=.1
        self.until(lambda:any(t>cutoff and v>.05 for t,v,w in self.outputs))
        self.assertEqual(self.statuses[-1]['path_version'],2)
        self.assertEqual(self.statuses[-1]['grant_id'],self.grant_id)

    def test_source_freeze_stops_despite_fresh_raw_and_health_then_recovers(self):
        self.source_enabled=False
        self.assert_stopped('planner_result_stale')
        self.source_enabled=True
        cutoff=time.monotonic()
        self.until(lambda:any(t>cutoff and abs(v)>.05 for t,v,w in self.outputs))

    def test_repeated_source_does_not_refresh_age(self):
        self.duplicate=True
        self.assert_stopped('planner_result_stale')

    def test_fatal_wins_and_late_running_and_results_cannot_clear(self):
        now=time.monotonic();self.health_seq+=1
        self.publish_json('health',dict(schema='r1_neupan_worker_health_v1',boot_id=self.worker_boot,
            sequence=self.health_seq,state='FAULT_LATCHED',worker_generation=7,navigation_attempt_id='attempt-a',
            monotonic=now,fault_id='fault-a',fault_code='worker_response_failure'))
        self.assert_stopped('worker_fault_latched')

    def test_health_dropout_stops_with_continuous_source_and_raw(self):
        self.health_enabled=False
        self.assert_stopped('worker_health_stale')

    def test_authority_dropout_stops_with_continuous_health_and_raw(self):
        self.grant_state=None
        self.assert_stopped('navigation_authority_stale')

    def test_g2_is_rejected_and_late_g1_does_not_revive(self):
        self.generation=8
        self.assert_stopped('unauthorized_worker_generation')
        self.generation=7
        self.assert_stopped('unauthorized_worker_generation')

    def test_duplicate_identity_with_different_speed_latches(self):
        self.duplicate=True;self.corrupt_duplicate=True
        self.assert_stopped('duplicate_command_mismatch')

    def test_original_odom_watchdog_still_enforces_stop(self):
        self.odom_enabled=False
        self.assert_stopped('odom_stale')

    def test_revoke_permanently_invalidates_attempt(self):
        self.grant_state='REVOKE'
        self.assert_stopped('navigation_not_armed')
        self.grant_state='PREPARE'
        self.assert_stopped('navigation_not_armed')

    def test_smoother_input_loss_stops_even_with_raw_and_source_alive(self):
        self.input_enabled=False
        self.assert_stopped('command_stale')

    def test_disabled_collision_profile_does_not_gate_on_footprint(self):
        self.footprint_enabled=False
        start=time.monotonic()
        while time.monotonic()<start+.7:self.tick()
        rows=[r for r in self.outputs if r[0]>start+.55]
        self.assertTrue(rows)
        self.assertTrue(all(v>.05 for _,v,w in rows),rows)
        self.assertEqual(self.statuses[-1]['reason'],'collision_check_disabled')

    def test_old_input_is_not_laundered_by_fresh_result_publication(self):
        self.input_age=.8
        self.assert_stopped('planner_result_stale')

    def test_nonfinite_command_status_remains_valid_json_and_output_zero(self):
        self.nonfinite_command=True
        self.assert_stopped('source_command_nonfinite')

    def test_source_recovery_deadline_is_terminal(self):
        self.source_enabled=False
        self.assert_stopped('planner_result_recovery_timeout')
        self.source_enabled=True
        self.assert_stopped('planner_result_recovery_timeout')


if __name__=='__main__':unittest.main()
