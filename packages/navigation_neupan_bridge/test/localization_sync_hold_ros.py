#!/usr/bin/env python3
"""Actual localization monitor with synthetic TF, odometry and costmap; no robot."""
import json
import os
import signal
import subprocess
import time
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.srv import GetCostmap
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST") == "1", "isolated opt-in")
class LocalizationSyncHold(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"), "193")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        rclpy.init()
        self.node = Node("test_localization_sync_inputs")
        self.proc = subprocess.Popen([os.environ["NEUPAN_TEST_MONITOR"], "--ros-args",
            "-p", "require_dynamic_footprint:=true", "-p", "stable_window_sec:=3.0",
            "-p", "costmap_service:=/test/costmap", "-p", "odom_topic:=/test/odom",
            "-p", "localization_status_topic:=/test/localization",
            "-p", "dynamic_footprint_topic:=/test/footprint",
            "-p", "costmap_footprint_topic:=/test/published_footprint",
            "-p", "status_topic:=/test/status", "-p", "static_map_topic:=/test/static_map",
            "-p", "static_map_overlap_hard_fault:="+str(getattr(self,"hard_static_map",True)).lower()], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.tf = TransformBroadcaster(self.node)
        self.odom = self.node.create_publisher(Odometry, "/test/odom", 20)
        self.loc = self.node.create_publisher(String, "/test/localization", 20)
        self.fp = self.node.create_publisher(PolygonStamped, "/test/footprint", 20)
        self.mapfp = self.node.create_publisher(PolygonStamped, "/test/published_footprint", 20)
        map_qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.node.create_publisher(OccupancyGrid,"/test/static_map",map_qos)
        self.lethal_map=False
        self.srv = self.node.create_service(GetCostmap, "/test/costmap", lambda req, resp: resp)
        self.status = []
        self.sub = self.node.create_subscription(String, "/test/status", lambda m:self.status.append(json.loads(m.data)), 20)
        self.mismatch = False
        self.shrink = False
        self.drop_footprint = False
        self.drop_published_footprint = False
        self.drop_localization = False
        self.localization_state = "NORMAL"
        self.last = 0.

    def tearDown(self):
        self.proc.send_signal(signal.SIGINT)
        try: self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired: self.proc.kill(); self.proc.wait()
        self.node.destroy_node()
        rclpy.shutdown()

    def tick(self):
        if time.monotonic() - self.last > .05:
            self.last = time.monotonic()
            now = self.node.get_clock().now()
            stamp = now.to_msg()
            tf = TransformStamped(); tf.header.stamp = stamp; tf.header.frame_id="map"; tf.child_frame_id="base_link"; tf.transform.rotation.w=1.
            self.tf.sendTransform(tf)
            odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id="odom"; odom.child_frame_id="base_link"; odom.pose.pose.orientation.w=1.; self.odom.publish(odom)
            if not self.drop_localization:
                self.loc.publish(String(data=json.dumps(dict(schema="r1_indoor_localization_status_v1",state=self.localization_state,reason="ready"))))
            if self.lethal_map:
                grid=OccupancyGrid();grid.header.frame_id="map";grid.header.stamp=stamp
                grid.info.resolution=.1;grid.info.width=20;grid.info.height=20
                grid.info.origin.position.x=-1.;grid.info.origin.position.y=-1.
                grid.info.origin.orientation.w=1.;grid.data=[0]*400;grid.data[210]=100
                self.map_pub.publish(grid)
            for pub, frame, dx in ((self.fp,"base_link",0.),(self.mapfp,"map",.3 if self.mismatch else 0.)):
                if pub is self.fp and self.drop_footprint: continue
                if pub is self.mapfp and self.drop_published_footprint: continue
                msg=PolygonStamped();msg.header.frame_id=frame
                msg.header.stamp=(now-rclpy.duration.Duration(seconds=.1)).to_msg()
                msg.polygon.points=[Point32(x=x*(.5 if self.shrink and pub is self.fp else 1.)+dx,
                    y=y*(.5 if self.shrink and pub is self.fp else 1.),z=0.)
                    for x,y in ((-.3,-.2),(-.3,.2),(.3,.2),(.3,-.2))]
                pub.publish(msg)
        rclpy.spin_once(self.node,timeout_sec=.01)

    def wait_for(self, pred, timeout=6):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.tick()
            if self.status and pred(self.status[-1]): return
        self.fail(str(self.status[-5:]))

    def test_smaller_live_footprint_continues_with_conservative_previous_shape(self):
        self.wait_for(lambda s:s["state"]=="READY")
        start=len(self.status)
        self.shrink=True
        self.wait_for(lambda s:s["reason"]=="using_conservative_costmap_footprint")
        deadline=time.monotonic()+1.2
        while time.monotonic()<deadline:self.tick()
        self.assertTrue(all(s["state"]=="READY" for s in self.status[start:]))

    def test_footprint_mismatch_stays_ready_and_reports_pending(self):
        self.wait_for(lambda s:s["state"]=="READY")
        start=len(self.status)
        self.mismatch=True
        self.wait_for(lambda s:s["reason"]=="costmap_footprint_sync_pending")
        deadline=time.monotonic()+1.5
        while time.monotonic()<deadline:self.tick()
        self.assertTrue(all(s["state"]=="READY" for s in self.status[start:]))
        self.assertFalse(self.status[-1]["fault_latched"])
        self.mismatch=False
        self.wait_for(lambda s:s["reason"]=="ready")

    def test_missing_published_footprint_is_diagnostic_only(self):
        self.wait_for(lambda s:s["state"]=="READY")
        start=len(self.status)
        self.drop_published_footprint=True
        self.wait_for(lambda s:s["reason"]=="costmap_footprint_sync_pending",5.)
        deadline=time.monotonic()+1.5
        while time.monotonic()<deadline:self.tick()
        self.assertTrue(all(s["state"]=="READY" for s in self.status[start:]))
        self.assertFalse(self.status[-1]["fault_latched"])

    def test_brief_footprint_dropout_holds_and_recovers(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_footprint=True
        self.wait_for(lambda s:s["state"]=="HOLD")
        self.assertEqual(self.status[-1]["reason"],"dynamic_footprint_unavailable_or_stale")
        self.drop_footprint=False
        recovered_at=time.monotonic()
        self.wait_for(lambda s:s["state"]=="READY")
        self.assertLess(time.monotonic()-recovered_at, 2.0,
                        "do not impose the 3 s cold-start window on a short dropout")
        self.assertFalse(any(s["state"]=="FAULT" for s in self.status))

    def test_brief_status_dropout_continues_until_original_deadline(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_localization=True
        self.wait_for(lambda s:s["reason"]=="localization_status_delayed_using_last_normal")
        self.assertEqual(self.status[-1]["state"],"READY")
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.assertEqual(self.status[-1]["reason"],"localization_status_stale")

    def test_reported_localization_loss_never_uses_old_normal_state(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.localization_state="LOST"
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.assertTrue(self.status[-1]["reason"].startswith("localization_LOST:"))

    def test_persistent_live_footprint_dropout_clears_automatically(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_footprint=True
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.assertEqual(self.status[-1]["reason"],"dynamic_footprint_unavailable_or_stale")
        self.drop_footprint=False
        self.wait_for(lambda s:s["state"]=="READY",8.)
        self.assertFalse(self.status[-1]["fault_latched"])

    def test_static_overlap_is_not_hidden_by_recovering_footprint_fault(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_footprint=True
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.lethal_map=True;self.drop_footprint=False
        self.wait_for(lambda s:s["reason"]=="start_in_lethal_static_map",8.)
        self.assertTrue(self.status[-1]["fault_latched"])

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST") == "1", "isolated opt-in")
class LocalizationOverlapAdvisory(unittest.TestCase):
    hard_static_map=False
    setUp=LocalizationSyncHold.setUp
    tearDown=LocalizationSyncHold.tearDown
    tick=LocalizationSyncHold.tick
    wait_for=LocalizationSyncHold.wait_for

    def test_static_overlap_is_reported_without_latching_stable_localization(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_footprint=True
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.lethal_map=True;self.drop_footprint=False
        self.wait_for(lambda s:s["state"]=="READY" and s.get("static_map_overlap"),8.)
        self.assertFalse(self.status[-1]["static_map_overlap_hard_fault"])
        self.assertFalse(self.status[-1]["fault_latched"])
        self.assertGreater(self.status[-1]["lethal_cells"],0)
        self.localization_state="LOST"
        self.wait_for(lambda s:s["state"]=="FAULT",4.)
        self.assertTrue(self.status[-1]["reason"].startswith("localization_LOST:"))

if __name__=="__main__": unittest.main()
