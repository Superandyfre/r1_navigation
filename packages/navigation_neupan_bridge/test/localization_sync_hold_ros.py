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
from nav_msgs.msg import Odometry
from nav2_msgs.srv import GetCostmap
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST") == "1", "isolated opt-in")
class LocalizationSyncHold(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"), "193")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        rclpy.init()
        self.node = Node("test_localization_sync_inputs")
        self.proc = subprocess.Popen([os.environ["NEUPAN_TEST_MONITOR"], "--ros-args",
            "-p", "require_dynamic_footprint:=true", "-p", "stable_window_sec:=0.2",
            "-p", "costmap_service:=/test/costmap", "-p", "odom_topic:=/test/odom",
            "-p", "localization_status_topic:=/test/localization",
            "-p", "dynamic_footprint_topic:=/test/footprint",
            "-p", "costmap_footprint_topic:=/test/published_footprint",
            "-p", "status_topic:=/test/status"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.tf = TransformBroadcaster(self.node)
        self.odom = self.node.create_publisher(Odometry, "/test/odom", 20)
        self.loc = self.node.create_publisher(String, "/test/localization", 20)
        self.fp = self.node.create_publisher(PolygonStamped, "/test/footprint", 20)
        self.mapfp = self.node.create_publisher(PolygonStamped, "/test/published_footprint", 20)
        self.srv = self.node.create_service(GetCostmap, "/test/costmap", lambda req, resp: resp)
        self.status = []
        self.sub = self.node.create_subscription(String, "/test/status", lambda m:self.status.append(json.loads(m.data)), 20)
        self.mismatch = False
        self.drop_footprint = False
        self.drop_localization = False
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
                self.loc.publish(String(data=json.dumps(dict(schema="r1_indoor_localization_status_v1",state="NORMAL",reason="ready"))))
            for pub, frame, dx in ((self.fp,"base_link",0.),(self.mapfp,"map",.3 if self.mismatch else 0.)):
                if pub is self.fp and self.drop_footprint: continue
                msg=PolygonStamped();msg.header.frame_id=frame
                msg.header.stamp=(now-rclpy.duration.Duration(seconds=.1)).to_msg()
                msg.polygon.points=[Point32(x=x+dx,y=y,z=0.) for x,y in ((-.3,-.2),(-.3,.2),(.3,.2),(.3,-.2))]
                pub.publish(msg)
        rclpy.spin_once(self.node,timeout_sec=.01)

    def wait_for(self, pred, timeout=6):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.tick()
            if self.status and pred(self.status[-1]): return
        self.fail(str(self.status[-5:]))

    def test_short_mismatch_holds_then_recovers(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.mismatch=True
        self.wait_for(lambda s:s["state"]=="HOLD")
        self.assertFalse(self.status[-1]["fault_latched"])
        self.mismatch=False
        self.wait_for(lambda s:s["state"]=="READY")
        self.assertFalse(any(s["state"]=="FAULT" for s in self.status))

    def test_brief_footprint_dropout_holds_and_recovers(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_footprint=True
        self.wait_for(lambda s:s["state"]=="HOLD")
        self.assertEqual(self.status[-1]["reason"],"dynamic_footprint_unavailable_or_stale")
        self.drop_footprint=False
        self.wait_for(lambda s:s["state"]=="READY")
        self.assertFalse(any(s["state"]=="FAULT" for s in self.status))

    def test_status_dropout_holds_then_faults_at_original_deadline(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.drop_localization=True
        self.wait_for(lambda s:s["state"]=="HOLD")
        self.assertEqual(self.status[-1]["reason"],"localization_status_stale")
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.assertEqual(self.status[-1]["reason"],"localization_status_stale")

    def test_persistent_mismatch_faults(self):
        self.wait_for(lambda s:s["state"]=="READY")
        self.mismatch=True
        self.wait_for(lambda s:s["state"]=="FAULT")
        self.assertEqual(self.status[-1]["reason"],"costmap_footprint_not_synchronized")

if __name__=="__main__": unittest.main()
