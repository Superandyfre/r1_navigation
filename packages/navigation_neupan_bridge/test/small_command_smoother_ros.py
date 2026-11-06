#!/usr/bin/env python3
"""Real Nav2 smoother with production profile, isolated test-only Twist topics."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState

ROOT=Path(__file__).resolve().parents[3]
EXE=Path("/home/rpp/rpp_ws/install/nav2_velocity_smoother/lib/nav2_velocity_smoother/velocity_smoother")
@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1","isolated opt-in")
class SmallCommandSmoother(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"),"195")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"),"1")
        rclpy.init();self.node=Node("small_command_smoother_test")
        self.temp=tempfile.TemporaryDirectory()
        params=yaml.safe_load((ROOT/"packages/navigation_neupan_bridge/config/nav2_params_r1_neupan_hw1.yaml").read_text())["neupan_velocity_smoother"]["ros__parameters"]
        params["use_sim_time"]=False
        path=Path(self.temp.name)/"params.yaml";path.write_text(yaml.safe_dump({"neupan_velocity_smoother":{"ros__parameters":params}}))
        self.proc=subprocess.Popen([str(EXE),"--ros-args","-r","__node:=neupan_velocity_smoother",
            "--params-file",str(path),"-r","cmd_vel:=/test_small/input","-r","cmd_vel_smoothed:=/test_small/output"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        self.pub=self.node.create_publisher(Twist,"/test_small/input",10);self.rows=[]
        self.sub=self.node.create_subscription(Twist,"/test_small/output",lambda m:self.rows.append((time.monotonic(),m.linear.x,m.angular.z)),10)
        self.client=self.node.create_client(ChangeState,"/neupan_velocity_smoother/change_state")
        self.wait(lambda:self.client.service_is_ready())
        for transition in (1,3):
            req=ChangeState.Request();req.transition.id=transition;f=self.client.call_async(req)
            self.wait(f.done);self.assertTrue(f.result().success)
    def tearDown(self):
        self.proc.send_signal(signal.SIGINT)
        try:self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        self.node.destroy_node();rclpy.shutdown();self.temp.cleanup()
    def wait(self,pred,seconds=5):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not pred():rclpy.spin_once(self.node,timeout_sec=.02)
        self.assertTrue(pred())
    def test_hw1_profile_does_not_add_second_command_deadband(self):
        data=yaml.safe_load((ROOT/"packages/navigation_neupan_bridge/config/robot_r1_hw1.yaml").read_text())
        self.assertFalse(data["n1_16_production_bridge"]["ros__parameters"]["command_stabilizer_enabled"])
        profile=yaml.safe_load((ROOT/"packages/navigation_neupan_bridge/config/nav2_params_r1_neupan_hw1.yaml").read_text())
        self.assertEqual(profile["neupan_velocity_smoother"]["ros__parameters"]["deadband_velocity"],[0.,0.,0.])

    def test_gentle_commands_are_not_zeroed(self):
        msg=Twist();msg.linear.x=.004;msg.angular.z=.005
        start=time.monotonic()
        while time.monotonic()<start+.8:
            self.pub.publish(msg);rclpy.spin_once(self.node,timeout_sec=.02)
        recent=[(v,w) for t,v,w in self.rows if t>start+.5]
        self.assertTrue(recent)
        self.assertTrue(all(v>0 and w>0 for v,w in recent),recent)
        self.assertFalse(any(name=="/cmd_vel" for name,_ in self.node.get_topic_names_and_types()))

if __name__=="__main__":unittest.main()
