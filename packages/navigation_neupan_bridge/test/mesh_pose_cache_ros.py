#!/usr/bin/env python3
"""Real mesh node, synthetic TF/joints/scans in a local isolated ROS domain."""
import json, os, signal, subprocess, tempfile, time, unittest
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1","isolated test only")
class MeshPoseCache(unittest.TestCase):
    def test_gripper_motion_and_short_tf_gap(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"),"197")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"),"1")
        rclpy.init(); node=Node("mesh_cache_test_peer")
        tf=TransformBroadcaster(node)
        left=node.create_publisher(JointState,"/left_arm/joint_states",20)
        right=node.create_publisher(JointState,"/right_arm/joint_states",20)
        scan_pub=node.create_publisher(LaserScan,"/scan_neupan",qos_profile_sensor_data)
        components=[];states=[];scans=[]
        node.create_subscription(Float64MultiArray,"/r1_arm_mesh_components",
            lambda m:components.append((time.monotonic(),list(m.data))),20)
        node.create_subscription(String,"/r1_arm_mesh_filter/status",
            lambda m:states.append((time.monotonic(),json.loads(m.data))),20)
        node.create_subscription(LaserScan,"/scan_neupan_mesh_shadow",
            lambda m:scans.append(time.monotonic()),qos_profile_sensor_data)
        frames=[]
        for arm in ("left_","right_"):
            frames += [arm+"base"]+[arm+"link"+str(i) for i in range(1,8)]
        for arm,g in (("left_","L_"),("right_","R_")):
            frames += [arm+"moduan_link",g+"omniPicker_base_link"]
            frames += [g+"omniPicker_hand_"+s for s in ("narrow1_Link","narrow2_Link",
                "narrow3_Link","narrow_loop_Link","wide1_Link","wide2_Link","wide3_Link","wide_loop_Link")]
        target="L_omniPicker_hand_narrow1_Link"
        exe=os.environ["NEUPAN_TEST_ARM_FOOTPRINT"]
        log=tempfile.TemporaryFile(mode="w+")
        proc=subprocess.Popen([exe,"--ros-args","-p","mesh_projection_frequency:=4.0",
            "-p","mesh_filter_frequency:=15.0"],stdout=log,stderr=subprocess.STDOUT)
        gripper=0.; missing=False; stale_joints=False; duplicate_joint=False; next_pulse=0.
        def tick():
            nonlocal next_pulse
            self.assertIsNone(proc.poll(),"mesh node exited")
            now=time.monotonic()
            if now>=next_pulse:
                next_pulse=now+.04;stamp=node.get_clock().now().to_msg()
                for prefix,pub in (("left_",left),("right_",right)):
                    msg=JointState();msg.header.stamp=stamp
                    if stale_joints: msg.header.stamp.sec=1
                    msg.name=[prefix+"joint"+str(i) for i in range(1,8)];msg.position=[0.]*7
                    if duplicate_joint and prefix=="right_": msg.name[-1]=msg.name[0]
                    pub.publish(msg)
                transforms=[]
                for name in frames:
                    if missing and name==target:continue
                    msg=TransformStamped();msg.header.stamp=stamp;msg.header.frame_id="base_link"
                    msg.child_frame_id=name;msg.transform.rotation.w=1.
                    if name==target:msg.transform.translation.x=gripper
                    transforms.append(msg)
                tf.sendTransform(transforms)
                scan=LaserScan();scan.header.frame_id="base_link";scan.header.stamp=stamp
                scan.angle_min=-3.14;scan.angle_max=3.14;scan.angle_increment=.01
                scan.range_min=.15;scan.range_max=8.;scan.ranges=[7.]*629
                scan_pub.publish(scan)
            rclpy.spin_once(node,timeout_sec=.01)
        def until(predicate,timeout=8):
            end=time.monotonic()+timeout
            while not predicate() and time.monotonic()<end:tick()
            self.assertTrue(predicate(),str(states[-3:]))
        def component(data,index):
            off=0
            for _ in range(index):
                n=int(data[off]);off+=1+2*n
            n=int(data[off]);return data[off+1:off+1+2*n]
        try:
            until(lambda:components and len(components[-1][1])>0 and
                states and states[-1][1].get("state")=="READY",15.)
            before=component(components[-1][1],18);marker=time.monotonic()
            gripper=.10
            until(lambda:any(t>marker and data and component(data,18)!=before for t,data in components),5.)
            self.assertTrue(any(t>marker for t in scans))
            # Arm joints never changed; only a gripper TF changed.
            missing=True;start=time.monotonic()
            until(lambda:any(t>start and s.get("reason")=="tf_gap_using_compatible_cache" for t,s in states),3.)
            until(lambda:any(t>start+.5 for t in scans),1.)
            until(lambda:any(t>start and s.get("reason") in ("tf_or_cache_invalid","geometry_cache_expired")
                for t,s in states),3.)
            marker=time.monotonic();missing=False
            until(lambda:any(t>marker and s.get("state")=="READY" for t,s in states),5.)
            until(lambda:any(t>marker for t in scans),3.)
            # Fresh DDS arrivals with old embedded joint timestamps must not
            # renew trust in stale mechanical pose.
            stale_joints=True;marker=time.monotonic()
            until(lambda:any(t>marker and s.get("reason")=="arm_state_stale" for t,s in states),3.)
            stale_joints=False;marker=time.monotonic()
            until(lambda:any(t>marker and s.get("state")=="READY" for t,s in states),5.)
            until(lambda:any(t>marker for t in scans),3.)
            duplicate_joint=True;marker=time.monotonic()
            until(lambda:any(t>marker and s.get("reason")=="arm_state_stale" for t,s in states),3.)
            duplicate_joint=False;marker=time.monotonic()
            until(lambda:any(t>marker and s.get("state")=="READY" for t,s in states),5.)
        finally:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
            log.seek(0);data=log.read()
            if proc.returncode not in (0,-2):print(data)
            log.close();node.destroy_node();rclpy.shutdown()

if __name__=="__main__":unittest.main()
