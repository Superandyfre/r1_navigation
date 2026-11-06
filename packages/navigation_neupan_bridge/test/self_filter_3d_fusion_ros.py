#!/usr/bin/env python3
"""Shadow 3D filter + actual pointcloud projection and dual-scan fusion."""
import json,os,signal,struct,subprocess,tempfile,time,unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1","isolated only")
class ShadowFusion(unittest.TestCase):
    def test_brief_tf_gap_keeps_fused_scan_flowing(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"),"197")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"),"1")
        rclpy.init();node=Node("filter_3d_fusion_test_peer");tf=TransformBroadcaster(node)
        cp=node.create_publisher(PointCloud2,"/lidar/pointcloud",qos_profile_sensor_data)
        sp=node.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
        fused=[];statuses=[]
        node.create_subscription(LaserScan,"/test_3d_fused",lambda m:fused.append((time.monotonic(),m)),qos_profile_sensor_data)
        node.create_subscription(String,"/r1_self_filter_3d_shadow/status",lambda m:statuses.append(json.loads(m.data)),50)
        frames=[]
        for arm in ("left_","right_"):
            frames += [arm+"base"]+[arm+"link"+str(i) for i in range(1,8)]
        for arm,g in (("left_","L_"),("right_","R_")):
            frames += [arm+"moduan_link",g+"omniPicker_base_link"]
            frames += [g+"omniPicker_hand_"+s for s in ("narrow1_Link","narrow2_Link","narrow3_Link","narrow_loop_Link","wide1_Link","wide2_Link","wide3_Link","wide_loop_Link")]
        target="L_omniPicker_hand_narrow1_Link";missing=False;last_tf=last_scan=0.
        paths=[os.environ["NEUPAN_TEST_3D_SHADOW"],os.environ["NEUPAN_TEST_CLOUD_TO_SCAN"],os.environ["NEUPAN_TEST_SCAN_FUSION"]]
        commands=[
            [paths[0]],
            [paths[1],"--ros-args","--params-file",os.environ["NEUPAN_TEST_CLOUD_CONFIG"],
             "-r","cloud_in:=/r1_self_filter_3d_shadow/cloud","-r","scan:=/test_3d_front"],
            [paths[2],"--ros-args","--params-file",os.environ["NEUPAN_TEST_FUSION_CONFIG"],
             "-p","livox_scan_topic:=/test_3d_front","-p","rear_scan_topic:=/r1_self_filter_3d_shadow/rear_scan",
             "-p","output_scan_topic:=/test_3d_fused","-p","status_topic:=/test_3d_fusion_status"]]
        logs=[];procs=[]
        for cmd in commands:
            log=tempfile.TemporaryFile(mode="w+");logs.append(log)
            procs.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
        def tick():
            nonlocal last_tf,last_scan
            for proc in procs:self.assertIsNone(proc.poll(),"component exited")
            now=time.monotonic();stamp=node.get_clock().now()
            if now-last_tf>=.02:
                last_tf=now;messages=[]
                for frame in ["lidar_link","laser_link",*frames]:
                    if missing and frame==target:continue
                    t=TransformStamped();t.header.stamp=stamp.to_msg();t.header.frame_id="base_link";t.child_frame_id=frame;t.transform.rotation.w=1.
                    if frame not in ("lidar_link","laser_link"):t.transform.translation.x=20.
                    messages.append(t)
                tf.sendTransform(messages)
            if now-last_scan>=.10:
                last_scan=now;sample=rclpy.time.Time(nanoseconds=stamp.nanoseconds-30000000).to_msg()
                cloud=PointCloud2();cloud.header.stamp=sample;cloud.header.frame_id="lidar_link"
                cloud.height=1;cloud.width=2;cloud.point_step=20;cloud.row_step=40;cloud.is_dense=True
                cloud.fields=[PointField(name="x",offset=0,datatype=7,count=1),PointField(name="y",offset=4,datatype=7,count=1),PointField(name="z",offset=8,datatype=7,count=1),PointField(name="timestamp",offset=12,datatype=8,count=1)]
                ts=float(sample.sec*1000000000+sample.nanosec)
                cloud.data=struct.pack("<fffdfffd",3.,0.,0.,ts,3.,.2,0.,ts);cp.publish(cloud)
                scan=LaserScan();scan.header.stamp=sample;scan.header.frame_id="laser_link"
                scan.angle_min=-3.14;scan.angle_max=3.14;scan.angle_increment=.0175
                scan.time_increment=.0001;scan.scan_time=.036;scan.range_min=.15;scan.range_max=8.
                scan.ranges=[7.]*360;sp.publish(scan)
            rclpy.spin_once(node,timeout_sec=.005)
        def until(pred,timeout=15):
            end=time.monotonic()+timeout
            while not pred() and time.monotonic()<end:tick()
            self.assertTrue(pred(),str(statuses[-4:]))
        try:
            # Construct real mesh index and warm TF history.
            until(lambda:cp.get_subscription_count()>0 and sp.get_subscription_count()>0,60.)
            until(lambda:len(fused)>=8,15.)
            start=time.monotonic()
            while time.monotonic()<start+3.:
                missing=start+1.0<time.monotonic()<start+1.16
                tick()
            rows=[t for t,m in fused if start+.25<t<start+2.9]
            self.assertGreater(len(rows),15)
            gaps=[b-a for a,b in zip(rows,rows[1:])]
            self.assertLess(max(gaps),.25,repr(gaps))
            self.assertTrue(any(s.get("reason") in ("stable_tf_fallback","reused_last_filtered_original_stamp") for s in statuses))
            self.assertFalse(any(name=="/cmd_vel" for name,_ in node.get_topic_names_and_types()))
        finally:
            for proc in procs:
                if proc.poll() is None:proc.send_signal(signal.SIGINT)
            for proc,log in zip(procs,logs):
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
                if proc.returncode not in (0,-2):log.seek(0);print(log.read())
                log.close()
            node.destroy_node();rclpy.shutdown()

if __name__=="__main__":unittest.main()
