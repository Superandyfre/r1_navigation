#!/usr/bin/env python3
"""Historical moving-mesh TF against both sensors, isolated ROS domain."""
import json,math,os,signal,struct,subprocess,tempfile,time,unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1","isolated only")
class ShadowFilter(unittest.TestCase):
    def test_historical_pose_for_cloud_and_rear_scan(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"),"197")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"),"1")
        root=Path(get_package_share_directory("rpp_components_description"))
        mesh=root/"meshes/gripper/agibot/omniPicker/narrow1_Link.STL"
        raw=mesh.read_bytes()
        vx,vy,vz=struct.unpack_from("<fff",raw,96)
        rclpy.init();node=Node("filter_3d_shadow_test_peer");tf=TransformBroadcaster(node)
        cp=node.create_publisher(PointCloud2,"/lidar/pointcloud",qos_profile_sensor_data)
        sp=node.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
        clouds=[];scans=[];states=[]
        node.create_subscription(PointCloud2,"/r1_self_filter_3d_shadow/cloud",lambda m:clouds.append(m),qos_profile_sensor_data)
        node.create_subscription(LaserScan,"/r1_self_filter_3d_shadow/rear_scan",lambda m:scans.append(m),qos_profile_sensor_data)
        node.create_subscription(String,"/r1_self_filter_3d_shadow/status",lambda m:states.append(m.data),20)
        frames=[]
        for arm in ("left_","right_"):
            frames += [arm+"base"]+[arm+"link"+str(i) for i in range(1,8)]
        for arm,g in (("left_","L_"),("right_","R_")):
            frames += [arm+"moduan_link",g+"omniPicker_base_link"]
            frames += [g+"omniPicker_hand_"+s for s in ("narrow1_Link","narrow2_Link","narrow3_Link","narrow_loop_Link","wide1_Link","wide2_Link","wide3_Link","wide_loop_Link")]
        target="L_omniPicker_hand_narrow1_Link";frames.append("held_object")
        exe=os.environ["NEUPAN_TEST_3D_SHADOW"];log=tempfile.TemporaryFile(mode="w+")
        uri="package://rpp_components_description/meshes/gripper/agibot/omniPicker/narrow1_Link.STL"
        proc=subprocess.Popen([exe,"--ros-args","-p","additional_mesh_frames:=[held_object]",
            "-p","additional_mesh_uris:=["+uri+"]","-p","additional_mesh_scales:=[1.0]",
            "-p","front_echo_box_base_xyz:=[0.045,0.235,-0.405,-0.305,0.670,0.795]"],
            stdout=log,stderr=subprocess.STDOUT)
        position=1.;missing=False;last=0.
        def tick():
            nonlocal last
            self.assertIsNone(proc.poll(),"shadow process exited")
            now=time.monotonic()
            if now-last>.02:
                last=now;stamp=node.get_clock().now().to_msg()
                transforms=[]
                for f in ["lidar_link","laser_link",*frames]:
                    if missing and f==target:continue
                    t=TransformStamped();t.header.stamp=stamp;t.header.frame_id="base_link";t.child_frame_id=f;t.transform.rotation.w=1.
                    if f in ("lidar_link","laser_link"): pass
                    elif f==target:t.transform.translation.x=position;t.transform.translation.z=-vz
                    elif f=="held_object":t.transform.translation.x=2.5;t.transform.translation.z=-vz
                    else:t.transform.translation.x=20.
                    transforms.append(t)
                tf.sendTransform(transforms)
            rclpy.spin_once(node,timeout_sec=.01)
        def until(pred,timeout=10):
            end=time.monotonic()+timeout
            while not pred() and time.monotonic()<end:tick()
            self.assertTrue(pred(),str(states[-4:]))
        try:
            # Wait for model construction and an established TF history.
            until(lambda:cp.get_subscription_count()>0 and sp.get_subscription_count()>0,60.)
            start=time.monotonic()
            while time.monotonic()<start+.8:tick()
            captured=node.get_clock().now()
            x,y=1.+vx,vy
            # Keep the original pose through the midpoint of its 5 ms bin.
            start=time.monotonic()
            while time.monotonic()<start+.07:tick()
            # Let the gripper move; the subsequently delivered sensor data
            # still refers to the earlier pose, not the newest TF.
            position=1.5;start=time.monotonic()
            while time.monotonic()<start+.35:tick()
            msg=PointCloud2();msg.header.frame_id="lidar_link";msg.header.stamp=captured.to_msg()
            msg.height=1;msg.width=6;msg.is_bigendian=False;msg.is_dense=True;msg.point_step=20;msg.row_step=120
            msg.fields=[PointField(name="x",offset=0,datatype=7,count=1),PointField(name="y",offset=4,datatype=7,count=1),
                PointField(name="z",offset=8,datatype=7,count=1),PointField(name="timestamp",offset=12,datatype=8,count=1)]
            ts=float(captured.nanoseconds)
            # Third point was acquired 150 ms later after the gripper moved.
            msg.data=struct.pack("<"+"fffd"*6,x,y,0.,ts,x,y,.40,ts,
                1.5+vx,y,0.,ts+150000000.,2.5+vx,y,0.,ts,
                .15,-.35,.73,ts,.15,-.35,.80,ts)
            scan=LaserScan();scan.header.frame_id="laser_link";scan.header.stamp=captured.to_msg()
            scan.angle_min=math.atan2(y,x);scan.angle_max=scan.angle_min
            scan.angle_increment=.001;scan.time_increment=.0001;scan.scan_time=.001
            scan.range_min=.01;scan.range_max=10.;scan.ranges=[math.hypot(x,y)]
            cp.publish(msg);sp.publish(scan)
            until(lambda:bool(clouds) and bool(scans),8.)
            first=struct.unpack_from("<fff",clouds[-1].data,0)
            second=struct.unpack_from("<fff",clouds[-1].data,20)
            self.assertTrue(all(math.isnan(v) for v in first),first)
            self.assertTrue(all(math.isfinite(v) for v in second),second)
            third=struct.unpack_from("<fff",clouds[-1].data,40)
            self.assertTrue(all(math.isnan(v) for v in third),third)
            fourth=struct.unpack_from("<fff",clouds[-1].data,60)
            self.assertTrue(all(math.isnan(v) for v in fourth),fourth)
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,80)[0]), "observed cluster was not filtered")
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,100)[0]), "nearby return outside box was removed")
            until(lambda:any(json.loads(x).get("additional_meshes")==1 for x in states),2.)
            self.assertTrue(math.isinf(scans[-1].ranges[0]),scans[-1].ranges)
            msg.width=2;msg.row_step=40
            # Pause only one unchanged part TF for a short time. The filter
            # must emit an explicitly degraded, temporally bounded result
            # instead of dropping the sensor frame.
            missing=True;start=time.monotonic()
            while time.monotonic()<start+.04:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg()
            x=1.5+vx;ts=float(captured.nanoseconds)
            msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts)
            cp.publish(msg)
            until(lambda:bool(clouds) and any(json.loads(x).get("reason")=="stable_tf_fallback"
                for x in states),4.)
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,0)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,20)[0]))
            previous_stamp=clouds[-1].header.stamp
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:bool(clouds) and any(json.loads(x).get("reason")==
                "reused_last_filtered_original_stamp" for x in states),2.)
            self.assertEqual(clouds[0].header.stamp,previous_stamp)
            # A much older transform cannot be used as an indefinitely
            # renewable cache, even if the part appears stationary.
            start=time.monotonic()
            while time.monotonic()<start+.20:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:any(json.loads(x).get("reason")=="tf_wait_expired" for x in states),3.)
            self.assertFalse(clouds,"expired pose must not publish a filtered frame")
            missing=False
            start=time.monotonic()
            while time.monotonic()<start+.3:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:any(c.header.stamp==captured.to_msg() for c in clouds),4.)
            last_good_stamp=captured.to_msg()
            # If the most recent history contains material movement, it is
            # not safe to reuse that last pose for a newer unknown instant.
            position=2.;last_before=last
            until(lambda:last>last_before,1.)
            missing=True;start=time.monotonic()
            while time.monotonic()<start+.04:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            x=2.+vx;msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:any(json.loads(x).get("reason")=="tf_wait_expired" for x in states),3.)
            self.assertTrue(all(c.header.stamp==last_good_stamp for c in clouds),
                "moving pose cannot produce a freshly stamped fallback")
            self.assertFalse(any(name=="/cmd_vel" for name,_ in node.get_topic_names_and_types()))
        finally:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
            log.seek(0)
            if proc.returncode not in (0,-2):print(log.read())
            log.close();node.destroy_node();rclpy.shutdown()

if __name__=="__main__":unittest.main()
