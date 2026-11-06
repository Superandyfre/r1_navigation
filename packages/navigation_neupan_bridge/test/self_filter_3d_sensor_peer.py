#!/usr/bin/env python3
"""Synthetic dual sensors and arm TF for real solver integration, no hardware."""
import os,signal,struct,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from tf2_ros import TransformBroadcaster
assert os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1"
assert os.environ.get("ROS_DOMAIN_ID")=="192" and os.environ.get("ROS_LOCALHOST_ONLY")=="1"
flag=Path(sys.argv[1]);rclpy.init();node=Node("synthetic_filter_sensor_peer")
tf=TransformBroadcaster(node)
cp=node.create_publisher(PointCloud2,"/lidar/pointcloud",qos_profile_sensor_data)
sp=node.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
frames=[]
for arm in ("left_","right_"):frames += [arm+"base"]+[arm+"link"+str(i) for i in range(1,8)]
for arm,g in (("left_","L_"),("right_","R_")):
    frames += [arm+"moduan_link",g+"omniPicker_base_link"]
    frames += [g+"omniPicker_hand_"+s for s in ("narrow1_Link","narrow2_Link","narrow3_Link","narrow_loop_Link","wide1_Link","wide2_Link","wide3_Link","wide_loop_Link")]
target="L_omniPicker_hand_narrow1_Link";stopping=False;last_tf=last_scan=0.
def stop(*unused):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
try:
    while not stopping:
        now=time.monotonic();stamp=node.get_clock().now()
        if now-last_tf>=.02:
            last_tf=now;messages=[]
            for frame in ["lidar_link","laser_link",*frames]:
                if flag.exists() and frame==target:continue
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
        rclpy.spin_once(node,timeout_sec=.003)
finally:node.destroy_node();rclpy.shutdown()
