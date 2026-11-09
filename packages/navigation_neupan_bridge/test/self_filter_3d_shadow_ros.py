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
        target="L_omniPicker_hand_narrow1_Link";target_right="R_omniPicker_hand_narrow1_Link";frames.append("held_object")
        exe=os.environ["NEUPAN_TEST_3D_SHADOW"];log=tempfile.TemporaryFile(mode="w+")
        uri="package://rpp_components_description/meshes/gripper/agibot/omniPicker/narrow1_Link.STL"
        calibration=tempfile.NamedTemporaryFile(mode="w",suffix=".json")
        json.dump({"schema":"confirmed_livox_link_echo_v1","templates":[
            {"frame":target,"sensor_frame":"lidar_link","radius_m":.008,"points_m":[[.3,0.,.3],[.3,0.,vz]]},
            {"frame":target_right,"sensor_frame":"lidar_link","radius_m":.008,"points_m":[[.3,.2,.3]]}]},calibration);calibration.flush()
        proc=subprocess.Popen([exe,"--ros-args","-p","confirmed_livox_echo_file:="+calibration.name,
            "-p","additional_mesh_frames:=[held_object]",
            "-p","additional_mesh_uris:=["+uri+"]","-p","additional_mesh_scales:=[1.0]"],
            stdout=log,stderr=subprocess.STDOUT)
        position=1.;right_position=2.;left_yaw=right_yaw=0.;missing=False;last=0.
        def tick():
            nonlocal last
            self.assertIsNone(proc.poll(),"shadow process exited")
            now=time.monotonic()
            if now-last>.02:
                last=now;stamp=node.get_clock().now().to_msg()
                transforms=[]
                for f in ["lidar_link","other_lidar_link","laser_link",*frames]:
                    if missing and f==target:continue
                    t=TransformStamped();t.header.stamp=stamp;t.header.frame_id="base_link";t.child_frame_id=f;t.transform.rotation.w=1.
                    if f in ("lidar_link","other_lidar_link","laser_link"): pass
                    elif f==target:
                        t.transform.translation.x=position;t.transform.translation.z=-vz
                        t.transform.rotation.z=math.sin(left_yaw/2);t.transform.rotation.w=math.cos(left_yaw/2)
                    elif f==target_right:
                        t.transform.translation.x=right_position;t.transform.translation.z=-vz
                        t.transform.rotation.z=math.sin(right_yaw/2);t.transform.rotation.w=math.cos(right_yaw/2)
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
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,80)[0]), "legacy base-frame blind box must not filter a real point")
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,100)[0]), "nearby return outside box was removed")
            until(lambda:any(json.loads(x).get("additional_meshes")==1 for x in states),2.)
            self.assertTrue(math.isinf(scans[-1].ranges[0]),scans[-1].ranges)
            until(lambda:any(json.loads(x).get("removed_by_mesh",{}).get(target,0)>=2
                for x in states),2.)
            matching=[json.loads(x) for x in states if json.loads(x).get("sensor")=="front_cloud"
                and json.loads(x).get("removed_by_mesh",{}).get(target,0)>=2]
            entries=matching[-1]["removed_sample"]
            self.assertTrue(any(e["mesh_frame"]==target and e["index"]==0 for e in entries))
            self.assertTrue(any(e["mesh_frame"]==target and e["index"]==2 and
                e["point_stamp_ns"]>entries[0]["point_stamp_ns"] for e in entries))
            self.assertTrue(any(e["mesh_frame"]=="held_object" for e in entries))
            # Calibrated self return follows the moving gripper, while a nearby
            # obstacle and the same old world coordinate remain unchanged.
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.width=3;msg.row_step=60
            msg.data=struct.pack("<"+"fffd"*3,1.8,0.,.3-vz,ts,1.3,0.,.3-vz,ts,1.82,0.,.3-vz,ts)
            clouds.clear();states.clear();cp.publish(msg)
            until(lambda:bool(clouds),4.)
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,0)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,20)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,40)[0]))
            until(lambda:any(any(e.get("reason")=="confirmed_link_echo" for e in json.loads(x).get("removed_sample",[])) for x in states),2.)
            scans.clear();scan.header.stamp=captured.to_msg();scan.angle_min=0.;scan.angle_max=0.;scan.ranges=[1.8]
            sp.publish(scan);until(lambda:bool(scans),4.)
            self.assertTrue(math.isfinite(scans[-1].ranges[0]), "Livox-only calibration must not filter rear scan")
            # The calibrated front-sensor pattern is not applied to a different sensor.
            msg.header.frame_id="other_lidar_link";msg.width=1;msg.row_step=20
            msg.data=struct.pack("<fffd",1.8,0.,.3-vz,ts)
            clouds.clear();cp.publish(msg);until(lambda:bool(clouds),4.)
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,0)[0]))
            msg.header.frame_id="lidar_link"
            # Right gripper moves independently; it is filtered from its own
            # mesh TF, and its old position is not treated as a fixed mask.
            right_position=2.1;start=time.monotonic()
            while time.monotonic()<start+.35:tick()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.width=4;msg.row_step=80
            msg.data=struct.pack("<"+"fffd"*4,2.1+vx,y,0.,ts,2.+vx,y,0.,ts,
                2.4,.2,.3-vz,ts,2.3,.2,.3-vz,ts)
            clouds.clear();states.clear();cp.publish(msg);until(lambda:bool(clouds),4.)
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,0)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,20)[0]))
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,40)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,60)[0]))
            until(lambda:any(json.loads(q).get("removed_by_mesh",{}).get(target_right,0)>0 for q in states),2.)
            self.assertTrue(any(e.get("reason")=="confirmed_link_echo" and e.get("mesh_frame")==target_right
                for q in states for e in json.loads(q).get("removed_sample",[])))
            # Each template follows its own complete rotation, not just translation.
            left_yaw=.65;right_yaw=-.8;start=time.monotonic()
            while time.monotonic()<start+.35:tick()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            lx=1.5+.3*math.cos(left_yaw);ly=.3*math.sin(left_yaw)
            rx=2.1+.3*math.cos(right_yaw)-.2*math.sin(right_yaw)
            ry=.3*math.sin(right_yaw)+.2*math.cos(right_yaw)
            msg.width=4;msg.row_step=80
            msg.data=struct.pack("<"+"fffd"*4,lx,ly,.3-vz,ts,rx,ry,.3-vz,ts,
                1.8,0.,.3-vz,ts,2.4,.2,.3-vz,ts)
            clouds.clear();states.clear();cp.publish(msg);until(lambda:bool(clouds),4.)
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,0)[0]))
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,20)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,40)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,60)[0]))
            until(lambda:any(sum(e.get("reason")=="confirmed_link_echo" for e in
                json.loads(q).get("removed_sample",[]))>=2 for q in states),2.)
            left_yaw=right_yaw=0.;start=time.monotonic()
            while time.monotonic()<start+.35:tick()
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
            until(lambda:bool(clouds) and any(json.loads(x).get("reason")=="held_previous_pose"
                for x in states),4.)
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,0)[0]))
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,20)[0]))
            held=[json.loads(x) for x in states if json.loads(x).get("reason")=="held_previous_pose"]
            self.assertIn(target,held[-1]["held_mesh_frames"])
            self.assertGreater(held[-1]["max_pose_time_error_sec"],0.)
            self.assertEqual(held[-1]["input_stamp_ns"],held[-1]["output_stamp_ns"])
            previous_stamp=clouds[-1].header.stamp
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:any(c.header.stamp==captured.to_msg() for c in clouds) and
                  any(json.loads(x).get("reason")=="held_previous_pose" for x in states),2.)
            held=[json.loads(x) for x in states if json.loads(x).get("reason")=="held_previous_pose"]
            self.assertGreater(held[-1]["max_pose_time_error_sec"],0.)
            self.assertEqual(held[-1]["input_stamp_ns"],held[-1]["output_stamp_ns"])
            # A much older transform cannot be used as an indefinitely
            # renewable cache, even if the part appears stationary.
            start=time.monotonic()
            while time.monotonic()<start+1.60:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.width=3;msg.row_step=60
            msg.data=struct.pack("<"+"fffd"*3,x,y,0.,ts,x,y,.40,ts,2.1+vx,y,0.,ts);cp.publish(msg)
            until(lambda:bool(clouds) and any(json.loads(x).get("reason")=="partial_tf_filtered" for x in states),3.)
            self.assertEqual(clouds[-1].header.stamp,captured.to_msg())
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,0)[0]), "unknown link pose must preserve its point")
            self.assertTrue(math.isfinite(struct.unpack_from("<f",clouds[-1].data,20)[0]), "real obstacle remains")
            self.assertTrue(math.isnan(struct.unpack_from("<f",clouds[-1].data,40)[0]), "known right link is still filtered")
            partial=[json.loads(x) for x in states if json.loads(x).get("reason")=="partial_tf_filtered"]
            self.assertIn(target,partial[-1]["missing_mesh_frames"])
            self.assertEqual(partial[-1]["max_pose_time_error_sec"],-1.)
            msg.width=2;msg.row_step=40
            scans.clear(); states.clear()
            captured=node.get_clock().now();scan.header.stamp=captured.to_msg()
            scan.ranges=[math.hypot(x,y)];sp.publish(scan)
            until(lambda:bool(scans) and any(json.loads(x).get("reason")=="partial_tf_filtered" for x in states),3.)
            self.assertEqual(scans[-1].header.stamp,captured.to_msg())
            self.assertAlmostEqual(scans[-1].ranges[0],scan.ranges[0],places=6)
            missing=False
            start=time.monotonic()
            while time.monotonic()<start+.3:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:any(c.header.stamp==captured.to_msg() for c in clouds),4.)
            last_good_stamp=captured.to_msg()
            # During a short TF gap retain the last observed pose, without
            # claiming the new physical position was seen.
            position=2.;last_before=last
            until(lambda:last>last_before,1.)
            missing=True;start=time.monotonic()
            while time.monotonic()<start+.04:tick()
            clouds.clear();states.clear()
            captured=node.get_clock().now();msg.header.stamp=captured.to_msg();ts=float(captured.nanoseconds)
            x=2.+vx;msg.data=struct.pack("<fffdfffd",x,y,0.,ts,x,y,.40,ts);cp.publish(msg)
            until(lambda:any(json.loads(x).get("reason")=="held_previous_pose" for x in states),3.)
            current=[c for c in clouds if c.header.stamp==captured.to_msg()]
            self.assertTrue(current, "unknown pose must forward the original scan")
            self.assertTrue(math.isfinite(struct.unpack_from("<f",current[-1].data,0)[0]))
            self.assertFalse(any(name=="/cmd_vel" for name,_ in node.get_topic_names_and_types()))
        finally:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
            log.seek(0)
            if proc.returncode not in (0,-2):print(log.read())
            log.close();node.destroy_node();rclpy.shutdown()

if __name__=="__main__":unittest.main()
