#!/usr/bin/env python3
"""Read-only rosbag replay into an isolated ROS domain; shadow output only."""
import json,os,signal,statistics,subprocess,tempfile,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1"
assert os.environ.get("ROS_DOMAIN_ID")=="197" and os.environ.get("ROS_LOCALHOST_ONLY")=="1"
exe=os.environ["NEUPAN_TEST_3D_SHADOW"];bag=os.environ["NEUPAN_3D_REPLAY_BAG"];output=Path(os.environ["NEUPAN_3D_REPLAY_REPORT"])
assert not output.exists()
rclpy.init();node=rclpy.create_node("self_filter_3d_replay_observer");rows=[];fused=[]
node.create_subscription(LaserScan,"/test_3d_fused",lambda msg:fused.append(time.monotonic()),qos_profile_sensor_data)
node.create_subscription(String,"/r1_self_filter_3d_shadow/status",lambda msg:rows.append(json.loads(msg.data)),50)
log=tempfile.TemporaryFile(mode="w+")
shadow=subprocess.Popen([exe,"--ros-args","-p","use_sim_time:=true"],stdout=log,stderr=subprocess.STDOUT)
extra=[]
if os.environ.get("NEUPAN_3D_REPLAY_FUSION")=="1":
    commands=[
      [os.environ["NEUPAN_TEST_CLOUD_TO_SCAN"],"--ros-args","--params-file",os.environ["NEUPAN_TEST_CLOUD_CONFIG"],
       "-p","use_sim_time:=true","-r","cloud_in:=/r1_self_filter_3d_shadow/cloud","-r","scan:=/test_3d_front"],
      [os.environ["NEUPAN_TEST_SCAN_FUSION"],"--ros-args","--params-file",os.environ["NEUPAN_TEST_FUSION_CONFIG"],
       "-p","use_sim_time:=true","-p","livox_scan_topic:=/test_3d_front",
       "-p","rear_scan_topic:=/r1_self_filter_3d_shadow/rear_scan","-p","output_scan_topic:=/test_3d_fused",
       "-p","status_topic:=/test_3d_fusion_status"]]
    extra=[subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL) for cmd in commands]
player=None
def tick():
    if shadow.poll() is not None:raise RuntimeError("shadow process exited")
    if any(p.poll() is not None for p in extra):raise RuntimeError("fusion component exited")
    rclpy.spin_once(node,timeout_sec=.03)
try:
    end=time.monotonic()+60
    while node.count_publishers("/r1_self_filter_3d_shadow/status")==0 and time.monotonic()<end:tick()
    if node.count_publishers("/r1_self_filter_3d_shadow/status")==0:raise RuntimeError("shadow startup timeout")
    player=subprocess.Popen(["ros2","bag","play",bag,"--clock","100","--disable-keyboard-controls",
        "--topics","/tf","/tf_static","/lidar/pointcloud","/scan"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    while player.poll() is None:tick()
    end=time.monotonic()+2
    while time.monotonic()<end:tick()
    if player.returncode:raise RuntimeError("rosbag player failed: "+str(player.returncode))
    summary={}
    for name in ("front_cloud","rear_scan"):
        data=[r for r in rows if r.get("sensor")==name]
        good=[r for r in data if r.get("state")=="READY"]
        fallback=[r for r in data if r.get("reason") in ("stable_tf_fallback","reused_last_filtered_original_stamp") and r.get("output_published")]
        bad=[r for r in data if not r.get("output_published",False)]
        emitted=good+fallback
        ms=sorted(r["processing_ms"] for r in emitted)
        summary[name]=dict(total=len(data),ready=len(good),fallback=len(fallback),not_published=len(bad),
            reasons={reason:sum(r["reason"]==reason for r in bad) for reason in sorted({r["reason"] for r in bad})},
            p50_ms=statistics.median(ms) if ms else None,
            p95_ms=ms[min(len(ms)-1,int(.95*len(ms)))] if ms else None,
            max_ms=max(ms) if ms else None,
            max_queue_age_sec=max((r["queue_age_sec"] for r in data),default=None),
            max_dropped=max((r["dropped_frames"] for r in data),default=0),
            filtered_points=sum(r["filtered_points"] for r in emitted))
    steady=fused[2:-2] if len(fused)>4 else []
    gaps=[b-a for a,b in zip(steady,steady[1:])]
    fusion=dict(samples=len(fused),steady_samples=len(steady),max_gap_sec=max(gaps,default=None))
    report=dict(schema="r1_filter_3d_replay_result_v1",bag=bag,physical_motion=False,fusion=fusion,
        non_closed_meshes=rows[-1].get("open_mesh_frames",[]) if rows else [],
        summary=summary,statuses=rows)
    output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!="statuses"},indent=2))
finally:
    if player and player.poll() is None:player.send_signal(signal.SIGINT);player.wait(timeout=5)
    for proc in extra:
        if proc.poll() is None:proc.send_signal(signal.SIGINT)
    for proc in extra:
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
    shadow.send_signal(signal.SIGINT)
    try:shadow.wait(timeout=8)
    except subprocess.TimeoutExpired:shadow.kill();shadow.wait(timeout=3)
    log.seek(0)
    if shadow.returncode not in (0,-2):print(log.read())
    log.close();node.destroy_node();rclpy.shutdown()
