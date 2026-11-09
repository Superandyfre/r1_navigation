"""Read-only ROS bag replay of timestamped arm snapshot production."""
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String

assert os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1"
assert os.environ.get("ROS_DOMAIN_ID")=="197" and os.environ.get("ROS_LOCALHOST_ONLY")=="1"
bag=os.environ["NEUPAN_ARM_REPLAY_BAG"]
exe=os.environ["NEUPAN_TEST_ARM_FOOTPRINT"]
output=Path(os.environ["NEUPAN_ARM_REPLAY_REPORT"])
assert not output.exists()
rclpy.init()
node=rclpy.create_node("arm_snapshot_bag_observer")
snapshots=[];statuses=[]
node.create_subscription(String,"/r1_arm_geometry_snapshot",lambda m:snapshots.append(json.loads(m.data)),20)
node.create_subscription(String,"/r1_arm_footprint/status",lambda m:statuses.append(json.loads(m.data)),20)
log=tempfile.TemporaryFile(mode="w+")
proc=subprocess.Popen([exe,"--ros-args","-p","use_sim_time:=true","-p","mesh_projection_frequency:=4.0"],
                      stdout=log,stderr=subprocess.STDOUT)
player=None
try:
    deadline=time.monotonic()+30.
    while node.count_publishers("/r1_arm_geometry_snapshot")==0 and time.monotonic()<deadline:
        if proc.poll() is not None: raise RuntimeError("arm projection exited")
        rclpy.spin_once(node,timeout_sec=.05)
    if node.count_publishers("/r1_arm_geometry_snapshot")==0: raise RuntimeError("arm projection did not start")
    player=subprocess.Popen(["ros2","bag","play",bag,"--clock","100","--disable-keyboard-controls",
        "--topics","/tf","/tf_static","/left_arm/joint_states","/right_arm/joint_states"],
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    while player.poll() is None:
        if proc.poll() is not None: raise RuntimeError("arm projection exited")
        rclpy.spin_once(node,timeout_sec=.03)
    if player.returncode: raise RuntimeError("bag player failed")
    deadline=time.monotonic()+1.
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.03)
    stamps=[s["stamp_ns"] for s in snapshots]
    components=[len(s["components"]) for s in snapshots]
    result=dict(snapshot_count=len(snapshots),first_stamp_ns=min(stamps,default=None),
        last_stamp_ns=max(stamps,default=None),min_components=min(components,default=0),
        max_components=max(components,default=0),
        monotonic_stamps=all(a<b for a,b in zip(stamps,stamps[1:])),
        status_reasons=sorted(set(s.get("reason","") for s in statuses)))
    output.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result),flush=True)
finally:
    if player and player.poll() is None:player.send_signal(signal.SIGINT);player.wait(timeout=5)
    proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
    log.seek(0)
    if proc.returncode not in (0,-2):print(log.read())
    log.close();node.destroy_node();rclpy.shutdown()
