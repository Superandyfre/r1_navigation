#!/usr/bin/env python3
"""Forward actual FollowPath send_goal with its UUID; delay only its response."""
import json
import os
from pathlib import Path
import signal
import sys
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import FollowPath

assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST')=='1'
assert os.environ.get('ROS_DOMAIN_ID')=='192' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
report=Path(sys.argv[1]);report.mkdir(exist_ok=False)
rclpy.init();node=Node('test_delayed_follow_path_ack')
group=ReentrantCallbackGroup();events=[]
client=node.create_client(FollowPath.Impl.SendGoalService,'/follow_path/_action/send_goal',callback_group=group)
def save(): (report/'events.json').write_text(json.dumps(events,indent=2)+'\n')
def forward(request,response):
    event=dict(received=time.monotonic(),uuid=bytes(request.goal_id.uuid).hex());events.append(event);save()
    if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('actual FollowPath unavailable')
    future=client.call_async(request)
    deadline=time.monotonic()+4.
    while not future.done() and time.monotonic()<deadline:time.sleep(.005)
    if not future.done():raise RuntimeError('actual send_goal response timeout')
    actual=future.result();event['actual_response']=time.monotonic();event['accepted']=actual.accepted;save()
    time.sleep(2.)
    event['delayed_response']=time.monotonic();save()
    return actual
service=node.create_service(FollowPath.Impl.SendGoalService,'/test_failstop_ack/follow_path/_action/send_goal',forward,callback_group=group)
executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);stopping=False
def stop(*unused):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
try:
    while not stopping:executor.spin_once(timeout_sec=.05)
finally:
    executor.shutdown(timeout_sec=5.);save();node.destroy_node();rclpy.shutdown()
