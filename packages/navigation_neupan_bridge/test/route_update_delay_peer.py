#!/usr/bin/env python3
"""Forward a real route update and delay only its response in an isolated ROS domain."""
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from r1_neupan_runtime.srv import UpdatePath

assert os.environ.get("NEUPAN_ISOLATED_ROS_TEST") == "1"
assert os.environ.get("ROS_DOMAIN_ID") == "192" and os.environ.get("ROS_LOCALHOST_ONLY") == "1"
report=Path(sys.argv[1]); report.mkdir(exist_ok=False)
rclpy.init(); node=Node("test_delayed_route_ack")
group=ReentrantCallbackGroup(); lock=threading.Lock(); events=[]
client=node.create_client(UpdatePath, "/update_follow_path", callback_group=group)
def save(name, obj):
    temporary=report/(name+".tmp")
    temporary.write_text(json.dumps(obj,indent=2)+"\n")
    temporary.replace(report/name)
def forward(request, response):
    event=dict(sequence=request.revision_sequence, received=time.monotonic())
    with lock: events.append(event);save("events.json",events)
    if not client.wait_for_service(timeout_sec=3.): raise RuntimeError("route service unavailable")
    future=client.call_async(request);deadline=time.monotonic()+3.
    while not future.done() and time.monotonic()<deadline: time.sleep(.005)
    if not future.done(): raise RuntimeError("forwarded route response timed out")
    actual=future.result()
    with lock:
        event.update(accepted=actual.accepted,actual_response=time.monotonic())
        save("events.json",events)
    time.sleep(.85)
    with lock:
        event["delayed_response"]=time.monotonic()
        save("events.json",events)
    return actual
service=node.create_service(UpdatePath,"/test_failstop_chain/delayed_route",forward,callback_group=group)
executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);stopping=False
def stop(*unused):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
try:
    ready=False
    while not stopping:
        executor.spin_once(timeout_sec=.05)
        if not ready and client.service_is_ready():
            save("ready.json",dict(monotonic=time.monotonic()))
            ready=True
finally:
    executor.shutdown(timeout_sec=5.);node.destroy_node();rclpy.shutdown()
