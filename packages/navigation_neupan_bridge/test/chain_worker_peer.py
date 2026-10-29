#!/usr/bin/env python3
"""Actual N116Node scheduler/TF/scan/context/source; only IPC solver replaced.

Domain192 test-only peer. Never imports/runs a hardware driver. This is not a
real NeuPAN optimization/model acceptance test.
"""
import json
import os
from pathlib import Path
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from r1_neupan_integration.n1_16_node import N116Node
from r1_neupan_integration.n1_16_bridge import BridgeProtocolError


def main():
    assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '192'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node=N116Node()
    # Isolated instrumentation: do not allocate an extra health snapshot or
    # refresh its source timestamp. Buffer real publication attempts and flush
    # only at orderly teardown, avoiding per-heartbeat disk IO.
    health_sends=[]
    original_health_pub=node.worker_health_pub
    class HealthRecorder:
        def publish(self,message):
            started=time.monotonic()
            payload=json.loads(message.data)
            row=dict(started=started,payload=payload,published=False)
            try:
                original_health_pub.publish(message)
                row['published']=True
            except Exception as exc:
                row['error']=repr(exc)
                raise
            finally:
                row['ended']=time.monotonic()
                health_sends.append(row)
    node.worker_health_pub=HealthRecorder()
    fatal=threading.Event()
    counter=0
    def startup(*args,**kwargs):
        node.client.health.ready(node.client.worker_generation)
    def solve(state,points,path,version,scan,stamp,**kwargs):
        nonlocal counter
        if fatal.is_set():
            raise BridgeProtocolError('worker_response_failure','test fake IPC failure')
        counter+=1
        dispatched=time.monotonic()
        opt=np.repeat(state,3,axis=1)
        opt[0,:]+=np.array([0.,.1,.2])
        outputs=dict(opt_state=opt,ref_state=opt.copy(),action=np.array([[.1],[0.]]))
        record=dict(worker_generation=node.client.worker_generation,request_id=counter,
            cycle_id=kwargs['cycle_id'],scan_counter=scan,info={},timings=dict(
            request_dispatched_monotonic=dispatched,worker_solve_end_monotonic=time.monotonic()))
        return outputs,record
    real_solver=os.environ.get('NEUPAN_CHAIN_WORKER')=='actual-solver'
    if not real_solver:
        node.client.startup=startup
        node.client.solve=solve
    source_paused=threading.Event()
    original_source_pub=node.planner_result_pub
    class SourceGate:
        def publish(self, message):
            if not source_paused.is_set():original_source_pub.publish(message)
    node.planner_result_pub=SourceGate()
    ack=node.create_publisher(String,'/test_failstop_chain/inject_ack',20)
    def inject(msg):
        request=json.loads(msg.data)
        if 'hold_source' in request:
            if request['hold_source']:source_paused.set()
            else:source_paused.clear()
            ack.publish(String(data=json.dumps(request)))
            return
        if real_solver:
            process=node.client.process
            if process is not None and process.is_alive():process.kill()
        else:fatal.set()
    sub=node.create_subscription(String,'/test_failstop_chain/inject',inject,20)
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    stopping=threading.Event()
    signal.signal(signal.SIGINT,lambda *args:stopping.set())
    signal.signal(signal.SIGTERM,lambda *args:stopping.set())
    try:
        while not stopping.is_set():executor.spin_once(timeout_sec=.05)
    finally:
        node.finalize();executor.shutdown(timeout_sec=2)
        output=Path(str(node.get_parameter('n115_output_dir').value))
        output.mkdir(parents=True,exist_ok=True)
        (output/'health-send-trace.json').write_text(json.dumps(dict(
            initial_constructor_publish_recorded=False,records=health_sends),indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
