#!/usr/bin/env python3
"""Actual N116Node scheduler/TF/scan/context/source; only IPC solver replaced.

Domain192 test-only peer. Never imports/runs a hardware driver. This is not a
real NeuPAN optimization/model acceptance test.
"""
import json
import os
from collections import deque
from pathlib import Path
import signal
import sys
import threading
import time
import traceback

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
    # Diagnostic-only, bounded startup evidence. Do not acquire state_lock,
    # change inputs, refresh protocol times, or trigger a planner cycle.
    startup_events=deque(maxlen=1024)
    startup_samples=deque(maxlen=512)
    startup_stacks=deque(maxlen=16)
    diagnostic_stop=threading.Event()
    def trace_call(owner,name):
        original=getattr(owner,name)
        def traced(*args,**kwargs):
            entered=time.monotonic()
            startup_events.append(dict(method=name,event='enter',monotonic=entered))
            try:
                result=original(*args,**kwargs)
                startup_events.append(dict(method=name,event='return',monotonic=time.monotonic(),
                    result=result if isinstance(result,bool) else None))
                return result
            except BaseException as exc:
                startup_events.append(dict(method=name,event='raise',monotonic=time.monotonic(),
                    error=repr(exc)))
                raise
        setattr(owner,name,traced)
    trace_call(node,'_robot_transform')
    robot_tf_paused=threading.Event()
    traced_robot_transform=node._robot_transform
    def gated_robot_transform():
        if robot_tf_paused.is_set():return False
        return traced_robot_transform()
    node._robot_transform=gated_robot_transform
    trace_call(node.client,'startup')
    trace_call(node.client,'solve')
    def sample_startup():
        next_stack=time.monotonic()+2.
        while not diagnostic_stop.wait(.25):
            now=time.monotonic()
            startup_samples.append(dict(monotonic=now,
                scheduler=node._planner_scheduler.snapshot(),
                active_request=node._active_request,scan_counter=node.scan_counter,
                last_processed_scan=node.last_processed_scan,
                tf_stamp_sec=getattr(node,'tf_stamp_sec',None),
                context_active=node._execution_store.active is not None,
                bridge_failed=node._bridge_failed))
            if now>=next_stack:
                next_stack=now+5.
                frames=sys._current_frames()
                startup_stacks.append(dict(monotonic=now,threads=[dict(
                    name=t.name,ident=t.ident,stack=traceback.format_stack(frames[t.ident]))
                    for t in threading.enumerate() if t.ident in frames and
                    t is not threading.current_thread()]))
    diagnostic_thread=threading.Thread(target=sample_startup,name='test-startup-evidence',daemon=True)
    diagnostic_thread.start()
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
        if 'hold_robot_tf' in request:
            if request['hold_robot_tf']:robot_tf_paused.set()
            else:robot_tf_paused.clear()
            ack.publish(String(data=json.dumps(request)))
            return
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
        diagnostic_stop.set();diagnostic_thread.join(timeout=2)
        node.finalize();executor.shutdown(timeout_sec=2)
        output=Path(str(node.get_parameter('n115_output_dir').value))
        output.mkdir(parents=True,exist_ok=True)
        (output/'health-send-trace.json').write_text(json.dumps(dict(
            initial_constructor_publish_recorded=False,records=health_sends),indent=2))
        (output/'startup-trace.json').write_text(json.dumps(dict(
            diagnostic_only=True,samples_are_unsynchronized_observations=True,
            constructor_not_instrumented=True,
            events=list(startup_events),samples=list(startup_samples),
            stacks=list(startup_stacks)),indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
