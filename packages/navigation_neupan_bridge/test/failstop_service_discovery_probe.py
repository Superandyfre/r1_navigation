#!/usr/bin/env python3
"""Isolated DDS service discovery probe; no navigation or driver processes."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

SERVICE='/test_failstop_comm/discovery_ping'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server',action='store_true')
    parser.add_argument('--report',required=True,type=Path)
    parser.add_argument('--count',type=int,default=60)
    args=parser.parse_args()
    assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST')=='1'
    assert os.environ.get('ROS_DOMAIN_ID')=='192'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert 1<=args.count<=200
    rclpy.init();node=Node('test_service_discovery_server' if args.server else 'test_service_discovery_client')
    if args.server:
        rows=[];stopping=False
        def stop(*unused):
            nonlocal stopping
            stopping=True
        signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
        def respond(request,response):
            rows.append(dict(event='callback',monotonic=time.monotonic()))
            response.success=True;response.message=str(len(rows));return response
        service=node.create_service(Trigger,SERVICE,respond)
        try:
            while not stopping:
                try:rclpy.spin_once(node,timeout_sec=.02)
                except Exception as exc:rows.append(dict(event='spin_error',monotonic=time.monotonic(),error=repr(exc)))
        finally:
            args.report.write_text(json.dumps(rows,indent=2)+'\n')
            node.destroy_node();rclpy.shutdown()
        return 0
    args.report.mkdir(parents=True,exist_ok=False)
    log=open(args.report/'server.log','w')
    proc=subprocess.Popen([sys.executable,__file__,'--server','--report',str(args.report/'server-events.json')],stdout=log,stderr=subprocess.STDOUT)
    rows=[]
    try:
        for index in range(args.count):
            client=node.create_client(Trigger,SERVICE)
            created=time.monotonic();deadline=created+5.
            while not client.service_is_ready() and time.monotonic()<deadline:
                assert proc.poll() is None
                rclpy.spin_once(node,timeout_sec=.005)
            ready=time.monotonic();assert client.service_is_ready(),'service never ready'
            # Alternate immediate-ready call vs a diagnostic 0.25s discovery
            # observation. This changes only probe timing, not production QoS.
            settle=.25 if index%2 else 0.
            while time.monotonic()<ready+settle:rclpy.spin_once(node,timeout_sec=.005)
            sent=time.monotonic();future=client.call_async(Trigger.Request())
            while not future.done() and time.monotonic()<sent+2.:
                assert proc.poll() is None
                rclpy.spin_once(node,timeout_sec=.005)
            rows.append(dict(index=index,created=created,ready=ready,sent=sent,ended=time.monotonic(),
                settle_seconds=settle,completed=future.done(),
                response=None if not future.done() else future.result().message))
            node.destroy_client(client)
            (args.report/'client-events.json').write_text(json.dumps(rows,indent=2)+'\n')
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
        log.close();node.destroy_node();rclpy.shutdown()
    print(json.dumps(dict(report=str(args.report),requests=len(rows),timeouts=sum(not row['completed'] for row in rows),
        physical_motion=False,navigation_processes=False)),flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
