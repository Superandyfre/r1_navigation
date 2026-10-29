#!/usr/bin/env python3
"""Run preserved legacy ROS adapter only in isolated compatibility tests."""
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
import r1_neupan_integration.n1_16_node as legacy

def main():
    assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST')=='1'
    assert os.environ.get('ROS_DOMAIN_ID')=='192'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    source=Path(legacy.__file__).resolve()
    assert str(source).startswith('/home/rpp/rpp_ws/src/application/neupan_migration/')
    rclpy.init();node=legacy.N116Node()
    directory=Path(node.get_parameter('n115_output_dir').value);directory.mkdir(parents=True,exist_ok=True)
    (directory/'legacy-import.json').write_text(json.dumps(dict(source=str(source),
        sha256=hashlib.sha256(source.read_bytes()).hexdigest(),has_health_publisher=hasattr(node,'worker_health_pub')),
        indent=2))
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *args:stop.set())
    signal.signal(signal.SIGTERM,lambda *args:stop.set())
    try:
        while not stop.is_set():executor.spin_once(timeout_sec=.05)
    finally:
        node.finalize();executor.shutdown(timeout_sec=2);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
