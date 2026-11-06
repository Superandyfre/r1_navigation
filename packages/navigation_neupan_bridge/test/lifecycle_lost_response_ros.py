#!/usr/bin/env python3
"""Actual lifecycle reconciler with one delayed response, isolated ROS only."""
import os
import signal
import subprocess
import threading
import time
import unittest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State

@unittest.skipUnless(os.environ.get("NEUPAN_ISOLATED_ROS_TEST")=="1","isolated opt-in")
class LifecycleLostResponse(unittest.TestCase):
    def test_reconciles_server_state_after_late_configure_reply(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"),"196")
        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"),"1")
        rclpy.init();node=Node("lifecycle_fake_server");group=ReentrantCallbackGroup()
        state={"id":State.PRIMARY_STATE_UNCONFIGURED,"transitions":[]}
        lock=threading.Lock()
        def get(req,resp):
            with lock:resp.current_state.id=state["id"]
            return resp
        def change(req,resp):
            with lock:
                state["transitions"].append(req.transition.id)
                if req.transition.id==1 and state["id"]==State.PRIMARY_STATE_UNCONFIGURED:
                    state["id"]=State.PRIMARY_STATE_INACTIVE;delay=True;resp.success=True
                elif req.transition.id==3 and state["id"]==State.PRIMARY_STATE_INACTIVE:
                    state["id"]=State.PRIMARY_STATE_ACTIVE;delay=False;resp.success=True
                else:delay=False;resp.success=False
            if delay:time.sleep(3.4)
            return resp
        node.create_service(GetState,"/test_nav/get_state",get,callback_group=group)
        node.create_service(ChangeState,"/test_nav/change_state",change,callback_group=group)
        executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        exe=os.environ["NEUPAN_TEST_LIFECYCLE_ENSURE"]
        proc=subprocess.Popen([exe,"--ros-args","-p","node_names:=[test_nav]","-p","max_attempts:=6"],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        try:
            output,_=proc.communicate(timeout=20)
            self.assertEqual(proc.returncode,0,output)
            self.assertIn("lifecycle transition timed out",output)
            with lock:self.assertEqual(state["transitions"],[1,3])
            self.assertFalse(any(n=="/cmd_vel" for n,_ in node.get_topic_names_and_types()))
        finally:
            if proc.poll() is None:proc.send_signal(signal.SIGINT);proc.wait(timeout=5)
            executor.shutdown();thread.join(timeout=5);node.destroy_node();rclpy.shutdown()

if __name__=="__main__":unittest.main()
