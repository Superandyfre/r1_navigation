#!/usr/bin/env python3
"""Actual ControllerServer cleanup destroys plugin instances; no robot driver."""
import json
from pathlib import Path
import time
import unittest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import failstop_chain_ros as chain


class PluginRecreate(chain.FailstopChainROS):
    def exercise_recreate(self, fault):
        handle=self.goal(); result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done(),'chain did not become executable')
        boot=next(j['controller_boot_id'] for _,j in reversed(self.runtime_statuses)
                  if j.get('schema')=='r1_neupan_execution_status_v1')
        worker_boot=self.healths[-1][1]['boot_id']
        guard_boot=self.statuses[-1][1]['guard_boot_id']
        if fault=='worker_fatal':self.fatal=True
        else:self.source_enabled=False
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
        terminal=time.monotonic()
        # Keep the Worker alive and stop repeated fault injection. For Source
        # starvation, restore scans: fresh health/data cannot reset the run.
        self.fatal=False; self.source_enabled=True
        controller=next(p for p in self.procs if Path(p.args[0]).name==chain.CONTROLLER.name)
        pid=controller.pid
        self.lifecycle('controller_server',4)
        self.lifecycle('controller_server',2)
        self.action.destroy()
        self.lifecycle('controller_server',1)
        self.lifecycle('controller_server',3)
        self.action=ActionClient(self.node,FollowPath,'/follow_path')
        self.until(self.action.server_is_ready)
        self.until(lambda:any(j.get('schema')=='r1_neupan_execution_status_v1' and
                             j['controller_boot_id']!=boot for t,j in self.runtime_statuses if t>terminal))
        new_boot=next(j['controller_boot_id'] for _,j in reversed(self.runtime_statuses)
                      if j.get('schema')=='r1_neupan_execution_status_v1')
        self.assertEqual(controller.pid,pid)
        self.assertIsNone(controller.poll())
        self.assertEqual(self.healths[-1][1]['boot_id'],worker_boot)
        self.assertEqual(self.statuses[-1][1]['guard_boot_id'],guard_boot)
        # The new object has no old in-memory ControllerExecution. Distributed
        # Guard/Worker fault evidence must still prevent a new arm and action.
        arm=self.node.create_client(Trigger,chain.PREFIX+'arm')
        self.until(arm.service_is_ready)
        response=self.future(arm.call_async(Trigger.Request()))
        self.assertFalse(response.success)
        retry=self.goal()
        self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        end=time.monotonic()+.5
        self.until(lambda:time.monotonic()>end)
        rows=[r for r in self.outputs if r[0]>terminal+.1]
        self.assertGreater(len(rows),10)
        self.assertTrue(all(v==0. and w==0. for _,v,w in rows))
        if fault=='source_starvation':
            self.assertFalse(any(j.get('state')=='FAULT_LATCHED' for _,j in self.healths),
                             'must test denial without a Worker FATAL')
        (self.report/'plugin-recreate-result.json').write_text(json.dumps(dict(
            fault=fault,old_controller_boot=boot,new_controller_boot=new_boot,controller_pid=pid,
            same_server_process=True,worker_boot_unchanged=True,guard_boot_unchanged=True,
            old_action='ABORTED',new_action='ABORTED',arm_denied=True,
            continuous_zero_samples=len(rows),physical_motion=False,worker=self.worker_mode,
            shared_library_dlclose_tested=False),indent=2))

    def test_worker_fatal(self):self.exercise_recreate('worker_fatal')
    def test_source_starvation(self):self.exercise_recreate('source_starvation')


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(PluginRecreate(name) for name in ['test_worker_fatal','test_source_starvation'])

if __name__=='__main__':unittest.main()
