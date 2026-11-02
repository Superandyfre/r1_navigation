"""Production recovery transaction over owned actual components, fake sensors."""
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped,Point32
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from r1_neupan_integration.ui7_runtime_supervisor import Supervisor
from r1_neupan_integration.runtime_recovery import OwnedProcessGroup
from failstop_restart_ros import PartialRestart
import failstop_chain_ros as chain


class ManagedRecovery(PartialRestart):
    guard_parameters_override=dict(collision_check_enabled=False)
    controller_parameters_override=dict(auto_arm_on_ready=True)

    def setUp(self):
        super().setUp()
        specs=[list(proc.args) for proc in self.procs]
        for proc in reversed(self.procs):
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=3)
            except Exception:proc.kill();proc.wait(timeout=2)
        self.procs.clear();self.action.destroy()
        # Test-only group launcher starts exactly the same four real nodes.
        # Neither this file nor the production Supervisor starts hardware.
        script=self.report/'owned-fixture.py'
        script.write_text('import os,sys,json,subprocess,time\n'
            'assert os.environ["ROS_DOMAIN_ID"]=="192"\n'
            'assert os.environ["ROS_LOCALHOST_ONLY"]=="1"\n'
            'specs=json.loads(sys.argv[1])\n'
            'for args in specs:\n'
            ' if any("controller_server" in arg for arg in args):args += ["-p","FollowPath.auto_arm_on_ready:="+sys.argv[2]]\n'
            ' if any(arg.startswith("n115_output_dir:=") for arg in args):args=[("n115_output_dir:="+sys.argv[3]) if arg.startswith("n115_output_dir:=") else arg for arg in args]\n'
            ' subprocess.Popen(args)\n'
            'while True:time.sleep(.1)\n')
        owner=self
        class FixtureSupervisor(Supervisor):
            def launch_runtime(self,auto_arm):
                self.started=time.monotonic()
                log=open(owner.report/('owned-'+str(time.time_ns())+'.log'),'w');self.logs.append(log)
                self.runtime=OwnedProcessGroup([sys.executable,str(script),json.dumps(specs),
                    str(auto_arm).lower(),str(owner.report/('managed-worker-'+str(time.time_ns())))],log,
                    environment=getattr(owner,'worker_environment',None))
                self.runtime.start()
        remap={'/cmd_vel':'output','/odom':'odom','/scan_neupan_mesh_shadow':'worker_scan',
               '/r1_arm_footprint':'footprint','/r1_localization_health':'localization',
               '/neupan_worker_health':'health','/neupan_controller_status':'diagnostics',
               '/r1_collision_guard/status':'status','/neupan_controller/arm':'arm',
               '/neupan_controller/revoke_uncertain_execution':'revoke',
               '/r1_collision_guard/stop_for_recovery':'guard_stop',
               '/neupan_runtime/recover':'recover','/neupan_runtime/cancel_recovery':'cancel_recovery'}
        args=['--ros-args','-p','report_dir:='+str(self.report/'supervisor')]
        # Guard's private service resolves under its actual node name.
        remap.pop('/r1_collision_guard/stop_for_recovery')
        for source,target in remap.items():args+=['-r',source+':='+chain.PREFIX+target]
        self.supervisor=FixtureSupervisor(cli_args=args)
        self.supervisor_executor=MultiThreadedExecutor(num_threads=4);self.supervisor_executor.add_node(self.supervisor)
        self.supervisor_thread=threading.Thread(target=self.supervisor_executor.spin);self.supervisor_thread.start()
        self.addCleanup(self.stop_supervisor)
        self.extra_pubs=dict(localization=self.node.create_publisher(String,chain.PREFIX+'localization',10),
                             footprint=self.node.create_publisher(PolygonStamped,chain.PREFIX+'footprint',10))
        self.activate_restarted('velocity_smoother');self.activate_restarted('controller_server')
        self.action=ActionClient(self.node,FollowPath,'/follow_path');self.until(self.action.server_is_ready)
        self.until(lambda:self.runtime_statuses[-1][1].get('run_armed'))

    def tick(self):
        super().tick()
        if not hasattr(self,'extra_pubs'):return
        self.extra_pubs['localization'].publish(String(data=json.dumps(dict(schema='r1_localization_health_v1',state='READY'))))
        msg=PolygonStamped();msg.header.frame_id='base_link';msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.polygon.points=[Point32(x=x,y=y,z=0.) for x,y in [(.3,.2),(.3,-.2),(-.3,-.2),(-.3,.2)]]
        self.extra_pubs['footprint'].publish(msg)

    def stop_supervisor(self):
        self.supervisor.transaction.cancelled.set()
        self.supervisor.close()
        self.supervisor_executor.shutdown(timeout_sec=3)
        self.supervisor_thread.join(timeout=3);self.supervisor.destroy_node()

    def test_fault_rebuild_waits_new_goal(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        old=dict(self.context);old_group=self.supervisor.runtime
        self.publish('inject',{'fault':'worker_crash'})
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
        client=self.node.create_client(Trigger,chain.PREFIX+'recover');self.until(client.service_is_ready)
        future=client.call_async(Trigger.Request())
        # Existing lifecycle helper in production performs these transitions.
        # The isolated fixture supplies equivalent activation explicitly.
        self.until(lambda:self.supervisor.transaction.phase in ('verify_new_idle','failed:start_new'),10)
        self.activate_restarted('velocity_smoother');self.activate_restarted('controller_server')
        answer=self.future(future,40)
        self.assertTrue(answer.success,answer.message)
        self.assertFalse(old_group.members())
        marker=time.monotonic();self.until(lambda:time.monotonic()>marker+.35)
        self.assertTrue(all(v==0. and w==0. for t,v,w in self.outputs if t>marker))
        new=self.goal();fresh=new.get_result_async()
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or fresh.done(),50)
        self.assertFalse(fresh.done())
        self.assertNotEqual(old['controller_boot_id'],self.context['controller_boot_id'])
        self.future(new.cancel_goal_async());self.future(fresh)
        self.node.destroy_client(client)
        (self.report/'managed-recovery-result.json').write_text(json.dumps(dict(
            service_recovery=True,old_group_gone=True,new_goal_executable=True,
            no_old_goal_resume=True,physical_motion=False),indent=2))


def load_tests(loader,tests,pattern):
    return unittest.TestSuite([ManagedRecovery('test_fault_rebuild_waits_new_goal')])
