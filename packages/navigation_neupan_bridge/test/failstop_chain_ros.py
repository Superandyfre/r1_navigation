#!/usr/bin/env python3
"""Actual ControllerServer/plugin + velocity smoother + final Guard, fake Worker.

Isolated DDS only. Synthetic pose/scan/odom/source are NOT physical evidence.
No driver is started and every command topic is test-only. Explicit opt-in.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml

WS = Path('/home/rpp/neupan_failstop_ws')
CONTROLLER = WS/'install/nav2_controller/lib/nav2_controller/controller_server'
GUARD = WS/'install/r1_collision_guard/lib/r1_collision_guard/r1_collision_guard_node'
SMOOTHER = Path('/home/rpp/rpp_ws/install/nav2_velocity_smoother/lib/nav2_velocity_smoother/velocity_smoother')
PREFIX = '/test_failstop_chain/'


@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'isolated opt-in only')
class FailstopChainROS(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '192')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.report = Path(os.environ.get('NEUPAN_CHAIN_REPORT_DIR', str(WS/'reports/neupan_failstop_20261027/tests/chain')))
        self.report = self.report/(self._testMethodName+'-'+str(time.time_ns()))
        self.report.mkdir(parents=True)
        self.procs, self.logs = [], []
        rclpy.init()
        self.node = Node('failstop_chain_peer')
        self.addCleanup(self.shutdown)
        self.tf = TransformBroadcaster(self.node)
        self.artifacts=[]
        self.context = None
        self.statuses, self.outputs, self.raw, self.smooth, self.contexts = [], [], [], [], []
        self.sources=[]
        self.healths=[]
        self.runtime_statuses=[]
        self.sequence = 0
        self.next_pulse = 0.
        self.worker_mode=os.environ.get('NEUPAN_CHAIN_WORKER','synthetic')
        self.real_node=self.worker_mode in ('actual-node-fake-ipc','actual-solver')
        self.fatal = False
        self.source_enabled = True
        self.position = 0.
        self.pubs = {k:self.node.create_publisher(t, PREFIX+k, 20) for k,t in (
            ('health',String),('source',String),('inject',String),('scan',LaserScan),('worker_scan',LaserScan),('odom',Odometry))}
        def context(msg):
            self.context=json.loads(msg.data)
            self.contexts.append((time.monotonic(),self.context))
        self.subs = [self.node.create_subscription(String,PREFIX+'diagnostics',lambda m:self.runtime_statuses.append((time.monotonic(),json.loads(m.data))),20),
                     self.node.create_subscription(String,PREFIX+'health',lambda m:self.healths.append((time.monotonic(),json.loads(m.data))),20),
                     self.node.create_subscription(String,PREFIX+'source',lambda m:self.sources.append((time.monotonic(),json.loads(m.data))),20),
                     self.node.create_subscription(String,PREFIX+'context',context,20),
                     self.node.create_subscription(String,PREFIX+'status',lambda m:self.statuses.append((time.monotonic(),json.loads(m.data))),20)]
        for name, rows in [('output',self.outputs),('raw',self.raw),('smooth',self.smooth)]:
            self.subs.append(self.node.create_subscription(Twist,PREFIX+name,
                lambda m,rows=rows:rows.append((time.monotonic(),m.linear.x,m.angular.z)),20))
        plugin = dict(plugin='navigation_neupan_bridge/NeuPANController',shadow_mode=False,
            require_localization_ready=False,require_gate_match=False,navigation_frame='map',
            max_linear=.20,max_reverse=.20,max_angular=.40,
            worker_health_timeout=.5,navigation_authority_timeout=.5,planner_input_timeout=.75,
            envelope_timeout=.5,envelope_recovery_timeout=1.,envelope_startup_timeout=40.,
            arm_odom_frame='odom',arm_base_frame='base_link')
        for key,topic in dict(path_topic='path',goal_topic='goal',envelope_topic='source',
            guard_status_topic='status',worker_health_topic='health',execution_context_topic='context',
            motion_grant_topic='grant',motion_arm_service='arm',motion_retry_service='retry',odom_topic='odom',
            gate_command_topic='unused_gate_cmd',gate_status_topic='unused_gate_status',
            localization_health_topic='unused_localization',diagnostics_topic='diagnostics').items():
            plugin[key]=PREFIX+topic
        config = {
          'controller_server': {'ros__parameters':dict(use_sim_time=False,controller_frequency=20.,
            failure_tolerance=0.,runtime_costmap_wait_timeout_sec=2.,odom_topic=PREFIX+'odom',
            controller_plugins=['FollowPath'],progress_checker_plugin='progress_checker',
            goal_checker_plugins=['goal_checker'],
            progress_checker={'plugin':'nav2_controller::SimpleProgressChecker','movement_time_allowance':getattr(self,'progress_allowance',60.)},
            goal_checker={'plugin':'nav2_controller::SimpleGoalChecker','xy_goal_tolerance':.1,'yaw_goal_tolerance':.1},
            FollowPath=plugin)},
          'local_costmap':{'local_costmap':{'ros__parameters':dict(use_sim_time=False,
            global_frame='map',robot_base_frame='base_link',rolling_window=True,width=4,height=4,
            resolution=.1,update_frequency=10.,publish_frequency=2.,transform_tolerance=.3,
            footprint='[[.3,.2],[.3,-.2],[-.3,-.2],[-.3,.2]]',plugins=['inflation_layer'],
            inflation_layer={'plugin':'nav2_costmap_2d::InflationLayer'})}},
          'velocity_smoother':{'ros__parameters':dict(use_sim_time=False,smoothing_frequency=30.,
            feedback='OPEN_LOOP',max_velocity=[.3,0.,.5],min_velocity=[-.3,0.,-.5],
            max_accel=[.3,0.,1.],max_decel=[-.3,0.,-1.],velocity_timeout=.3,
            odom_topic=PREFIX+'odom')},
          'r1_collision_guard':{'ros__parameters':dict(use_sim_time=False,
            footprint_points=[.3,.2,.3,-.2,-.3,-.2,-.3,.2],
            require_arms_stowed=False,require_localization_ready=False,require_dynamic_footprint=False,
            collision_check_enabled=True,cmd_vel_in_topic=PREFIX+'smooth',source_cmd_vel_topic=PREFIX+'raw',
            cmd_vel_out_topic=PREFIX+'output',status_topic=PREFIX+'status',scan_topic=PREFIX+'scan',
            odom_topic=PREFIX+'odom',worker_health_topic=PREFIX+'health',planner_source_topic=PREFIX+'source',
            motion_grant_topic=PREFIX+'grant',planner_recovery_timeout_sec=1.)}}
        path=self.report/'params.yaml';path.write_text(yaml.safe_dump(config))
        if self.real_node:
            packages=WS/'src/r1_navigation/packages'
            parameters=dict(robot_config_dir=str(packages/'navigation_neupan_bridge/config'),
                planner_config_file='planner_r1_hw1.yaml',
                dune_checkpoint_file=str(packages/'r1_neupan_integration/config/models/dune_model_5000.pth'),
                n115_output_dir=str(self.report/'worker'),map_frame='map',
                execution_context_topic=PREFIX+'context',worker_health_topic=PREFIX+'health',
                planner_result_topic=PREFIX+'source',cmd_vel_topic=PREFIX+'worker_diagnostic_cmd',
                scan_topic=PREFIX+'worker_scan',odom_topic=PREFIX+'odom',
                plan_input_topic=PREFIX+'path',goal_topic=PREFIX+'goal',
                forward_only_enabled=False,command_stabilizer_enabled=False,
                auto_gear_transition_enabled=False,obstacle_motion_enabled=False)
            helper=Path(__file__).with_name('chain_worker_peer.py')
            python='/home/rpp/venvs/neupan_ros_humble/bin/python' if self.worker_mode=='actual-solver' else '/usr/bin/python3'
            args=[python,str(helper),'--ros-args']
            for key,value in parameters.items():
                args+=['-p',key+':='+str(value).lower() if isinstance(value,bool) else key+':='+str(value)]
            log=open(self.report/'actual_worker_node.log','w');self.logs.append(log)
            self.procs.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))
        self.start(GUARD,path,[])
        self.start(SMOOTHER,path,['cmd_vel:='+PREFIX+'raw','cmd_vel_smoothed:='+PREFIX+'smooth'])
        self.start(CONTROLLER,path,['cmd_vel:='+PREFIX+'raw'])
        self.lifecycle('velocity_smoother',1)
        self.lifecycle('velocity_smoother',3)
        self.lifecycle('controller_server',1)
        self.lifecycle('controller_server',3)
        activated=time.monotonic()
        self.until(lambda:time.monotonic()-activated>.6)
        self.until(lambda:len(self.statuses)>0)
        self.until(lambda:len(self.outputs)>0)
        self.until(lambda:time.monotonic()-self.statuses[0][0]>.35)
        client=self.node.create_client(Trigger,PREFIX+'arm')
        self.until(client.service_is_ready)
        # DDS discovery and the stopped-odom dwell are asynchronous. Test
        # preparation may wait for admission, but never reset a fault or relax
        # its evidence thresholds. Preserve every denied preflight request.
        arm_attempts=[]
        arm_deadline=time.monotonic()+3.
        while True:
            response=self.future(client.call_async(Trigger.Request()))
            arm_attempts.append(dict(monotonic=time.monotonic(),success=response.success,message=response.message))
            (self.report/'preflight-arm.json').write_text(json.dumps(arm_attempts,indent=2))
            if response.success or time.monotonic()>=arm_deadline:break
            resume=time.monotonic()+.05
            self.until(lambda:time.monotonic()>=resume)
        self.assertTrue(response.success,response.message)
        self.action=ActionClient(self.node,FollowPath,'/follow_path')
        self.until(self.action.server_is_ready)
        self.assertFalse(any(name=='/cmd_vel' for name,_ in self.node.get_topic_names_and_types()))
        loaded=set()
        for proc in self.procs:
            maps=Path('/proc')/str(proc.pid)/'maps'
            content=maps.read_text()
            (self.report/('maps-'+str(proc.pid)+'.txt')).write_text(content)
            for line in content.splitlines():
                candidate=line.split()[-1]
                if candidate.startswith(str(WS)) or 'libvelocity_smoother_core.so' in candidate:
                    path=Path(candidate)
                    if path.is_file():loaded.add(path.resolve())
        for path in sorted(loaded):
            self.artifacts.append(dict(loaded_library=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        if self.real_node:
            for path in [WS/'src/r1_navigation/packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml',
                         WS/'src/r1_navigation/packages/r1_neupan_integration/config/models/dune_model_5000.pth']:
                self.artifacts.append(dict(planner_input=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            for name in ('n1_15_node.py','n1_16_node.py','execution_protocol.py'):
                path=WS/'src/r1_navigation/packages/r1_neupan_integration/r1_neupan_integration'/name
                self.artifacts.append(dict(worker_source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        self.assertTrue(any('neupan_failstop_ws' in p and 'libnavigation_neupan_bridge.so' in p
            for proc in self.procs for p in (Path('/proc')/str(proc.pid)/'maps').read_text().splitlines()))
        (self.report/'binaries.json').write_text(json.dumps(self.artifacts,indent=2))

    def start(self,exe,params,remaps):
        self.assertTrue(exe.is_file(),str(exe))
        self.artifacts.append(dict(path=str(exe),resolved=str(exe.resolve()),
            sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
        log=open(self.report/(exe.name+'.log'),'w')
        self.logs.append(log)
        args=[str(exe),'--ros-args','--params-file',str(params)]
        for remap in remaps:args+=['-r',remap]
        self.procs.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))

    def shutdown(self):
        for proc in reversed(self.procs):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
        for log in self.logs:log.close()
        (self.report/'trace.json').write_text(json.dumps(dict(status=self.statuses,output=self.outputs,
            raw=self.raw,smooth=self.smooth,context=self.contexts,source=self.sources,health=self.healths,runtime_status=self.runtime_statuses),indent=2))
        self.node.destroy_node();rclpy.shutdown()

    def publish(self,key,payload):
        self.pubs[key].publish(String(data=json.dumps(payload,allow_nan=False)))

    def tick(self):
        for proc in self.procs:
            self.assertIsNone(proc.poll(),'component exited, logs: '+str(self.report))
        now=time.monotonic()
        if now>=self.next_pulse:
            self.next_pulse=now+.025;self.sequence+=1
            stamp=self.node.get_clock().now().to_msg()
            transforms=[]
            for parent,child,x in [('map','odom',0.),('odom','base_link',self.position)]:
                t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
                t.transform.translation.x=x;t.transform.rotation.w=1.;transforms.append(t)
            self.tf.sendTransform(transforms)
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
            odom.pose.pose.position.x=self.position;odom.pose.pose.orientation.w=1.;self.pubs['odom'].publish(odom)
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link'
            scan.angle_min=-3.14;scan.angle_increment=.02;scan.angle_max=3.14
            scan.range_min=.05;scan.range_max=8.;scan.ranges=[7.]*315;self.pubs['scan'].publish(scan)
            if self.source_enabled:self.pubs['worker_scan'].publish(scan)
            active=self.context is not None and self.context['state']=='ACTIVE'
            health=dict(schema='r1_neupan_worker_health_v1',boot_id='synthetic-worker',worker_generation=1,
                sequence=self.sequence,monotonic=now,state='FAULT_LATCHED' if self.fatal else ('RUNNING' if active else 'STARTING'),
                navigation_attempt_id=self.context['navigation_attempt_id'] if self.context else None,
                path_version=self.context['path_version'] if self.context else 0,
                goal_version=self.context['goal_version'] if self.context else 0,
                fault_id='injected-fatal' if self.fatal else None,fault_code='worker_crash' if self.fatal else None)
            if not self.real_node:self.publish('health',health)
            elif self.fatal:self.publish('inject',dict(fatal=True))
            if active and not self.fatal and self.source_enabled and not self.real_node:
                source={k:self.context[k] for k in ['authority_boot_id','controller_boot_id','navigation_attempt_id','path_version','goal_version']}
                source.update(schema='r1_neupan_planner_source_v2',worker_boot_id='synthetic-worker',worker_generation=1,
                    request_id=self.sequence,cycle_id=self.sequence,scan_counter=self.sequence,finite=True,
                    scan_receive_monotonic=now-.01,request_dispatched_monotonic=now-.008,
                    worker_solve_end_monotonic=now-.002,planner_result_monotonic=now,
                    command=dict(linear_x=.1,angular_z=0.))
                self.publish('source',source)
        rclpy.spin_once(self.node,timeout_sec=.01)

    def until(self,predicate,seconds=6):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:self.tick()
        self.assertTrue(predicate(),'timeout; report='+str(self.report)+' last='+str(self.statuses[-1:]))

    def future(self,future,seconds=6):
        self.until(future.done,seconds);return future.result()

    def lifecycle(self,name,transition):
        client=self.node.create_client(ChangeState,'/'+name+'/change_state')
        self.until(client.service_is_ready)
        req=ChangeState.Request();req.transition.id=transition
        response=self.future(client.call_async(req))
        self.assertTrue(response.success,name+' transition '+str(transition)+' '+str(self.report))

    def goal(self):
        goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
        goal.path.header.frame_id='map'
        from geometry_msgs.msg import PoseStamped
        for x in [0.,2.,4.]:
            pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.orientation.w=1.
            goal.path.poses.append(pose)
        handle=self.future(self.action.send_goal_async(goal));self.assertTrue(handle.accepted)
        return handle

    def exercise(self,mode):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertTrue(any(v>.04 for _,v,_ in self.outputs),
            'Action ended before any final nonzero, see '+str(self.report))
        self.assertEqual(self.context['navigation_attempt_id'],bytes(handle.goal_id.uuid).hex())
        self.assertFalse(result.done())
        marker=time.monotonic()
        if mode=='fatal':self.fatal=True
        elif mode=='cancel':self.future(handle.cancel_goal_async())
        elif mode=='success':self.position=4.
        elif mode=='source_freeze':self.source_enabled=False
        terminal=self.future(result)
        expected={'fatal':GoalStatus.STATUS_ABORTED,'source_freeze':GoalStatus.STATUS_ABORTED,
                  'cancel':GoalStatus.STATUS_CANCELED,'success':GoalStatus.STATUS_SUCCEEDED}[mode]
        self.assertEqual(terminal.status,expected)
        terminal_at=time.monotonic()
        self.until(lambda:bool(self.outputs) and abs(self.outputs[-1][1])<1e-8)
        settled=time.monotonic()+.4
        while time.monotonic()<settled:self.tick()
        rows=[row for row in self.outputs if row[0]>=terminal_at+.06]
        self.assertGreaterEqual(len(rows),8)
        self.assertTrue(all(abs(v)<1e-8 and abs(w)<1e-8 for _,v,w in rows),rows)
        self.assertTrue(any(c['state']=='END' for _,c in self.contexts))
        nonzero=[t for t,v,w in self.outputs if t>=marker and (abs(v)>1e-8 or abs(w)>1e-8)]
        last_nonzero=max(nonzero,default=marker)
        first_zero=next(t for t,v,w in self.outputs if t>=last_nonzero and abs(v)<1e-8 and abs(w)<1e-8)
        if mode=='fatal':
            retry=self.goal()
            self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
            self.assertFalse(any(t>terminal_at+.06 and (abs(v)>1e-8 or abs(w)>1e-8) for t,v,w in self.outputs))
        record=dict(scenario=mode,result_status=terminal.status,terminal_sec=terminal_at-marker,
            final_zero_observed_sec=first_zero-marker,last_nonzero_after_injection_sec=last_nonzero-marker,
            continuous_zero_samples=len(rows),report=str(self.report),real_components=['ControllerServer','NeuPANController','velocity_smoother','CollisionGuard'],
            worker=self.worker_mode,physical_motion=False)
        (self.report/'result.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))

    def test_real_solver_observation_window(self):
        if self.worker_mode!='actual-solver':self.skipTest('actual solver only')
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        marker=time.monotonic()
        duration=30.
        while time.monotonic()<marker+duration:
            self.tick()
            self.assertFalse(result.done(),'unexpected terminal during steady observation')
        rows=[r for r in self.outputs if r[0]>=marker]
        self.assertGreater(len(rows),duration*30)
        nonzero=sum(abs(v)>.04 for _,v,w in rows)
        self.assertGreater(nonzero/len(rows),.8)
        sources=[s for t,s in self.sources if t>=marker]
        self.assertGreater(len(sources),30)
        age=sorted(s['planner_result_monotonic']-s['scan_receive_monotonic'] for s in sources)
        self.assertTrue(all(v>=0 for v in age))
        self.future(handle.cancel_goal_async())
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
        record=dict(scenario='real_solver_30sec',worker=self.worker_mode,physical_motion=False,
            duration_sec=duration,final_samples=len(rows),nonzero_fraction=nonzero/len(rows),
            source_count=len(sources),input_age_p95_sec=age[int(.95*(len(age)-1))],
            input_age_max_sec=max(age),report=str(self.report))
        (self.report/'result.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))

    def test_same_path_uuid_preemption(self):
        first=self.goal();first_result=first.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs),50)
        old_id=bytes(first.goal_id.uuid).hex()
        second=self.goal();second_id=bytes(second.goal_id.uuid).hex()
        self.assertNotEqual(old_id,second_id)
        self.assertEqual(self.future(first_result).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:self.context['state']=='ACTIVE' and self.context['navigation_attempt_id']==second_id)
        cutoff=time.monotonic()
        self.until(lambda:any(t>cutoff and v>.04 for t,v,w in self.outputs))
        result=second.get_result_async();self.future(second.cancel_goal_async())
        self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
        self.until(lambda:any(c['state']=='END' and c['navigation_attempt_id']==second_id for _,c in self.contexts))
        ends={c['navigation_attempt_id'] for _,c in self.contexts if c['state']=='END'}
        self.assertTrue({old_id,second_id}.issubset(ends))
        record=dict(scenario='same_path_uuid_preemption',old_attempt=old_id,new_attempt=second_id,
            worker=self.worker_mode,
            report=str(self.report),physical_motion=False)
        (self.report/'result.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))

    def test_fatal(self):self.exercise('fatal')
    def test_cancel(self):self.exercise('cancel')
    def test_success(self):self.exercise('success')
    def test_source_freeze(self):self.exercise('source_freeze')


if __name__=='__main__':unittest.main()
