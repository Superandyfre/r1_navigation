"""Own UI7 navigation processes; explicit bounded recovery, never replay a goal."""
import json
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PolygonStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from r1_neupan_integration.runtime_recovery import RecoveryTransaction, OwnedProcessGroup


def runtime_launch_arguments(values):
    # An empty optional launch argument is represented by omission: the ROS 2
    # CLI rejects `name:=`, while the owned launch declares an empty default.
    return [key + ':=' + str(value) for key, value in values.items() if str(value)]


class Supervisor(Node):
    def __init__(self, **node_arguments):
        super().__init__('ui7_runtime_supervisor', **node_arguments)
        self.group=ReentrantCallbackGroup();self.data_lock=threading.Lock();self.rows={}
        self.logs=[];self.runtime=None;self.rebuilding=False;self.closing=threading.Event()
        self.recovery_zero=False
        self.transaction=RecoveryTransaction(self,timeout=45.)
        parameters=dict(calibration_record='',scan_source='dual',use_mesh_self_filter='true',
                        mesh_filter_mode='3d',arm_extra_mesh_config='',mesh_projection_margin_m='0.06',command_profile='smoothed',cmd_vel_out_topic='/cmd_vel',
                        report_dir='/home/rpp/neupan_failstop_ws/reports/runtime-recovery')
        self.values={key:self.declare_parameter(key,value).value for key,value in parameters.items()}
        self.report_root=Path(self.values.pop('report_dir'));self.report_root.mkdir(parents=True,exist_ok=True)
        self.base_command=['/opt/ros/humble/bin/ros2','launch','navigation_neupan_bridge','ui7_owned_runtime.launch.py']
        self.base_command += runtime_launch_arguments(self.values)
        self.subscriptions_owned=[]
        def sub(typ,topic,key,json_payload=False):
            def callback(message):
                try:payload=json.loads(message.data) if json_payload else message
                except ValueError:return
                with self.data_lock:self.rows[key]=(time.monotonic(),payload)
            self.subscriptions_owned.append(self.create_subscription(
                typ,topic,callback,qos_profile_sensor_data if typ in (Odometry,LaserScan) else 20,
                callback_group=self.group))
        for key,topic in [('worker','/neupan_worker_health'),('guard','/r1_collision_guard/status'),
                          ('controller','/neupan_controller_status'),('localization','/r1_localization_health')]:
            sub(String,topic,key,True)
        sub(Twist,self.values['cmd_vel_out_topic'],'output');sub(Odometry,'/odom','odom')
        sub(LaserScan,'/scan_neupan_mesh_shadow' if self.values['use_mesh_self_filter']=='true' else '/scan_neupan','scan')
        sub(PolygonStamped,'/r1_arm_footprint','footprint')
        self.recovery_clients={key:self.create_client(Trigger,topic,callback_group=self.group) for key,topic in
                      [('controller','/neupan_controller/revoke_uncertain_execution'),
                       ('guard','/r1_collision_guard/stop_for_recovery'),('arm','/neupan_controller/arm')]}
        self.recover_service=self.create_service(Trigger,'/neupan_runtime/recover',self.recover,callback_group=self.group)
        self.cancel_service=self.create_service(Trigger,'/neupan_runtime/cancel_recovery',self.cancel,callback_group=self.group)
        self.zero_pub=self.create_publisher(Twist,self.values['cmd_vel_out_topic'],10)
        self.status_pub=self.create_publisher(String,'/neupan_runtime/recovery_status',10)
        self.timer=self.create_timer(.1,self.status,callback_group=self.group)
        self.launch_runtime(auto_arm=True)

    def status(self):
        if self.recovery_zero:self.zero_pub.publish(Twist())
        self.status_pub.publish(String(data=json.dumps(dict(schema='r1_runtime_recovery_v1',
            phase=self.transaction.phase,runtime_leader_alive=bool(self.runtime and self.runtime.process.poll() is None),
            monotonic=time.monotonic()))))

    def recover(self,request,response):
        response.success,response.message=self.transaction.run()
        if response.success:
            self.rebuilding=False;self.recovery_zero=False
        return response

    def cancel(self,request,response):
        self.transaction.cancelled.set();response.success=True;response.message='cancellation_requested';return response

    def wait(self,predicate,deadline,cancel):
        while True:
            if cancel.is_set() or self.closing.is_set() or time.monotonic()>=deadline:
                raise RuntimeError('cancelled_or_deadline')
            if predicate():return
            time.sleep(.02)

    def call(self,key,deadline,cancel):
        client=self.recovery_clients[key];end=min(deadline,time.monotonic()+1.)
        self.wait(client.service_is_ready,end,cancel)
        future=client.call_async(Trigger.Request());self.wait(future.done,end,cancel)
        if not future.result().success:raise RuntimeError('deny:'+key+':'+future.result().message)

    def launch_runtime(self,auto_arm):
        self.started=time.monotonic()
        log=(self.report_root/('runtime-'+str(time.time_ns())+'.log')).open('w');self.logs.append(log)
        self.runtime=OwnedProcessGroup(self.base_command+['runtime_auto_arm:='+str(auto_arm).lower()],log)
        self.runtime.start()

    def capture(self,deadline,cancel):
        self.rebuilding=False
        if self.runtime is None:raise RuntimeError('owned_runtime_missing')
        self.report=self.report_root/str(time.time_ns());self.report.mkdir()
        with self.data_lock:
            self.old_ids={key:self.rows.get(key,(0,{}))[1].get(field) for key,field in
                [('worker','boot_id'),('guard','guard_boot_id'),('controller','controller_boot_id')]}
        self.save_evidence(deadline,cancel)

    def revoke(self,deadline,cancel):
        successes=0
        for key in ('controller','guard'):
            try:self.call(key,deadline,cancel);successes+=1
            except RuntimeError:
                if cancel.is_set():raise
        if not successes:raise RuntimeError('no_independent_revoke_ack')
        # Independent deny-only final output also covers an already dead Guard.
        # Fresh stopped odometry remains mandatory before any process rebuild.
        self.recovery_zero=True
        self.zero_pub.publish(Twist())

    def stopped(self):
        now=time.monotonic()
        with self.data_lock:
            out_time,out=self.rows.get('output',(0,None));odom_time,odom=self.rows.get('odom',(0,None))
        if out is None or odom is None or now-out_time>.2 or now-odom_time>.2:return None
        if any(not math.isfinite(v) or v!=0. for v in
               (out.linear.x,out.linear.y,out.linear.z,out.angular.x,out.angular.y,out.angular.z)):return None
        stamp=odom.header.stamp.sec*1000000000+odom.header.stamp.nanosec
        v=odom.twist.twist
        if (odom.header.frame_id!='odom' or odom.child_frame_id!='base_link' or
            not 0<=self.get_clock().now().nanoseconds-stamp<=200000000 or
            any(not math.isfinite(value) or abs(value)>.01 for value in
                (v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z))):return None
        return stamp

    def verify_stopped(self,deadline,cancel):
        since=None;previous=-1
        def stable():
            nonlocal since,previous
            stamp=self.stopped()
            if stamp is None or stamp<previous:since=None;return False
            if stamp==previous:return False
            previous=stamp
            if since is None:since=time.monotonic()
            return time.monotonic()-since>=.3
        try:self.wait(stable,min(deadline,time.monotonic()+3.),cancel)
        except RuntimeError:
            with self.data_lock:
                t,odom=self.rows.get('odom',(0,None));out_t,out=self.rows.get('output',(0,None))
                debug=dict(now=time.monotonic(),odom_received=t,output_received=out_t,
                           odom=str(odom),output=str(out),ros_now=self.get_clock().now().nanoseconds,
                           since=since,previous=previous)
            (self.report/'stopped-failure.json').write_text(json.dumps(debug,indent=2))
            raise

    def save_evidence(self,deadline,cancel):
        with self.data_lock:rows={key:dict(received=t,payload=row) for key,(t,row) in self.rows.items() if isinstance(row,dict)}
        (self.report/(self.transaction.phase+'.json')).write_text(json.dumps(dict(
            phase=self.transaction.phase,old_ids=self.old_ids,states=rows,
            owned_pids=self.runtime.members() if self.runtime else [],monotonic=time.monotonic()),indent=2))

    def stop_old(self,deadline,cancel):
        self.rebuilding=True
        self.runtime.stop(deadline,cancel)

    def start_new(self,deadline,cancel):
        if self.runtime.members():raise RuntimeError('old_runtime_survived')
        self.launch_runtime(auto_arm=False)

    def verify_new_idle(self,deadline,cancel):
        def ready():
            if self.runtime.process.poll() is not None:raise RuntimeError('replacement_launch_exited')
            now=time.monotonic()
            with self.data_lock:
                for key,field in [('worker','boot_id'),('guard','guard_boot_id'),('controller','controller_boot_id')]:
                    stamp,row=self.rows.get(key,(0,{}))
                    if (stamp<self.started or now-stamp>.3 or not row.get(field)
                            or row[field]==self.old_ids[key]):return False
                guard=self.rows['guard'][1];controller=self.rows['controller'][1];worker=self.rows['worker'][1]
                if (guard.get('schema')!='r1_collision_guard_status_v1' or
                        guard.get('health_protocol')!='r1_neupan_worker_health_v2' or
                        controller.get('schema')!='r1_neupan_execution_status_v1' or
                        worker.get('schema')!='r1_neupan_worker_health_v2'):return False
                if guard.get('authorization_state')!='DISARMED':return False
                if worker.get('state') not in ('STARTING','RUNNING') or worker.get('fault_id'):return False
                if (controller.get('execution_active') or controller.get('fault_latched') or
                        controller.get('run_armed') or not controller.get('plugin_active')):return False
                stamp,loc=self.rows.get('localization',(0,{}))
                if now-stamp>.5 or loc.get('state')!='READY':return False
                for key in ('scan','footprint'):
                    stamp,row=self.rows.get(key,(0,None))
                    if stamp<self.started or now-stamp>.5 or row is None:return False
                    header=row.header.stamp.sec*1000000000+row.header.stamp.nanosec
                    if not 0<=self.get_clock().now().nanoseconds-header<=500000000:return False
                    if row.header.frame_id!='base_link':return False
                    if key=='footprint':
                        if len(row.polygon.points)<3 or not all(math.isfinite(v) for p in row.polygon.points for v in (p.x,p.y,p.z)):return False
                    elif not row.ranges or not math.isfinite(row.angle_increment) or row.angle_increment<=0:return False
                return True
        self.wait(ready,deadline,cancel);self.verify_stopped(deadline,cancel);self.save_evidence(deadline,cancel)

    def enable_idle(self,deadline,cancel):
        self.call('arm',deadline,cancel)
        # This is internal completion of recovery, not a normal manual-arm step.
        self.verify_stopped(deadline,cancel)
        with self.data_lock:
            if self.rows['controller'][1].get('execution_active'):raise RuntimeError('unexpected_new_goal_during_recovery')

    def abort(self):
        if self.rebuilding and self.runtime is not None:
            # Only after old runtime was verified stopped. Failed preconditions
            # leave the old fault/deny state intact for operator investigation.
            self.runtime.stop(time.monotonic()+4.,threading.Event())

    def close(self):
        self.closing.set();self.transaction.cancelled.set()
        if self.runtime:self.runtime.stop(time.monotonic()+4.,threading.Event())
        for log in self.logs:log.close()


def main():
    rclpy.init();node=Supervisor();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:
        node.closing.set();node.transaction.cancelled.set()
        executor.shutdown(timeout_sec=5.)
        node.close();node.destroy_node();rclpy.try_shutdown()
