#!/usr/bin/env python3
"""Bounded configure-only discovery A/B; never activates or sends a goal."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--navigator',type=Path)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--rounds',type=int,default=12)
    parser.add_argument('--with-chain',action='store_true')
    parser.add_argument('--background-chain',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='192'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('NEUPAN_ISOLATED_ROS_TEST')=='1'
    assert 1<=args.rounds<=30
    bundle=args.bundle.resolve(strict=True)
    if args.background_chain:
        # Reuse the actual isolated fixture without sending any Action goal.
        import failstop_chain_ros as chain
        install=bundle/'install';base=chain.FailstopChainROS
        chain.CONTROLLER=install/'nav2_controller/lib/nav2_controller/controller_server'
        chain.GUARD=install/'r1_collision_guard/lib/r1_collision_guard/r1_collision_guard_node'
        base.expected_plugin_root=str(install/'navigation_neupan_bridge/lib')
        base.worker_source_root=install/'r1_neupan_integration/lib/python3.10/site-packages/r1_neupan_integration'
        base.candidate_configdir=install/'navigation_neupan_bridge/share/navigation_neupan_bridge/config'
        base.candidate_checkpoint=install/'r1_neupan_integration/share/r1_neupan_integration/config/models/dune_model_5000.pth'
        base.worker_environment=dict(os.environ)
        base.worker_environment['PYTHONPATH']=str(bundle/'third_party/NeuPAN')+os.pathsep+os.environ.get('PYTHONPATH','')
        os.environ['NEUPAN_CHAIN_WORKER']='actual-solver'
        os.environ['NEUPAN_CHAIN_REPORT_DIR']=str(args.report)
        fixture=base('runTest');stopping=False
        def stop(*unused):
            nonlocal stopping
            stopping=True
        try:
            fixture.setUp()
            signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
            (args.report/'background-ready.json').write_text(json.dumps(dict(report=str(fixture.report))))
            while not stopping:fixture.tick()
        finally:fixture.doCleanups()
        return 0
    if args.navigator is None:parser.error('--navigator is required')
    exe=args.navigator.resolve(strict=True)
    args.report.mkdir(parents=True,exist_ok=False)
    root=bundle/'install/navigation_neupan_bridge'
    config=root/'share/navigation_neupan_bridge/config'
    plugin=root/'lib/libneupan_wait_condition_bt_node.so'
    assert plugin.is_file()
    params={'bt_navigator':{'ros__parameters':dict(use_sim_time=False,global_frame='map',
        robot_base_frame='base_link',odom_topic='/test_failstop_comm/odom',
        bt_loop_duration=10,default_server_timeout=1000,wait_for_service_timeout=1000,
        default_nav_to_pose_bt_xml=str(config/'navigate_w_replanning_neupan.xml'),
        default_nav_through_poses_bt_xml=str(config/'navigate_through_poses_neupan.xml'),
        plugin_lib_names=['nav2_compute_path_to_pose_action_bt_node',
            'nav2_compute_path_through_poses_action_bt_node','nav2_follow_path_action_bt_node',
            'nav2_rate_controller_bt_node','nav2_pipeline_sequence_bt_node',
            'nav2_recovery_node_bt_node','neupan_wait_condition_bt_node'])}}
    paramsfile=args.report/'params.yaml';paramsfile.write_text(yaml.safe_dump(params))
    rows=[];background=None;background_log=None
    if args.with_chain:
        background_log=(args.report/'background-chain.log').open('x')
        background=subprocess.Popen([sys.executable,__file__,'--bundle',str(bundle),
            '--report',str(args.report/'background'),'--background-chain'],stdout=background_log,stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        if background:
            deadline=time.monotonic()+12.
            while not (args.report/'background/background-ready.json').is_file() and time.monotonic()<deadline:
                if background.poll() is not None:raise RuntimeError('background chain startup failed')
                time.sleep(.01)
            if not (args.report/'background/background-ready.json').is_file():raise RuntimeError('background chain not ready')
        for index in range(args.rounds):
            modes=('ephemeral','precreated') if index%2==0 else ('precreated','ephemeral')
            for mode in modes:
                node=Node('test_navigator_configure_probe')
                row=dict(index=index,mode=mode,created=time.monotonic())
                proc=None
                logpath=args.report/(str(index)+'-'+mode+'.log')
                with logpath.open('x') as log:
                    try:
                        if background and background.poll() is not None:raise RuntimeError('background chain exited')
                        def clients():
                            return (node.create_client(ChangeState,'/bt_navigator/change_state'),
                                    node.create_client(GetState,'/bt_navigator/get_state'))
                        if mode=='precreated':change,state=clients()
                        proc=subprocess.Popen([str(exe),'--ros-args','--params-file',str(paramsfile)],
                                              stdout=log,stderr=subprocess.STDOUT)
                        if mode=='ephemeral':
                            # Wait until the server process has created its node,
                            # without a service client or an arbitrary settle sleep.
                            deadline=time.monotonic()+5.
                            while ('bt_navigator' not in {n for n,_ in node.get_node_names_and_namespaces()}
                                   and time.monotonic()<deadline):
                                assert proc.poll() is None
                                rclpy.spin_once(node,timeout_sec=.005)
                            change,state=clients()
                        deadline=time.monotonic()+5.
                        while not change.service_is_ready() and time.monotonic()<deadline:
                            assert proc.poll() is None
                            rclpy.spin_once(node,timeout_sec=.005)
                        row['ready']=time.monotonic();row['service_ready']=change.service_is_ready()
                        if not row['service_ready']:raise RuntimeError('configure service discovery timeout')
                        request=ChangeState.Request();request.transition.id=1
                        row['sent']=time.monotonic();future=change.call_async(request)
                        while not future.done() and time.monotonic()<row['sent']+2.:
                            assert proc.poll() is None
                            rclpy.spin_once(node,timeout_sec=.005)
                        row['completed']=future.done();row['ended']=time.monotonic()
                        row['success']=future.result().success if future.done() else None
                        # Query only. Never retry the non-idempotent transition.
                        query=state.call_async(GetState.Request());deadline=time.monotonic()+2.
                        while not query.done() and time.monotonic()<deadline:
                            rclpy.spin_once(node,timeout_sec=.005)
                        row['state']=query.result().current_state.id if query.done() else None
                        maps=(Path('/proc')/str(proc.pid)/'maps').read_text()
                        (args.report/(str(index)+'-'+mode+'-maps.txt')).write_text(maps)
                        row['actual_candidate_bt_loaded']=str(plugin) in maps
                    except Exception as exc:row['error']=repr(exc)
                    finally:
                        if proc is not None and proc.poll() is None:
                            proc.send_signal(signal.SIGINT)
                            try:proc.wait(timeout=3)
                            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
                        node.destroy_node()
                row['log']=str(logpath)
                row['server_response_timeout']='client will not receive response' in logpath.read_text()
                rows.append(row)
                (args.report/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
    finally:
        rclpy.shutdown()
        if background:
            if background.poll() is None:
                background.send_signal(signal.SIGINT)
                try:background.wait(timeout=15)
                except subprocess.TimeoutExpired:background.kill();background.wait(timeout=3)
            background_log.close()
    summary=dict(rounds=args.rounds,configure_calls=len(rows),activated=False,goals_sent=0,
        background_chain=args.with_chain,background_exit_code=background.returncode if background else None,
        candidate_plugin=dict(path=str(plugin),sha256=hashlib.sha256(plugin.read_bytes()).hexdigest()),
        navigator=dict(path=str(exe),sha256=hashlib.sha256(exe.read_bytes()).hexdigest()),
        modes={mode:dict(calls=sum(x['mode']==mode for x in rows),
            completed=sum(x['mode']==mode and x.get('success') is True for x in rows),
            response_timeouts=sum(x['mode']==mode and x['server_response_timeout'] for x in rows))
               for mode in ('ephemeral','precreated')},physical_motion=False)
    (args.report/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
    return 0 if all(x.get('success') and x.get('actual_candidate_bt_loaded') for x in rows) else 1


if __name__=='__main__':raise SystemExit(main())
