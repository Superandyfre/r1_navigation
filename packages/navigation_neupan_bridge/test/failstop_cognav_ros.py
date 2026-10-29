#!/usr/bin/env python3
"""Actual protected CogNav Bridge + actual failstop chain, no camera/driver."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import unittest
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import failstop_chain_ros as chain
sys.path.insert(0,str(chain.WS/'src/r1_navigation/integrations/cognav'))
from robot_live.cognav_bridge import Bridge
from robot_live.protocol import SCHEMA, write_json

class CogNavFailstop(chain.FailstopChainROS):
    def setUp(self):
        self.bridge=None;self.bridge_executor=None;self.plan=None
        super().setUp()
        self.localization=self.node.create_publisher(String,chain.PREFIX+'localization',20)
        args=argparse.Namespace(spool=str(self.report/'cognav_spool'),
            rgb_topic=chain.PREFIX+'rgb',depth_topic=chain.PREFIX+'depth',rgb_info_topic=chain.PREFIX+'rgb_info',
            depth_info_topic=chain.PREFIX+'depth_info',scan_topic=chain.PREFIX+'scan',odom_topic=chain.PREFIX+'odom',
            health_topic=chain.PREFIX+'localization',guard_topic=chain.PREFIX+'status',
            worker_health_topic=chain.PREFIX+'health',controller_status_topic=chain.PREFIX+'diagnostics',
            follow_path_action='/follow_path',controller_id='FollowPath',goal_checker_id='goal_checker',
            max_obs_hz=.5,motion_hz=20.,input_max_age=.5,lease_timeout=.6,max_obstacles=360,
            candidate_frame='test_candidate',calibration_version='synthetic')
        self.bridge_args=args
        self.bridge=Bridge(args)
        self.bridge_executor=SingleThreadedExecutor();self.bridge_executor.add_node(self.bridge)
        self.until(lambda:not self.bridge.neupan.reason(time.monotonic()) and
                   getattr(self.bridge,'motion_payload',{}).get('inputs_valid') is True,seconds=8)
        for name in ['cognav_bridge.py','protocol.py','shutdown.py','neupan_failstop.py']:
            p=chain.WS/'src/r1_navigation/integrations/cognav/robot_live'/name
            self.artifacts.append(dict(cognav_source=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
        (self.report/'binaries.json').write_text(json.dumps(self.artifacts,indent=2))

    def tick(self):
        super().tick()
        if self.bridge is None:return
        self.localization.publish(String(data=json.dumps(dict(state='READY',ready=True,
            operator_relocalization_pending=False,preflight_ready=True,fault_latched=False))))
        if self.plan:
            lease=dict(self.plan,at=time.monotonic(),valid_until=time.monotonic()+10.,permitted=True)
            write_json(self.bridge.root/'live/lease.json',lease)
        for _ in range(4):self.bridge_executor.spin_once(timeout_sec=0.)

    def shutdown(self):
        if self.bridge is not None:
            self.bridge.stopping=True
            self.bridge_executor.remove_node(self.bridge);self.bridge_executor.shutdown()
            chain.destroy_actions(self.bridge)
            self.bridge.destroy_node();self.bridge=None
        super().shutdown()

    def submit(self,seq):
        np.savez(self.bridge.root/'plan/path.npz',path=np.array([[0.,0.,0.],[2.,0.,0.],[4.,0.,0.]]))
        self.plan=dict(schema=SCHEMA,bridge_session=self.bridge.session,driver_session='test-driver',
            task_id='test-task',map_session='test-map',generation=1,seq=seq,version=seq,uid='same-target',
            kind='navigate',path_file='path.npz',expires_at=time.monotonic()+10.)
        write_json(self.bridge.root/'plan'/('%06d.json'%seq),self.plan)

    def exercise_cognav(self, freeze=False):
        self.submit(1)
        self.until(lambda:self.bridge.counts['goals_sent']==1,seconds=8)
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or
                   self.bridge.lifecycle.current['state'] in ('aborted','failed','rejected'),seconds=45)
        self.assertTrue(any(v>.04 for _,v,_ in self.outputs),'no initial motion: '+str(self.report))
        ctx=self.bridge.lifecycle.current
        self.assertEqual(ctx['navigation_attempt_id'],self.context['navigation_attempt_id'])
        marker=time.monotonic()
        if freeze:self.source_enabled=False
        else:
            self.fatal=True;self.publish('inject',dict(fatal=True))
        self.until(lambda:bool(self.bridge.neupan.fault))
        self.until(lambda:not self.bridge.lifecycle.busy())
        self.assertIn(self.bridge.lifecycle.current['state'],('aborted','cancelled'))
        self.submit(2)
        ack=self.bridge.root/'ack/000002.json';self.until(ack.is_file)
        refused=json.loads(ack.read_text())
        self.assertFalse(refused['sent']);self.assertIn('neupan:',refused['note'])
        self.assertEqual(self.bridge.counts['goals_sent'],1)
        settled=time.monotonic();self.until(lambda:time.monotonic()-settled>.35)
        # Source freshness expires at .5s; unlike explicit FATAL it cannot
        # be required to stop .2s after the last input. Verify the existing
        # source watchdog deadline and the settled terminal output separately.
        zero_deadline=marker+(.65 if freeze else .2)
        self.assertTrue(any(t>zero_deadline and v==0. and w==0. for t,v,w in self.outputs))
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>zero_deadline))
        runtime=[j for t,j in self.runtime_statuses if j.get('schema')=='r1_neupan_execution_status_v1' and t>settled]
        self.assertTrue(runtime,'Controller status must continue after Action terminal')
        self.assertTrue(all(j['fault_latched'] for j in runtime))
        (self.report/'cognav-result.json').write_text(json.dumps(dict(action=self.bridge.lifecycle.result(),
            veto=self.bridge.neupan.fault,new_plan_ack=refused,goals_sent=self.bridge.counts['goals_sent'],
            worker=self.worker_mode,source_freeze=freeze,physical_motion=False),indent=2))
        if freeze:
            self.assertTrue(any(j['state']=='RUNNING' for t,j in self.healths if t>settled))
        old_session=self.bridge.session
        self.bridge_executor.remove_node(self.bridge)
        chain.destroy_actions(self.bridge)
        self.bridge.destroy_node()
        self.plan=None
        self.bridge=Bridge(self.bridge_args);self.bridge_executor.add_node(self.bridge)
        self.assertNotEqual(self.bridge.session,old_session)
        self.until(lambda:bool(self.bridge.neupan.fault))
        self.submit(3)
        restart_ack=self.bridge.root/'ack/000003.json';self.until(restart_ack.is_file)
        row=json.loads(restart_ack.read_text())
        self.assertFalse(row['sent']);self.assertIn('neupan:',row['note'])
        self.assertEqual(self.bridge.counts['goals_sent'],0)
        (self.report/'cognav-restart-result.json').write_text(json.dumps(dict(veto=self.bridge.neupan.fault,
            ack=row,goals_sent=self.bridge.counts['goals_sent'],physical_motion=False),indent=2))

    def test_cognav_fatal_refuses_new_plan(self):self.exercise_cognav()
    def test_cognav_source_timeout_refuses_new_plan(self):self.exercise_cognav(True)

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(CogNavFailstop(name) for name in ['test_cognav_fatal_refuses_new_plan',
        'test_cognav_source_timeout_refuses_new_plan'])
if __name__=='__main__':unittest.main()
