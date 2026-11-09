"""N116 proof producer tests without ROS graph or physical motion."""
from dataclasses import asdict
from types import SimpleNamespace
from unittest.mock import Mock
import json,threading,time,unittest
from r1_neupan_integration.n1_16_node import N116Node
from r1_neupan_integration.trajectory_continuation import Continuation

class Producer(unittest.TestCase):
    def setUp(self):
        self.node=N116Node.__new__(N116Node);n=self.node
        n.state_lock=threading.Lock();n._shutdown_requested=False;n._freeze_planner_result=False
        n._continuation_handled=0;n._continuation_proof_sequence=0;n._continuation_last_scan=2
        n._continuation_timeout=.75;n._continuation_pub=Mock();n._trace_wait_state=Mock()
        n.scan_counter=2;n.latest_local_scan_counter=0;n._continuation_worker=Mock()
        self.now=time.monotonic();t=self.now
        self.context=('authority','controller','attempt','worker',7,1,0)
        self.parent={'schema':'r1_neupan_planner_source_v2','authority_boot_id':'authority',
            'controller_boot_id':'controller','navigation_attempt_id':'attempt','worker_boot_id':'worker',
            'worker_generation':7,'path_version':1,'goal_version':0,'request_id':1,'cycle_id':1,
            'scan_counter':1,'finite':True,'scan_receive_monotonic':t-.5,
            'request_dispatched_monotonic':t-.45,'worker_solve_end_monotonic':t-.4,
            'planner_result_monotonic':t-.3,'command':{'linear_x':.1,'angular_z':.2}}
        n._continuation_parent=self.parent
        n._execution_store=SimpleNamespace(active=SimpleNamespace(fields=lambda:{
            'authority_boot_id':'authority','controller_boot_id':'controller',
            'navigation_attempt_id':'attempt','path_version':1,'goal_version':0}))
    def test_positive_then_new_wall_rejection_and_cancel(self):
        n=self.node;t=self.now
        value=Continuation(self.context,1,2,t-.3,t-.05,t-.05,t-.04,t-.03,.15,-.1,.05)
        positive={'completed_sequence':1,'failure':None,'event':{'scan_counter':2,'result':asdict(value)}}
        n._continuation_worker.snapshot.return_value=positive;n._continuation_tick()
        first=json.loads(n._continuation_pub.publish.call_args.args[0].data)
        self.assertTrue(first['valid']);self.assertEqual(first['parent_request_id'],1)
        self.assertEqual(first['scan_counter'],2);self.assertEqual(first['command']['linear_x'],.15)
        n.scan_counter=3
        event={'scan_counter':3,'result':None,'scan_receive_monotonic':t-.02,
            'pose_receive_monotonic':t-.02,'validation_started_monotonic':t-.015,
            'validated_monotonic':t-.01}
        n._continuation_worker.snapshot.return_value={'completed_sequence':2,'failure':None,'event':event}
        n._continuation_tick();second=json.loads(n._continuation_pub.publish.call_args.args[0].data)
        self.assertFalse(second['valid']);self.assertEqual(second['sequence'],2)
        self.assertEqual(n._continuation_pub.publish.call_count,2)
        n._execution_store.active=None;n.scan_counter=4
        n._continuation_worker.snapshot.return_value={'completed_sequence':3,'failure':None,'event':event}
        n._continuation_tick();self.assertEqual(n._continuation_pub.publish.call_count,2)

if __name__=='__main__':unittest.main()
