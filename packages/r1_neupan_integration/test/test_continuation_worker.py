"""Actual spawned NeuPAN revalidator; no ROS, driver or physical motion."""
import time
import unittest
from pathlib import Path
import numpy as np
from r1_neupan_integration.continuation_worker import ContinuationWorker, _result_is_relevant

ROOT=Path(__file__).resolve().parents[3]

class ContinuationWorkerTest(unittest.TestCase):
    def test_older_confirmed_obstacle_is_not_hidden_by_a_queued_scan(self):
        event={'type':'result','epoch':3,'sequence':8,'result':None,'outcome':'obstacle_conflict'}
        self.assertTrue(_result_is_relevant(event,3,9,7))
        self.assertFalse(_result_is_relevant(dict(event,outcome='unavailable'),3,9,7))
        self.assertFalse(_result_is_relevant(dict(event,result={'valid':True}),3,9,7))
        self.assertFalse(_result_is_relevant(event,4,9,7))
        self.assertFalse(_result_is_relevant(event,3,9,8))

    def test_fresh_arm_geometry_revalidates_old_trajectory(self):
        worker=ContinuationWorker(ROOT/'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml',
            ROOT/'packages/r1_neupan_integration/config/models/dune_model_5000.pth')
        self.addCleanup(worker.close)
        deadline=time.monotonic()+25
        while time.monotonic()<deadline and not worker.snapshot()['ready'] and not worker.snapshot()['failure']:
            time.sleep(.02)
        self.assertTrue(worker.snapshot()['ready'],worker.snapshot())
        context=('authority','controller','attempt',1,0,'worker',1)
        worker.install(context=context,request_id=1,scan_counter=1,
            started_monotonic=time.monotonic()-.1,controls=np.vstack((np.full(20,.2),np.zeros(20))))
        now=time.monotonic()
        snapshot=dict(schema="r1_arm_geometry_v1",stamp_ns=1000000000,sequence=1,geometry_version=1,
            components=[dict(frame="left_link7",polygon_xy=[[.4,.4],[.8,.4],[.8,.6],[.4,.6]],
                             z_min=0.,z_max=1.)])
        self.assertTrue(worker.submit(context=context,scan_counter=2,
            scan_received_monotonic=now-.01,pose_received_monotonic=now-.01,input_timeout=.75,
            state=np.zeros((3,1)),points=np.array([[.9],[.5]]),
            geometry=dict(snapshot=snapshot,received_monotonic=now)))
        sequence=worker.snapshot()['sequence'];deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            result=worker.snapshot()
            self.assertIsNone(result['failure'],result)
            if result['completed_sequence']==sequence: break
            time.sleep(.01)
        self.assertEqual(result['completed_sequence'],sequence,result)
        self.assertIsNone(result['result'])
        self.assertEqual(result['event']['outcome'],'obstacle_conflict')
        self.assertEqual(result['event']['geometry']['snapshot']['sequence'],1)

    def test_new_wall_and_cancel_do_not_reuse_previous_result(self):
        worker=ContinuationWorker(ROOT/'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml',
            ROOT/'packages/r1_neupan_integration/config/models/dune_model_5000.pth')
        self.addCleanup(worker.close)
        deadline=time.monotonic()+25
        while time.monotonic()<deadline and not worker.snapshot()['ready'] and not worker.snapshot()['failure']:
            time.sleep(.02)
        self.assertTrue(worker.snapshot()['ready'],worker.snapshot())
        context=('authority','controller','attempt',1,0,'worker',1)
        controls=np.vstack((np.full(20,-.2),np.zeros(20)))
        worker.install(context=context,request_id=1,scan_counter=1,
                       started_monotonic=time.monotonic()-.1,controls=controls)
        def scene(scan,x):
            now=time.monotonic()
            points=np.vstack((np.full(11,x),np.linspace(-.4,.4,11)))
            self.assertTrue(worker.submit(context=context,scan_counter=scan,
                scan_received_monotonic=now-.01,pose_received_monotonic=now-.01,input_timeout=.75,
                state=np.zeros((3,1)),points=points))
            sequence=worker.snapshot()['sequence'];end=time.monotonic()+3
            while time.monotonic()<end:
                snapshot=worker.snapshot()
                self.assertIsNone(snapshot['failure'],snapshot)
                if snapshot['completed_sequence']==sequence:return snapshot['result']
                time.sleep(.01)
            self.fail('validator timed out')
        result=scene(2,-2.)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['linear_x'],-.2)
        self.assertIsNone(scene(3,-.43),'new rear wall must invalidate old reverse trajectory')
        event=worker.snapshot()['event']
        self.assertEqual(event['scan_counter'],3)
        self.assertIsNone(event['result'])
        self.assertEqual(event['outcome'],'obstacle_conflict')
        self.assertLessEqual(event['validation_started_monotonic'],event['validated_monotonic'])
        worker.clear()
        self.assertIsNone(worker.snapshot()['result'])
        now=time.monotonic()
        self.assertFalse(worker.submit(context=context,scan_counter=4,scan_received_monotonic=now,
            pose_received_monotonic=now,input_timeout=.75,state=np.zeros((3,1)),points=np.empty((2,0))))

if __name__=='__main__':unittest.main()
