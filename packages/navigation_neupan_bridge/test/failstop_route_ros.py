#!/usr/bin/env python3
"""Same real Action UUID, test-only route adapter, actual solver/smoother/Guard."""
import json
import time
import unittest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain

class SameUUIDRoute(chain.FailstopChainROS):
    controller_plugin='navigation_neupan_bridge/SameUUIDRouteController'

    def hold_source(self,hold):
        request=dict(hold_source=hold,nonce=time.time_ns())
        self.publish('inject',request)
        self.until(lambda:any(j==request for t,j in self.inject_acks))

    def exercise_route(self,timeout=False,repeated=False):
        self.assertTrue(self.real_node,'this test requires actual N116Node')
        handle=self.goal();result=handle.get_result_async()
        moving_threshold=.12 if self.worker_mode=='actual-solver' else .06
        self.until(lambda:any(v>moving_threshold for t,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        attempt=bytes(handle.goal_id.uuid).hex();old_path=self.context['path_version']
        original_grant=self.statuses[-1][1]['grant_id']
        self.assertTrue(original_grant)
        first=min(t for t,j in self.contexts if j['state']=='ACTIVE')
        self.until(lambda:time.monotonic()-first>2.1)
        self.assertGreater(self.smooth[-1][1],moving_threshold,'scenario needs a real nonzero smoother tail')
        old_source=self.sources[-1][1]
        self.hold_source(True)
        route=Path();route.header.frame_id='map'
        for x,y in [(0.,0.),(2.,.4),(4.,0.)]:
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.
            route.poses.append(p)
        pub=self.node.create_publisher(Path,chain.PREFIX+'route_update',10)
        start=time.monotonic();next_send=0.
        while self.context['path_version']==old_path and time.monotonic()-start<.4:
            if time.monotonic()>=next_send:pub.publish(route);next_send=time.monotonic()+.06
            self.tick()
        self.assertGreater(self.context['path_version'],old_path)
        self.assertEqual(self.context['navigation_attempt_id'],attempt)
        new_path=self.context['path_version']
        self.until(lambda:any(j.get('path_version')==new_path and j['state']=='PREPARE' for t,j in self.grants))
        grant=next(j for t,j in self.grants if j.get('path_version')==new_path and j['state']=='PREPARE')
        new_grant=grant['grant_id'];self.assertNotEqual(new_grant,original_grant)
        marker=grant['monotonic']
        drain=time.monotonic()+.25
        replay_count=0
        while time.monotonic()<drain:
            # Replay the exact old authenticated source; it must neither
            # authorize the new route nor reset the source-recovery deadline.
            self.publish('source',old_source);replay_count+=1
            self.tick()
        waiting=[j for t,j in self.statuses if t>=marker+.06 and j.get('grant_id')==new_grant]
        self.assertTrue(waiting)
        # A same-attempt revision may reuse the immutable old command only
        # while its original source timestamps are still fresh.
        old_expiry=min(old_source['planner_result_monotonic']+.5,
                       old_source['worker_solve_end_monotonic']+.5,
                       old_source['scan_receive_monotonic']+.75)
        for j in waiting:
            if j['output_linear_x']!=0. or j['output_angular_z']!=0.:
                self.assertEqual(j['authorization_state'],'ROUTE_HANDOVER')
                self.assertLessEqual(j['monotonic'],old_expiry+.001)
        self.assertTrue(any(j['authorization_state']!='ARMED' for j in waiting))
        self.assertTrue(any(t>marker+.03 and v>.02 for t,v,w in self.smooth),
                        'must observe an actual old smoother tail after new PREPARE')
        changed_routes=0
        if repeated:
            next_change=time.monotonic()
            while not result.done() and time.monotonic()-marker<1.7:
                if time.monotonic()>=next_change:
                    changed_routes+=1
                    route.poses[-1].pose.position.x=4.+changed_routes*.1
                    pub.publish(route);next_change=time.monotonic()+.12
                self.publish('source',old_source);replay_count+=1
                self.tick()
            self.assertTrue(result.done(),'route updates may not renew recovery timeout')
            self.assertLess(time.monotonic()-marker,1.7)
            self.assertGreater(changed_routes,2)
            self.assertGreater(self.context['path_version'],new_path+1)
        if timeout:
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_ABORTED)
            terminal=time.monotonic();self.until(lambda:time.monotonic()-terminal>.35)
            self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>old_expiry+.08))
            self.assertFalse(any(j['state']=='COMMIT' and j['grant_id']==new_grant for t,j in self.grants))
        else:
            self.hold_source(False)
            self.until(lambda:any(j.get('grant_id')==new_grant and j.get('authorization_state')=='ARMED'
                                  for t,j in self.statuses) or result.done())
            self.assertFalse(result.done(),'route must resume the existing Action')
            armed_at=next(t for t,j in self.statuses if j.get('grant_id')==new_grant and j.get('authorization_state')=='ARMED')
            self.until(lambda:any(t>armed_at and v>.04 for t,v,w in self.outputs) or result.done())
            self.assertFalse(result.done(),'new route must produce motion after authorization')
            resumed_at=time.monotonic()
            handover_outputs=[(t,v,w) for t,v,w in self.outputs if marker+.04<t<resumed_at]
            self.assertTrue(handover_outputs)
            self.assertTrue(all(abs(v)>1e-8 or abs(w)>1e-8 for t,v,w in handover_outputs),
                            'unexpected zero during valid route handover: '+str(handover_outputs))
            matching=[(t,j) for t,j in self.statuses if j.get('grant_id')==new_grant]
            barriers=[j for t,j in matching if j.get('command_barrier_ready') and j['authorization_state'] in ('PREPARED','ROUTE_HANDOVER')]
            self.assertTrue(barriers,'real Guard must ACK drained raw+smooth command streams')
            commits=[j for t,j in self.grants if j['grant_id']==new_grant and j['state']=='COMMIT']
            self.assertTrue(commits)
            self.assertTrue(any(j['path_version']==new_path and j['navigation_attempt_id']==attempt
                                for t,j in self.sources))
            first_commit=min(j['monotonic'] for j in commits)
            self.assertLessEqual(min(j['monotonic'] for j in barriers),first_commit)
            for t,j in matching:
                if j['monotonic']<first_commit and (j['output_linear_x']!=0. or j['output_angular_z']!=0.):
                    self.assertEqual(j['authorization_state'],'ROUTE_HANDOVER')
                    self.assertLessEqual(j['monotonic'],old_expiry+.001)
            self.assertFalse(any(j['state']=='REVOKE' and j['navigation_attempt_id']==attempt
                                 for t,j in self.grants if t<first_commit))
            self.future(handle.cancel_goal_async())
            self.assertEqual(self.future(result).status,GoalStatus.STATUS_CANCELED)
        attempts={j['navigation_attempt_id'] for t,j in self.contexts}
        self.assertEqual(attempts,{attempt})
        # Action result and END context are independent DDS deliveries.
        self.until(lambda:any(j['state']=='END' for t,j in self.contexts))
        if timeout:
            self.hold_source(False)
            retry=self.goal();retry_result=retry.get_result_async()
            retry_started=time.monotonic()
            self.until(lambda:any(t>retry_started and v>moving_threshold for t,v,w in self.outputs)
                       or retry_result.done(),50)
            self.assertFalse(retry_result.done(), "new attempt must not inherit route timeout")
            self.assertNotEqual(self.context['navigation_attempt_id'],attempt)
            self.future(retry.cancel_goal_async())
        record=dict(attempt=attempt,old_path=old_path,new_path=new_path,old_grant=original_grant,
                    new_grant=new_grant,source_timeout=timeout,repeated_routes=changed_routes,
                    replay_count=replay_count,worker=self.worker_mode,physical_motion=False,
                    controller='actual NeuPANController through isolated setPlan adapter')
        (self.report/'route-result.json').write_text(json.dumps(record,indent=2))

    def test_same_uuid_route_barrier_resume(self):self.exercise_route()
    def test_same_uuid_route_source_timeout(self):self.exercise_route(True)
    def test_same_uuid_repeated_route_timeout(self):self.exercise_route(True,True)

def load_tests(loader,tests,pattern):
    return unittest.TestSuite(SameUUIDRoute(name) for name in ['test_same_uuid_route_barrier_resume',
        'test_same_uuid_route_source_timeout','test_same_uuid_repeated_route_timeout'])
if __name__=='__main__':unittest.main()
