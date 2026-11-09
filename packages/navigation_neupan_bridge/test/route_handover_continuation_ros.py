#!/usr/bin/env python3
"""Real NeuPAN, Controller, smoother and Guard; no hardware or live DDS."""
import os
import json
from pathlib import Path
import time
import unittest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String
import failstop_chain_ros as chain

chain.CONTROLLER=Path(os.environ["NEUPAN_TEST_CONTROLLER"])
chain.GUARD=Path(os.environ["NEUPAN_TEST_GUARD"])
chain.SMOOTHER=Path(os.environ["NEUPAN_TEST_SMOOTHER"])

class RouteContinuation(chain.FailstopChainROS):
    controller_plugin="navigation_neupan_bridge/SameUUIDRouteController"
    worker_parameters_override=dict(auto_gear_transition_enabled=False,continuation_enabled=True)
    controller_parameters_override=dict(continuation_enabled=True,max_linear=.5,max_reverse=.5,
                                        envelope_recovery_timeout=5.)
    guard_parameters_override=dict(continuation_enabled=True,max_linear_mps=.5,max_reverse_mps=.5,
                                   planner_recovery_timeout_sec=5.)
    smoother_parameters_override=dict(max_velocity=[.5,0.,.4],min_velocity=[-.5,0.,-.4])
    expected_plugin_root=os.environ["NEUPAN_TEST_ROUTE_PLUGIN_ROOT"]
    worker_environment=dict(os.environ,NEUPAN_TEST_ROUTE_SOLVE_DELAY="0.8",
                            NEUPAN_TEST_CYCLE_SOLVE_DELAY="0.15")

    def setUp(self):
        super().setUp()
        self.proofs=[]
        self.proof_sub=self.node.create_subscription(String,"/neupan_trajectory_proof",
            lambda msg:self.proofs.append((time.monotonic(),json.loads(msg.data))),30)
        self.addCleanup(lambda:(self.report/"trajectory-proofs.json").write_text(json.dumps(self.proofs)))
        original=self.pubs["worker_scan"];node=self.node
        class DelayedScan:
            last=0.
            def publish(self, msg):
                from rclpy.duration import Duration
                now=time.monotonic()
                if now-self.last<.10:return
                self.last=now
                msg.header.stamp=(node.get_clock().now()-Duration(seconds=.08)).to_msg()
                original.publish(msg)
        self.pubs["worker_scan"]=DelayedScan()

    def test_temporary_worker_heartbeat_gap_does_not_latch_navigation(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50.)
        self.assertFalse(result.done())
        self.publish("inject",dict(hold_health=True))
        self.until(lambda:any(v.get("hold_health") is True for _,v in self.inject_acks),3.)
        self.until(lambda:any(v.get("authorization_state")=="WORKER_HEALTH_HOLD" for _,v in self.statuses),4.)
        self.assertFalse(result.done())
        self.publish("inject",dict(hold_health=False));marker=time.monotonic()
        self.until(lambda:any(v.get("hold_health") is False for _,v in self.inject_acks),3.)
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or result.done(),6.)
        self.assertFalse(result.done())
        self.assertFalse(any(v.get("fault_latched") for _,v in self.runtime_statuses))
        self.future(handle.cancel_goal_async());self.future(result)
        second=self.goal();second_result=second.get_result_async();marker=time.monotonic()
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or second_result.done(),40.)
        self.assertFalse(second_result.done())
        self.future(second.cancel_goal_async());self.future(second_result)

    def test_new_goal_after_extended_heartbeat_gap(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50.)
        self.assertFalse(result.done())
        self.publish("inject",dict(hold_health=True))
        self.until(lambda:any(v.get("hold_health") is True for _,v in self.inject_acks),3.)
        self.until(result.done,12.)
        self.assertFalse(any(v.get("fault_latched") for _,v in self.runtime_statuses))
        self.publish("inject",dict(hold_health=False))
        self.until(lambda:any(v.get("hold_health") is False for _,v in self.inject_acks),3.)
        self.until(lambda:any(v.get("authorization_state")=="DISARMED" for _,v in self.statuses[-10:]),4.)
        second=self.goal();second_result=second.get_result_async();marker=time.monotonic()
        self.until(lambda:any(t>marker and v>.04 for t,v,w in self.outputs) or second_result.done(),40.)
        self.assertFalse(second_result.done())
        self.future(second.cancel_goal_async());self.future(second_result)

    def test_fresh_old_trajectory_survives_slow_same_attempt_route_update(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50.)
        self.assertFalse(result.done())
        self.until(lambda:any(s.get("continuation_parent_request_id",0)>0 for _,s in self.statuses),10.)
        self.until(lambda:time.monotonic()-self.contexts[0][0]>2.1)
        old=self.context["path_version"]
        route=NavPath();route.header.frame_id="map"
        for x,y in ((0.,0.),(2.,.4),(4.,0.)):
            p=PoseStamped();p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.w=1.;route.poses.append(p)
        pub=self.node.create_publisher(NavPath,chain.PREFIX+"route_update",10)
        start=time.monotonic()
        while self.context["path_version"]==old and time.monotonic()-start<1.:
            pub.publish(route);self.tick()
        self.assertGreater(self.context["path_version"],old)
        changed=time.monotonic();deadline=changed+1.5
        while time.monotonic()<deadline and not result.done():self.tick()
        self.assertFalse(result.done())
        rows=[(t,v,w) for t,v,w in self.outputs if changed+.08<t<deadline-.05]
        self.assertGreater(len(rows),40)
        self.assertTrue(all(abs(v)>1e-8 or abs(w)>1e-8 for _,v,w in rows),rows)
        self.assertTrue(any(t>changed+.5 and s.get("authorization_state")=="ROUTE_HANDOVER"
            for t,s in self.statuses))
        self.assertTrue(any(t>changed and s.get("path_version")==self.context["path_version"]
            for t,s in self.sources))
        self.future(handle.cancel_goal_async());self.future(result)
        log=(self.report/"actual_worker_node.log").read_text()
        self.assertIn("ISOLATED_ROUTE_SOLVE_DELAY",log)

if __name__=="__main__":unittest.main()
