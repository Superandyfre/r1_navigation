#!/usr/bin/env python3
"""Same-attempt route handover with real Controller, smoother and Guard."""
import os
from pathlib import Path
import time
import unittest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
import failstop_chain_ros as chain
chain.CONTROLLER=Path(os.environ["NEUPAN_TEST_CONTROLLER"])
chain.GUARD=Path(os.environ["NEUPAN_TEST_GUARD"])

class RouteHandoverContinuity(chain.FailstopChainROS):
    controller_plugin="navigation_neupan_bridge/SameUUIDRouteController"
    def begin_handover(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done())
        self.assertFalse(result.done())
        self.until(lambda:time.monotonic()-self.contexts[0][0]>2.1)
        old_path=self.context["path_version"]
        route=NavPath();route.header.frame_id="map"
        for x,y in ((0.,0.),(2.,.4),(4.,0.)):
            p=PoseStamped();p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;route.poses.append(p)
        pub=self.node.create_publisher(NavPath,chain.PREFIX+"route_update",10)
        self.source_enabled=False
        marker=time.monotonic()
        while self.context["path_version"]==old_path and time.monotonic()-marker<.35:
            pub.publish(route);self.tick()
        self.assertGreater(self.context["path_version"],old_path)
        return handle,result,time.monotonic()

    def test_different_fresh_target_transitions_without_zero(self):
        handle,result,changed=self.begin_handover()
        while time.monotonic()<changed+.1:self.tick()
        self.synthetic_linear=.16;self.source_enabled=True
        self.until(lambda:any(j.get("authorization_state")=="ARMED" and
                             j.get("path_version")==self.context["path_version"]
                             for t,j in self.statuses if t>changed) or result.done(),5)
        self.assertFalse(result.done())
        self.until(lambda:any(t>changed and v>.12 for t,v,w in self.outputs) or result.done(),5)
        self.assertFalse(result.done())
        rows=[v for t,v,w in self.outputs if t>changed+.04]
        self.assertTrue(rows)
        self.assertTrue(all(v>.04 for v in rows),rows)
        self.future(handle.cancel_goal_async())

    def test_old_result_expiration_stops_without_renewal(self):
        handle,result,changed=self.begin_handover()
        self.until(lambda:any(t>changed+.55 and abs(v)<1e-8 for t,v,w in self.outputs),2)
        end=time.monotonic()+.1
        while time.monotonic()<end:self.tick()
        self.assertFalse(any(t>changed+.65 and abs(v)>1e-8 for t,v,w in self.outputs))
        self.assertFalse(result.done())
        self.future(handle.cancel_goal_async())

    def test_worker_fault_during_handover_stops_and_aborts(self):
        handle,result,changed=self.begin_handover()
        self.fatal=True
        from action_msgs.msg import GoalStatus
        self.assertEqual(self.future(result,5).status,GoalStatus.STATUS_ABORTED)
        terminal=time.monotonic()
        while time.monotonic()<terminal+.25:self.tick()
        self.assertFalse(any(t>terminal+.08 and (abs(v)>1e-8 or abs(w)>1e-8) for t,v,w in self.outputs))

    def test_identical_new_target_has_no_zero_and_cancel_revokes(self):
        self.assertFalse(self.real_node)
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done())
        self.assertFalse(result.done())
        self.until(lambda:time.monotonic()-self.contexts[0][0]>2.1)
        old_path=self.context["path_version"]
        old_source=dict(self.sources[-1][1]);old_attempt=self.context["navigation_attempt_id"]
        route=NavPath();route.header.frame_id="map"
        for x,y in ((0.,0.),(2.,.4),(4.,0.)):
            p=PoseStamped();p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;route.poses.append(p)
        pub=self.node.create_publisher(NavPath,chain.PREFIX+"route_update",10)
        self.source_enabled=False
        self.until(lambda:len(self.outputs)>10)
        marker=time.monotonic()
        while self.context["path_version"]==old_path and time.monotonic()-marker<.35:
            pub.publish(route);self.tick()
        self.assertGreater(self.context["path_version"],old_path)
        changed=time.monotonic()
        while time.monotonic()<changed+.12:self.tick()
        self.source_enabled=True
        self.until(lambda:any(t>changed and j.get("authorization_state")=="ARMED" and
                            j.get("path_version")==self.context["path_version"] for t,j in self.statuses),3)
        end=time.monotonic()
        rows=[(t,v,w) for t,v,w in self.outputs if marker+.04<t<end]
        self.assertGreater(len(rows),3)
        self.assertTrue(all(v>.04 for _,v,_ in rows),rows)
        self.assertFalse(result.done())
        self.future(handle.cancel_goal_async());self.future(result)
        terminal=time.monotonic()
        while time.monotonic()<terminal+.4:
            self.publish("source",old_source);self.tick()
        self.assertTrue(all(abs(v)<1e-8 and abs(w)<1e-8 for t,v,w in self.outputs if t>terminal+.08))
        self.assertEqual(old_attempt,self.context["navigation_attempt_id"])

if __name__=="__main__":unittest.main()
