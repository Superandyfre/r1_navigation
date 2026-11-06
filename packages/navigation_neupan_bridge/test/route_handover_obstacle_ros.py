#!/usr/bin/env python3
"""Real solver: a new avoidable obstacle while a same-goal route is revised."""
import json
import time
import unittest
from route_handover_continuity_ros import RouteHandoverContinuity

class ObstacleDuringHandover(RouteHandoverContinuity):
    worker_parameters_override=dict(obstacle_motion_enabled=True,neupan_bypass_enabled=True,
        auto_gear_transition_enabled=True,auto_gear_opposite_direction_commit_cycles=4,
        command_stabilizer_enabled=False)
    guard_parameters_override=dict(collision_check_enabled=False,require_dynamic_footprint=False)

    def test_new_obstacle_and_route_revision_keep_fresh_output(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle,result,changed=self.begin_handover()
        original=self.pubs["worker_scan"]
        published=[]
        class ObstacleScan:
            def publish(self,msg):
                ranges=list(msg.ranges)
                for index in range(154,161): ranges[index]=1.5
                msg.ranges=ranges
                published.append(time.monotonic())
                original.publish(msg)
        self.pubs["worker_scan"]=ObstacleScan()
        self.source_enabled=True
        self.until(lambda:any(j.get("path_version")==self.context["path_version"] and
                             j.get("scan_receive_monotonic",0)>changed
                             for _,j in self.sources) or result.done(),8.)
        self.assertFalse(result.done())
        self.until(lambda:any(j.get("authorization_state")=="ARMED" and
                             j.get("path_version")==self.context["path_version"]
                             for t,j in self.statuses if t>changed) or result.done(),5.)
        self.assertFalse(result.done())
        observe_until=time.monotonic()+.7
        while time.monotonic()<observe_until and not result.done(): self.tick()
        self.assertFalse(result.done())
        end=time.monotonic()
        self.assertGreater(len(published),1)
        rows=[(t,v,w) for t,v,w in self.outputs if changed+.04<t<end]
        self.assertGreater(len(rows),25)
        self.assertTrue(all(abs(v)>1e-8 or abs(w)>1e-8 for _,v,w in rows),rows)
        self.future(handle.cancel_goal_async());self.future(result)
        (self.report/"new-obstacle-handover.json").write_text(json.dumps(dict(
            obstacle_m=1.5, obstacle_scan_count=len(published),samples=len(rows),
            zero_samples=0,physical_motion=False),indent=2))

if __name__=="__main__":
    unittest.main()
