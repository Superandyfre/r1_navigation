#!/usr/bin/env python3
"""Real solver, HW1 obstacle/gear controls and synthetic empty corridor."""
import os
from pathlib import Path
import time
import unittest
import failstop_chain_ros as chain
chain.CONTROLLER=Path(os.environ["NEUPAN_TEST_CONTROLLER"])
chain.GUARD=Path(os.environ["NEUPAN_TEST_GUARD"])

class Hw1ProfileContinuity(chain.FailstopChainROS):
    worker_parameters_override=dict(obstacle_motion_enabled=True,neupan_bypass_enabled=True,
        auto_gear_transition_enabled=True,auto_gear_opposite_direction_commit_cycles=4,
        command_stabilizer_enabled=False)
    guard_parameters_override=dict(collision_check_enabled=False,require_dynamic_footprint=False)

    def test_empty_corridor_does_not_intermittently_zero(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        start=time.monotonic()
        while time.monotonic()<start+10.:
            self.tick();self.assertFalse(result.done())
        rows=[(t,v,w) for t,v,w in self.outputs if t>start+.5]
        self.assertGreater(len(rows),300)
        zeros=[t for t,v,w in rows if abs(v)<1e-8 and abs(w)<1e-8]
        self.assertFalse(zeros,"unexpected zero outputs in empty corridor: "+str(zeros[:10]))
        self.future(handle.cancel_goal_async())
        print("HW1 controls, continuous nonzero samples:",len(rows))

if __name__=="__main__":unittest.main()
