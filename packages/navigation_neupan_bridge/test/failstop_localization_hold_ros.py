#!/usr/bin/env python3
"""Isolated real Controller/Guard regression for footprint synchronization HOLD."""
import json
import os
from pathlib import Path
import time
import unittest
from std_msgs.msg import String
import failstop_chain_ros as chain

chain.CONTROLLER = Path(os.environ["NEUPAN_TEST_CONTROLLER"])
chain.GUARD = Path(os.environ["NEUPAN_TEST_GUARD"])

class LocalizationHoldChain(chain.FailstopChainROS):
    def setUp(self):
        self.localization_state = "READY"
        self.localization_pub = None
        self.controller_parameters_override = dict(
            require_localization_ready=True,
            localization_health_topic=chain.PREFIX + "localization")
        self.guard_parameters_override = dict(
            require_localization_ready=True,
            localization_health_topic=chain.PREFIX + "localization")
        super().setUp()

    def tick(self):
        if self.localization_pub is None:
            self.localization_pub = self.node.create_publisher(String, chain.PREFIX + "localization", 20)
        self.localization_pub.publish(String(data=json.dumps(dict(
            schema="r1_localization_health_v1", state=self.localization_state,
            reason="costmap_footprint_not_synchronized" if self.localization_state == "HOLD" else "ready"))))
        super().tick()

    def test_hold_stops_and_recovers_without_terminating_action(self):
        handle = self.goal()
        result = handle.get_result_async()
        self.until(lambda: any(v > .04 for _, v, _ in self.outputs) or result.done(), 20)
        self.assertFalse(result.done())
        self.localization_state = "HOLD"
        self.until(lambda: self.statuses[-1][1]["reason"].startswith("localization_hold:"))
        marker = time.monotonic()
        while time.monotonic() < marker + .4:
            self.tick()
        self.assertFalse(result.done(), "HOLD aborted the active goal")
        self.assertTrue(all(abs(v) < 1e-8 and abs(w) < 1e-8
                            for t, v, w in self.outputs if t > marker + .05))
        self.localization_state = "READY"
        marker = time.monotonic()
        self.until(lambda: any(t > marker and v > .04 for t, v, _ in self.outputs) or result.done(), 10)
        self.assertFalse(result.done(), "goal failed during recovery")
        self.assertTrue(any(t > marker and v > .04 for t, v, _ in self.outputs))
        self.future(handle.cancel_goal_async())

if __name__ == "__main__":
    unittest.main()
