"""Actual Controller auto-arm, zero before goal, and no fatal auto-recovery."""
import time
import unittest
import failstop_chain_ros as chain


class AutoArmChain(chain.FailstopChainROS):
    controller_parameters_override = dict(auto_arm_on_ready=True)
    guard_parameters_override = dict(collision_check_enabled=False)

    def arm_runtime(self):
        # Never call the arm service. Fixture supplies stopped odometry/Guard
        # and disables localization for this isolated test.
        self.until(lambda: any(row.get('run_armed') is True for _, row in self.runtime_statuses), 3.)
        self.assertTrue(self.outputs)
        self.assertTrue(all(v == 0. and w == 0. for _, v, w in self.outputs))

    def test_automatic_start_and_fatal_stop(self):
        self.exercise('fatal')
        start = time.monotonic()
        self.until(lambda: time.monotonic() - start > .7)
        status = self.runtime_statuses[-1][1]
        self.assertTrue(status['fault_latched'])
        self.assertFalse(status['run_armed'])


def load_tests(loader, tests, pattern):
    return unittest.TestSuite([AutoArmChain('test_automatic_start_and_fatal_stop')])
