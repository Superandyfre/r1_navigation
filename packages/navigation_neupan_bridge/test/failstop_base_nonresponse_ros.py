"""Actual final nonzero + fresh stationary odometry must end the action."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
import failstop_chain_ros as chain


class BaseNonresponse(chain.FailstopChainROS):
    controller_parameters_override=dict(detect_base_nonresponse=True)
    guard_parameters_override=dict()

    def test_actual_nonresponse_is_bounded_without_stronger_motion(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,w in self.outputs) or result.done(),50)
        self.assertFalse(result.done())
        start=time.monotonic()
        self.assertEqual(self.future(result,4).status,GoalStatus.STATUS_ABORTED)
        end=time.monotonic();self.until(lambda:time.monotonic()>end+.4)
        self.assertTrue(all(v==0 and w==0 for t,v,w in self.outputs if t>end+.1))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        (self.report/'nonresponse-result.json').write_text(json.dumps(dict(
            terminal_seconds=end-start,stationary_synthetic_odometry=True,
            final_nonzero_observed=True,physical_motion=False),indent=2))


def load_tests(loader,tests,pattern):
    return unittest.TestSuite([BaseNonresponse('test_actual_nonresponse_is_bounded_without_stronger_motion')])
