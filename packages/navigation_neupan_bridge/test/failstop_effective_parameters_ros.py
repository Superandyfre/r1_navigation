"""Real isolated chain parameter checks before its test-only arm request."""
import copy
import json
from pathlib import Path
import sys
import unittest

from action_msgs.msg import GoalStatus
import yaml
import failstop_chain_ros as chain

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools/release'))
from effective_parameters import SCHEMA, sample
from runtime_topology import check as check_topology, control_contract


def flatten(values, prefix=''):
    result = {}
    for name, value in values.items():
        key = prefix + name
        if isinstance(value, dict):
            result.update(flatten(value, key + '.'))
        else:
            kinds = {bool: 1, int: 2, float: 3, str: 4}
            kind = kinds[type(value)] if type(value) is not list else kinds[type(value[0])] + 5
            result[key] = dict(type=kind, value=value)
    return result


class EffectiveParameters(chain.FailstopChainROS):
    def arm_runtime(self):
        # Expectations come from fixture launch inputs and independently named
        # A+ defaults, never learned from the live parameter server.
        config = yaml.safe_load((self.report / 'params.yaml').read_text())
        pins = dict(schema=SCHEMA, nodes={})
        for name in ('controller_server', 'velocity_smoother', 'r1_collision_guard'):
            pins['nodes']['/' + name] = dict(parameters=flatten(config[name]['ros__parameters']))
        pins['nodes']['/n1_16_production_bridge'] = dict(parameters={
            'worker_recovery_enabled': dict(type=1, value=False),
            'require_execution_context': dict(type=1, value=True),
            'worker_health_period_sec': dict(type=3, value=.1),
            'worker_health_topic': dict(type=4, value=chain.PREFIX + 'health'),
            'execution_context_topic': dict(type=4, value=chain.PREFIX + 'context'),
            'planner_result_topic': dict(type=4, value=chain.PREFIX + 'source')})
        (self.report / 'parameter-pins.json').write_text(json.dumps(pins, indent=2))
        result = sample(self.node, pins, 5., pump=self.tick)
        (self.report / 'parameters-before-arm.json').write_text(json.dumps(result, indent=2))
        self.assertTrue(result['ok'], result['errors'])
        self.assertFalse(any(v or w for _, v, w in self.outputs))
        bad = copy.deepcopy(pins)
        bad['nodes']['/controller_server']['parameters']['FollowPath.shadow_mode']['value'] = True
        rejected = sample(self.node, bad, 5., pump=self.tick)
        (self.report / 'wrong-pins-refused.json').write_text(json.dumps(rejected, indent=2))
        self.assertIn('/controller_server:FollowPath.shadow_mode:mismatch', rejected['errors'])
        self.assertFalse(any(v or w for _, v, w in self.outputs))
        # Base peer precreates unused source/health publishers even in actual
        # Worker mode. Remove those test-only endpoints before exclusivity checks.
        self.node.destroy_publisher(self.pubs['source'])
        self.node.destroy_publisher(self.pubs['health'])
        remaps = {'/controller_server': [('cmd_vel', chain.PREFIX + 'raw')],
                  '/velocity_smoother': [('cmd_vel', chain.PREFIX + 'raw'),
                                         ('cmd_vel_smoothed', chain.PREFIX + 'smooth')]}
        topology = control_contract(pins, remaps, smoother='/velocity_smoother')
        observations = []
        def ready():
            result = check_topology(self.node, topology)
            observations.append(result)
            return result['ok']
        self.until(ready, 3.)
        (self.report / 'topology-before-arm.json').write_text(json.dumps(observations, indent=2))
        wrong = copy.deepcopy(topology)
        wrong['nodes']['/controller_server']['publishers'][chain.PREFIX + 'wrong_raw'] = 'geometry_msgs/msg/Twist'
        mismatch = check_topology(self.node, wrong)
        self.assertFalse(mismatch['ok'])
        from geometry_msgs.msg import Twist
        rogue = self.node.create_publisher(Twist, chain.PREFIX + 'output', 10)
        try:
            self.until(lambda: len(self.node.get_publishers_info_by_topic(chain.PREFIX + 'output')) == 2, 3.)
            duplicated = check_topology(self.node, topology)
            self.assertIn(chain.PREFIX + 'output:publisher_not_exclusive', duplicated['errors'])
            (self.report / 'topology-refused.json').write_text(json.dumps(dict(
                wrong_remap=mismatch, extra_final_publisher=duplicated), indent=2))
        finally:
            self.node.destroy_publisher(rogue)
        self.until(ready, 3.)
        self.assertFalse(any(v or w for _, v, w in self.outputs))
        super().arm_runtime()

    def test_parameters_before_arm_and_normal_cancel(self):
        self.assertEqual(self.worker_mode, 'actual-solver')
        handle = self.goal()
        result = handle.get_result_async()
        self.until(lambda: any(v > .04 for _, v, _ in self.outputs) or result.done(), 50.)
        self.assertFalse(result.done())
        self.future(handle.cancel_goal_async())
        self.assertEqual(self.future(result).status, GoalStatus.STATUS_CANCELED)


def load_tests(loader, tests, pattern):
    return unittest.TestSuite([EffectiveParameters('test_parameters_before_arm_and_normal_cancel')])


if __name__ == '__main__':
    unittest.main()
