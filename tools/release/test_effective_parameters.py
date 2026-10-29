import copy
import unittest
from effective_parameters import SCHEMA, compare, validate


class ParameterPins(unittest.TestCase):
    def setUp(self):
        self.pins = dict(schema=SCHEMA, nodes={'/controller_server': dict(
            parameters={'FollowPath.shadow_mode': dict(type=1, value=False),
                        'FollowPath.worker_health_timeout': dict(type=3, value=.5)},
            closed_prefixes=['FollowPath'])})
        self.observed = {'/controller_server': dict(
            parameters=copy.deepcopy(self.pins['nodes']['/controller_server']['parameters']),
            names=['FollowPath.shadow_mode', 'FollowPath.worker_health_timeout', 'use_sim_time'])}

    def test_match_and_closed_namespace(self):
        self.assertEqual(compare(self.pins, self.observed), [])
        self.observed['/controller_server']['names'].append('FollowPath.new_unreviewed_timeout')
        self.assertIn('/controller_server:FollowPath.new_unreviewed_timeout:unpinned',
                      compare(self.pins, self.observed))

    def test_missing_node_or_parameter_fails(self):
        self.assertEqual(len(compare(self.pins, {})), 2)
        del self.observed['/controller_server']['parameters']['FollowPath.shadow_mode']
        self.assertIn('/controller_server:FollowPath.shadow_mode:missing',
                      compare(self.pins, self.observed))

    def test_no_bool_integer_or_numeric_type_coercion(self):
        row = self.observed['/controller_server']['parameters']
        for value in [dict(type=2, value=0), dict(type=1, value=0), dict(type=1, value=True)]:
            row['FollowPath.shadow_mode'] = value
            self.assertIn('/controller_server:FollowPath.shadow_mode:mismatch', compare(self.pins, self.observed))
        row['FollowPath.shadow_mode'] = dict(type=1, value=False)
        for value in [dict(type=2, value=1), dict(type=3, value=float('nan')),
                      dict(type=3, value=float('inf'))]:
            row['FollowPath.worker_health_timeout'] = value
            self.assertTrue(compare(self.pins, self.observed))

    def test_all_parameter_array_types_and_invalid_pins(self):
        rows = {'bytes': dict(type=5, value=[0, 255]), 'bools': dict(type=6, value=[True, False]),
                'ints': dict(type=7, value=[-4, 7]), 'doubles': dict(type=8, value=[.1, .2]),
                'strings': dict(type=9, value=['/scan', 'odom'])}
        pins = dict(schema=SCHEMA, nodes={'/fixture': dict(parameters=rows)})
        self.assertEqual(compare(pins, {'/fixture': dict(parameters=rows)}), [])
        for kind, value in [(5, [256]), (6, [0]), (7, [True]), (7, [2**63]),
                            (8, [float('nan')]), (9, [4]), (0, None)]:
            pins['nodes']['/fixture']['parameters'] = {'bad': dict(type=kind, value=value)}
            with self.assertRaises(ValueError):
                validate(pins)


if __name__ == '__main__':
    unittest.main()
