import copy
from types import SimpleNamespace
import unittest
from runtime_topology import check, validate


class GraphFixture:
    def __init__(self):
        self.owners = [SimpleNamespace(node_name='guard', node_namespace='/')]
    def get_node_names_and_namespaces(self):
        return [('guard', '/')]
    def get_publisher_names_and_types_by_node(self, name, namespace):
        return [('/test/out', ['geometry_msgs/msg/Twist'])]
    def get_subscriber_names_and_types_by_node(self, name, namespace):
        return [('/test/in', ['geometry_msgs/msg/Twist'])]
    def get_publishers_info_by_topic(self, topic):
        return self.owners


class RuntimeTopology(unittest.TestCase):
    def setUp(self):
        self.contract = dict(nodes={'/guard': dict(publishers={'/test/out': 'geometry_msgs/msg/Twist'},
                            subscriptions={'/test/in': 'geometry_msgs/msg/Twist'})},
                            exclusive_publishers={'/test/out': '/guard'})
        self.node = GraphFixture()

    def test_match_and_extra_output_publisher_refused(self):
        self.assertTrue(check(self.node, self.contract)['ok'])
        self.node.owners.append(SimpleNamespace(node_name='rogue', node_namespace='/'))
        self.assertIn('/test/out:publisher_not_exclusive', check(self.node, self.contract)['errors'])

    def test_wrong_remap_type_and_missing_node(self):
        for spec in [dict(subscriptions={'/wrong/in': 'geometry_msgs/msg/Twist'}),
                     dict(subscriptions={'/test/in': 'std_msgs/msg/String'})]:
            contract = copy.deepcopy(self.contract)
            contract['nodes']['/guard'].update(spec)
            self.assertFalse(check(self.node, contract)['ok'])
        self.node.get_node_names_and_namespaces = lambda: []
        self.assertFalse(check(self.node, self.contract)['ok'])

    def test_invalid_or_empty_contract_refused(self):
        for contract in [{}, dict(nodes={}), dict(nodes={'/guard': {}}),
                         dict(nodes={'/guard': {'publishers': {'/out': 'geometry_msgs/msg/Twist'}}},
                              exclusive_publishers={'/out': '/other'})]:
            with self.assertRaises(ValueError):
                validate(contract)


if __name__ == '__main__':
    unittest.main()
