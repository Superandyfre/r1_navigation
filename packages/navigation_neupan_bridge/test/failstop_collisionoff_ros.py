"""No duplicate Guard scan/mesh/footprint checks; real Worker still consumes scan."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
import failstop_chain_ros as chain


class CollisionOffChain(chain.FailstopChainROS):
    guard_parameters_override = dict(collision_check_enabled=False, require_dynamic_footprint=True,
        use_exact_mesh_components=True, mesh_self_filter_preapplied=False,
        scan_topic=chain.PREFIX + 'guard_scan_never_published',
        dynamic_footprint_topic=chain.PREFIX + 'guard_footprint_never_published',
        mesh_components_topic=chain.PREFIX + 'guard_mesh_never_published')

    def test_no_geometry_dependency_and_immutable_profile(self):
        handle = self.goal()
        result = handle.get_result_async()
        self.until(lambda: any(v > .04 for _, v, _ in self.outputs) or result.done(), 50.)
        self.assertFalse(result.done(), 'Guard still gates motion on duplicate collision inputs')
        self.until(lambda: self.statuses[-1][1]['reason'] == 'collision_check_disabled')
        self.assertFalse(self.statuses[-1][1]['collision_check_enabled'])
        topics = dict(self.node.get_subscriber_names_and_types_by_node('r1_collision_guard', '/'))
        for topic in ('guard_scan_never_published', 'guard_footprint_never_published', 'guard_mesh_never_published'):
            self.assertNotIn(chain.PREFIX + topic, topics)
        client = self.node.create_client(SetParameters, '/r1_collision_guard/set_parameters')
        getter = self.node.create_client(GetParameters, '/r1_collision_guard/get_parameters')
        try:
            self.until(client.service_is_ready)
            response = self.future(client.call_async(SetParameters.Request(parameters=[Parameter(
                name='collision_check_enabled', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))])))
            self.assertFalse(response.results[0].successful)
            self.until(getter.service_is_ready)
            actual = self.future(getter.call_async(GetParameters.Request(names=['collision_check_enabled'])))
            self.assertFalse(actual.values[0].bool_value)
            (self.report / 'collisionoff-result.json').write_text(json.dumps(dict(
                no_guard_scan_mesh_or_footprint_published=True, worker_scan_still_required=True,
                reenable_refused=True, reason=response.results[0].reason, physical_motion=False), indent=2))
        finally:
            self.node.destroy_client(client)
            self.node.destroy_client(getter)
        self.future(handle.cancel_goal_async())
        self.assertEqual(self.future(result).status, GoalStatus.STATUS_CANCELED)

    def test_fatal_still_stops(self):
        self.exercise('fatal')

    def test_worker_scan_loss_still_stops(self):
        self.exercise('source_freeze')


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(CollisionOffChain(name) for name in (
        'test_no_geometry_dependency_and_immutable_profile', 'test_fatal_still_stops',
        'test_worker_scan_loss_still_stops'))


if __name__ == '__main__':
    unittest.main()
