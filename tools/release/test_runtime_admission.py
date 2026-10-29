import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import runtime_admission as gate


class RuntimeAdmission(unittest.TestCase):
    def test_input_change_during_sampling_refused_and_ordered(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'reviewed.json'
            path.write_text('original')
            contract = dict(inputs={str(path): hashlib.sha256(path.read_bytes()).hexdigest()},
                pins=dict(schema='r1_neupan_parameter_pins_v1', nodes={'/worker': dict(
                    parameters={'recovery': dict(type=1, value=False)})}),
                topology=dict(nodes={'/guard': dict(publishers={'/test/out': 'geometry_msgs/msg/Twist'})},
                              exclusive_publishers={'/test/out': '/guard'}))
            good = dict(ok=True, errors=[])
            def sample(*args, **kwargs):
                path.write_text('changed')
                return good
            with patch.object(gate.preflight, 'check', return_value=good), \
                 patch.object(gate.runtime_maps, 'check', return_value=good), \
                 patch.object(gate.effective_parameters, 'sample', side_effect=sample), \
                 patch.object(gate.runtime_topology, 'check', return_value=good):
                result = gate.audit(None, {}, {}, contract)
            self.assertFalse(result['ok'])
            self.assertIn('input_hash_mismatch', str(result['errors']))
            self.assertFalse(result['motion_authorized'])
            self.assertEqual(list(result['checks']), ['inputs_before', 'release_before', 'maps_before',
                'parameters', 'topology', 'maps_after', 'inputs_after', 'release_after'])

    def test_wrong_process_or_release_stops_before_ros_queries(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pins'
            path.write_text('fixed')
            contract = dict(inputs={str(path): hashlib.sha256(path.read_bytes()).hexdigest()},
                pins=dict(schema='r1_neupan_parameter_pins_v1', nodes={'/worker': dict(
                    parameters={'recovery': dict(type=1, value=False)})}),
                topology=dict(nodes={'/guard': dict(publishers={'/test/out': 'geometry_msgs/msg/Twist'})},
                              exclusive_publishers={'/test/out': '/guard'}))
            for failing_stage in ['release', 'maps']:
                good = dict(ok=True, errors=[])
                bad = dict(ok=False, errors=['identity_changed'])
                with patch.object(gate.preflight, 'check', return_value=bad if failing_stage == 'release' else good), \
                     patch.object(gate.runtime_maps, 'check', return_value=bad if failing_stage == 'maps' else good), \
                     patch.object(gate.effective_parameters, 'sample') as sample:
                    result = gate.audit(None, {}, {}, contract)
                self.assertFalse(result['ok'])
                self.assertNotIn('parameters', result['checks'])
                sample.assert_not_called()

    def test_missing_pins_never_become_permission(self):
        result = gate.audit(None, {}, {}, {})
        self.assertFalse(result['ok'])
        self.assertFalse(result['coverage_complete'])
        self.assertFalse(result['motion_authorized'])


if __name__ == '__main__':
    unittest.main()
