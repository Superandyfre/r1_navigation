"""Regression for a production scheduler failure on NumPy boolean diagnostics."""
import json
import unittest
import numpy as np
from r1_neupan_integration.n1_16_node import (
    encode_semantic, deviation_diagnostic, publish_diagnostics_safely)
from r1_neupan_integration.path_validation import max_path_deviation_m, TemporalPathDeviationGuard


class SemanticSerialization(unittest.TestCase):
    def test_failed_diagnostics_do_not_skip_next_publication(self):
        errors = []
        self.assertFalse(publish_diagnostics_safely(
            lambda: encode_semantic({'unexpected': np.inf}), errors.append))
        self.assertEqual(len(errors), 1)
        published = []
        self.assertTrue(publish_diagnostics_safely(
            lambda: published.append(encode_semantic({'cycle': 2})), errors.append))
        self.assertEqual(json.loads(published[0]), {'cycle': 2})

    def test_failed_error_report_is_also_isolated(self):
        def broken():
            raise RuntimeError('diagnostic publisher unavailable')
        self.assertFalse(publish_diagnostics_safely(broken, lambda exc: broken()))

    def test_empty_arrival_horizon_diagnostic_does_not_poison_scheduler(self):
        deviation = max_path_deviation_m(np.empty((3, 0)), np.zeros((3, 3)))
        guard = TemporalPathDeviationGuard(.1, .3, 2)
        self.assertTrue(guard.evaluate(deviation).blocked)
        value = json.loads(encode_semantic({'path_deviation_guard': {
            'max_horizon_deviation_m': deviation_diagnostic(deviation),
            'measurement_valid': bool(np.isfinite(deviation))}}))
        self.assertIsNone(value['path_deviation_guard']['max_horizon_deviation_m'])
        self.assertFalse(value['path_deviation_guard']['measurement_valid'])
        self.assertEqual(deviation_diagnostic(.12), .12)

    def test_nested_planner_values_preserve_numeric_and_boolean_types(self):
        payload = {'obstacle_motion': {'unsafe_prediction': np.bool_(False),
            'recovery_reverse_requested': np.bool_(True), 'distance': np.float64(.25)},
            'info': {'stop': np.bool_(False), 'iterations': np.int64(3)},
            'trajectory': np.array([[1., 2.], [3., 4.]])}
        with self.assertRaises(TypeError):
            json.dumps(payload, allow_nan=False)
        result = json.loads(encode_semantic(payload))
        self.assertIs(result['obstacle_motion']['unsafe_prediction'], False)
        self.assertIs(result['obstacle_motion']['recovery_reverse_requested'], True)
        self.assertEqual(result['info'], {'stop': False, 'iterations': 3})
        self.assertEqual(result['trajectory'], [[1., 2.], [3., 4.]])
        self.assertEqual(result['obstacle_motion']['distance'], .25)

    def test_invalid_numeric_and_unknown_values_still_fail(self):
        for value in (np.float32(np.nan), np.array([np.inf])):
            with self.assertRaises(ValueError):
                encode_semantic({'value': value})
        with self.assertRaises(TypeError):
            encode_semantic({'value': object()})
