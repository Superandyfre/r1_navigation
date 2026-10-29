"""Regression for a production scheduler failure on NumPy boolean diagnostics."""
import json
import unittest
import numpy as np
from r1_neupan_integration.n1_16_node import encode_semantic


class SemanticSerialization(unittest.TestCase):
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
