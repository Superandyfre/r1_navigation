#!/usr/bin/env python3
"""Offline check of the recorded single-pose calibration, not a motion test."""
import json
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

class ConfirmedEchoSamples(unittest.TestCase):
    def test_single_pose_holdout_and_provenance(self):
        data=json.loads((ROOT/'config/confirmed_livox_link_echo.json').read_text())
        self.assertEqual(data['schema'],'confirmed_livox_link_echo_v1')
        self.assertEqual(len(data['templates']),2)
        model=data['templates'][0]
        self.assertEqual((model['frame'],model['sensor_frame']),('L_omniPicker_hand_wide3_Link','lidar_link'))
        self.assertEqual(model['radius_m'],.008)
        right=data['templates'][1]
        self.assertEqual(right['frame'],'R_omniPicker_hand_wide3_Link')
        self.assertEqual(right['radius_m'],model['radius_m'])
        self.assertEqual(right['points_m'],[], 'right side is unmeasured, not a mirrored template')
        samples=np.load(ROOT/'test/fixtures/left_gripper_confirmed_echo_20261109.npy',allow_pickle=False)
        self.assertEqual(samples.shape,(369,3))
        self.assertTrue(np.isfinite(samples).all())
        centers=np.unique(np.round(samples/.004)*.004,axis=0)
        np.testing.assert_allclose(np.asarray(model['points_m']),centers,atol=1e-8)
        # This split checks repeatability only in the recorded stationary pose.
        indices=np.random.default_rng(42).permutation(len(samples));n=int(len(samples)*.8)
        train=np.unique(np.round(samples[indices[:n]]/.004)*.004,axis=0)
        distances=np.sqrt(np.min(np.sum((samples[indices[n:],None,:]-train[None,:,:])**2,axis=2),axis=1))
        self.assertGreaterEqual(np.mean(distances<=model['radius_m']),.98)

if __name__=='__main__':unittest.main()
