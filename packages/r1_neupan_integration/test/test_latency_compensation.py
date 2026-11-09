"""Motion prediction remains tied to real observations and fresh odometry."""
import math
import numpy as np
import pytest
from r1_neupan_integration.latency_compensation import SolveLatency, predict_pose


def test_forward_reverse_turn_and_no_motion():
    state = np.array([[1.], [2.], [0.]])
    forward, elapsed = predict_pose(state, [[.7], [0.]], .1, .4, .75)
    assert elapsed == pytest.approx(.5)
    assert forward[:, 0] == pytest.approx([1.35, 2., 0.])
    reverse, _ = predict_pose(state, [[-.3], [0.]], .1, .4, .75)
    assert reverse[:, 0] == pytest.approx([.85, 2., 0.])
    curved, _ = predict_pose(state, [[.7], [.25]], 0., .4, .75)
    assert curved[:, 0] == pytest.approx([1.+2.8*math.sin(.1), 2.+2.8*(1-math.cos(.1)), .1])
    assert state[:, 0] == pytest.approx([1., 2., 0.])
    assert predict_pose(state, None, .1, .4, .75)[1] == 0
    assert predict_pose(state, [[.7], [0.]], .8, .1, .75)[1] == 0


def test_latency_estimator_tracks_observations_without_fabricating_samples():
    latency = SolveLatency(.75, window=3)
    assert latency.estimate() == 0
    for duration in (.1, .2, .3, .4, float("nan")):
        latency.observe(duration)
    assert list(latency.samples) == pytest.approx([.2, .3, .4])
    assert latency.estimate() == pytest.approx(.35)
    latency.observe(2.)
    assert latency.samples[-1] == .75
    pose, elapsed = predict_pose([[0.], [0.], [0.]], [[.7], [0.]], .5, .4, .75)
    assert elapsed == .75
    assert pose[0, 0] == pytest.approx(.525)
