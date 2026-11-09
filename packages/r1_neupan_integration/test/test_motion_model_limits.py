"""NeuPAN velocity constraints must be achievable by the configured smoother."""
import numpy as np
import cvxpy as cp
import pytest
from neupan.robot.robot import robot


def test_constraints_use_accel_and_decel_in_both_gears():
    body = robot(receding=3, step_time=.25, kinematics="diff", length=.682, width=.532,
                 max_speed=[.7, .25], max_reverse_speed=.3,
                 max_accel=[.4, .6], max_decel=[.3, .8])
    assert body.acce_bound[:, 0] == pytest.approx([.075, .15])
    body.define_variable(no_obs=True)
    body.state_parameter_define()
    body.para_s.value = np.zeros((3, 4))
    body.para_first_step_bound.value = body.acce_bound
    for initial in (.5, -.2, 0.):
        body.para_measured_u.value = np.array([[initial], [0.]])
        for direction in (-1., 1.):
            problem = cp.Problem(cp.Maximize(direction * body.indep_u[0, 2]),
                                 body.bound_su_constraints())
            problem.solve(solver="ECOS")
            assert problem.status == cp.OPTIMAL
            commands = np.r_[initial, body.indep_u.value[0]]
            assert np.max(np.abs(np.diff(commands))) <= .07501
    with pytest.raises(ValueError):
        robot(receding=3, kinematics="diff", length=.682, width=.532,
              max_accel=[-.1, .6], max_decel=[.3, .8])
