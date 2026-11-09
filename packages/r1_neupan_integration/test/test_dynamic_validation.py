import numpy as np
from neupan.dynamic_validation import validate_dynamic_candidate


def test_updated_arm_can_invalidate_old_plan_without_moving_obstacle():
    chassis=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    arm_before=[[.2,.8],[.6,.8],[.6,1.],[.2,1.]]
    arm_after=[[.2,.4],[.6,.4],[.6,.6],[.2,.6]]
    obstacle=np.array([[.9],[.5]])
    original=obstacle.copy()
    controls=np.vstack((np.full(8,.2),np.zeros(8)))
    assert validate_dynamic_candidate([chassis,arm_before],[0.,0.,0.],controls,.25,obstacle)
    assert not validate_dynamic_candidate([chassis,arm_after],[0.,0.,0.],controls,.25,obstacle)
    np.testing.assert_array_equal(obstacle,original)


def test_open_space_and_empty_observation_do_not_add_a_stop():
    chassis=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    assert validate_dynamic_candidate([chassis],[0.,0.,0.],[[.2],[0.]],.25,np.empty((2,0)))
