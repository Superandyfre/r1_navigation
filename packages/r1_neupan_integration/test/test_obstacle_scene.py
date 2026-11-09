import numpy as np
import pytest
from r1_neupan_integration.obstacle_scene import assemble_scene
from neupan.dynamic_geometry import DynamicGeometry


def test_scene_retains_points_and_separate_known_heights():
    scan=np.array([[1.,-2.],[.2,.4]])
    cloud=np.array([[1.],[.2],[1.5]])
    rear=np.array([[-2.],[.4],[.3]])
    geometry=DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
        dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=0.,z_max=1.5)])
    p,z,base,complete=assemble_scene(scan,cloud,rear,base_z_world=.1,arm_geometry=geometry,visibility_proven=True)
    np.testing.assert_array_equal(p[:,:2],scan)
    np.testing.assert_array_equal(p[:,2:],np.hstack((cloud[:2],rear[:2])))
    assert np.isnan(z[:2]).all()
    np.testing.assert_allclose(z[2:],[1.4,.2])
    assert base.tolist()==[1,1,2,2] and complete


def test_missing_sensor_keeps_fused_scan_for_arm_as_well():
    p,z,base,complete=assemble_scene(np.array([[1.],[2.]]),
                                      np.array([[1.],[2.],[3.]]),None)
    assert not complete and base.tolist()==[0,2]
    assert np.isnan(z[0])
    with pytest.raises(ValueError):
        assemble_scene(np.array([[1.],[2.]]),np.array([[1.],[np.nan],[3.]]),None)



def test_arm_observations_outside_all_measured_arm_heights_are_omitted():
    geometry=DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
        dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=1.,z_max=1.2,uncertainty_rate_mps=0.)])
    scan=np.array([[1.],[.5]])
    cloud=np.array([[1.,1.],[.5,.5],[.2,1.1]])
    rear=np.empty((3,0))
    points,z,scope,complete=assemble_scene(scan,cloud,rear,arm_geometry=geometry,visibility_proven=True)
    assert complete and points.shape[1]==2
    np.testing.assert_allclose(z[1:],[1.1])
    assert scope.tolist()==[1,2]


def test_sensor_matching_uses_original_timestamps_not_new_receipt():
    from r1_neupan_integration.obstacle_scene import select_sensor_scenes
    points=np.zeros((3,1))
    snapshot={"front":{"stamp_ns":1000000000,"points":points,"received_monotonic":999.},
              "rear":{"stamp_ns":1050000000,"points":points,"received_monotonic":999.}}
    front,rear=select_sensor_scenes(snapshot,1000000000,1100000000,.1,.2)
    assert front is points and rear is points
    front,rear=select_sensor_scenes(snapshot,1000000000,1300000000,.1,.2)
    assert front is None and rear is None
    front,rear=select_sensor_scenes(snapshot,1000000000,1100000000,.01,.2)
    assert front is points and rear is None


def test_receiving_both_sensors_does_not_prove_arm_height_visibility():
    geometry=DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
        dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=1.,z_max=1.2)])
    scan=np.array([[1.],[.5]])
    empty=np.empty((3,0))
    _,_,scope,complete=assemble_scene(scan,empty,empty,arm_geometry=geometry)
    assert not complete and scope.tolist()==[0]


def test_obstacle_height_is_relative_to_actual_base_orientation():
    pose=np.eye(4)
    pose[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    pose[:3,3]=[0.,2.,3.]
    # World y=1.5 is +.5 along the tilted base z axis.
    cloud=np.array([[1.],[1.5],[3.]])
    _,z,_,_=assemble_scene(np.empty((2,0)),cloud,None,base_pose_world=pose)
    assert z[0] == pytest.approx(.5)


def test_future_arm_height_and_unknown_motion_preserve_observations():
    base=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    arm=dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=1.,z_max=1.2,uncertainty_rate_mps=.2)
    cloud=np.array([[1.,1.],[.5,.5],[1.45,1.9]])
    points,_,_,_=assemble_scene(np.empty((2,0)),cloud,arm_geometry=DynamicGeometry([base,arm]),horizon_sec=1.5)
    assert points.shape[1] == 1
    arm["uncertainty_rate_mps"]=None
    points,_,_,_=assemble_scene(np.empty((2,0)),cloud,arm_geometry=DynamicGeometry([base,arm]),horizon_sec=1.5)
    assert points.shape[1] == 2
