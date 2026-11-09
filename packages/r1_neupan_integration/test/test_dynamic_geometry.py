"""Exact, non-warping component geometry for dynamic NeuPAN footprint."""
import numpy as np
import pytest
from neupan.dynamic_geometry import DynamicGeometry


def test_independent_arms_keep_gap_and_distances():
    chassis = [[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    left = [[.2,.6],[.5,.6],[.5,.8],[.2,.8]]
    right = [[.4,-.9],[.8,-.9],[.8,-.6],[.4,-.6]]
    model = DynamicGeometry([chassis,left,right])
    points = np.array([[.4,.4,.6,.0],[.5,.7,-.75,1.]])
    before = points.copy()
    distance, normal, support = model.distance_and_normal(points)
    np.testing.assert_allclose(distance,[.1,-.1,-.15,.283],atol=.002)
    assert np.all(np.isfinite(normal)) and np.all(np.isfinite(support))
    np.testing.assert_array_equal(points,before)
    # Moving left arm changes its own geometry; right is not mirrored.
    moved = DynamicGeometry([chassis,np.asarray(left)+[.4,0],right])
    updated,_,_ = moved.distance_and_normal(points)
    assert updated[1] > 0 and updated[2] == pytest.approx(distance[2])


def test_signed_distance_normals_and_support_are_consistent():
    model = DynamicGeometry([[[0,0],[2,0],[2,1],[0,1]]])
    points = np.array([[3.,1.,2.,3.],[.5,.2,.5,2.]])
    distance,normals,support = model.distance_and_normal(points)
    np.testing.assert_allclose(distance,[1.,-.2,0.,np.sqrt(2)])
    np.testing.assert_allclose(np.linalg.norm(normals,axis=0),1.)
    np.testing.assert_allclose((normals*points).sum(axis=0)-support,distance,atol=1e-9)


def test_snapshot_copies_and_rejects_invalid_geometry():
    original = np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.]])
    model = DynamicGeometry([original[::-1]])
    original[:]=100.
    assert model.distance_and_normal(np.array([[.5],[.5]]))[0][0] == pytest.approx(-.5)
    with pytest.raises(ValueError):
        DynamicGeometry([[[0,0],[1,0],[.5,.1],[1,1],[0,1]]])
    with pytest.raises(ValueError):
        DynamicGeometry([])



def test_dynamic_dune_coefficients_reproduce_geometry_distance():
    import torch
    from neupan.blocks.dynamic_dune import DynamicDUNE
    from neupan.blocks.nrmp import NRMP
    body = DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
                            [[.4,.4],[.7,.4],[.7,.6],[.4,.6]]])
    model = DynamicDUNE(body,.25)
    local = torch.tensor([[.8,.0],[.5,.5]], dtype=torch.float32)
    rotation = torch.tensor([[0.,-1.],[1.,0.]],dtype=torch.float32)
    robot_position = torch.tensor([[2.],[3.]])
    world = rotation @ local + robot_position
    offset,normal,ordered = model([local,local],[rotation,rotation],[world,world])
    nrmp = NRMP.__new__(NRMP)
    torch.nn.Module.__init__(nrmp)
    nrmp.T=1;nrmp.max_num=2;nrmp.no_obs=False;nrmp.h=torch.zeros((4,1))
    fa,fb=nrmp.generate_coefficient_parameter_value(offset,normal,ordered)
    optimized_distance=(fa @ robot_position - fb).flatten().numpy()
    expected,_,_=body.distance_and_normal(local.numpy())
    np.testing.assert_allclose(optimized_distance,sorted(expected),atol=1e-6)
    assert optimized_distance[0] == pytest.approx(.1,abs=1e-6)


def test_dynamic_neupan_distances_change_with_arm_pose_not_points():
    from test_neupan_avoidance import planner
    core = planner()
    points = np.array([[.8],[.5]])
    original = points.copy()
    state = np.zeros((3,1))
    core.set_dynamic_geometry([[[.4,.4],[.7,.4],[.7,.6],[.4,.6]]])
    near = core._candidate_distances(state,points)
    core.set_dynamic_geometry([[[.4,1.],[.7,1.],[.7,1.2],[.4,1.2]]])
    far = core._candidate_distances(state,points)
    assert near[0,0] == pytest.approx(.1,abs=1e-6)
    assert far[0,0] > near[0,0]
    np.testing.assert_array_equal(points,original)


def test_dynamic_solver_replans_against_actual_arm_geometry():
    from test_neupan_avoidance import planner
    core = planner()
    core.set_dynamic_geometry([[[.2,.32],[.65,.32],[.65,.55],[.2,.55]]])
    obstacle = np.array([[1.],[.6]])
    before = obstacle.copy()
    state = np.zeros((3,1))
    action, info = core(state, obstacle, measured_velocity=np.zeros((2,1)))
    assert np.isfinite(action).all()
    assert not info["stop"]
    controls = info["vel_tensor"].detach().cpu().numpy()
    rollout = core._rollout_controls(state, controls, subdivisions=3)
    distances = core._candidate_distances(rollout, obstacle, sample_dt=core.dt / 3.)
    assert core._candidate_feasibility(distances)[0]
    np.testing.assert_array_equal(obstacle, before)


def test_arm_risk_sampling_keeps_point_next_to_extended_arm():
    import torch
    from neupan.blocks.pan import PAN
    geometry=DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
                              [[1.0,.4],[1.4,.4],[1.4,.6],[1.0,.6]]])
    points=torch.stack((torch.zeros(200),torch.full((200,),.9)))
    arm_point=torch.tensor([[1.41],[.5]])
    points=torch.cat((points,arm_point),dim=1)
    indices=PAN.select_obstacle_indices(torch.zeros((3,2)),points,torch.zeros_like(points),
                                         20,.25,robot_geometry=geometry)
    assert 200 in indices.tolist()


def test_height_information_preserves_passage_and_unknown_remains_conservative():
    chassis = [[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    arm = dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=1.,z_max=1.2)
    model = DynamicGeometry([chassis,arm])
    points = np.array([[.55,.55,.55],[.5,.5,.5]])
    distance,_,_ = model.distance_and_normal(points,point_heights=[1.1,.2,np.nan])
    assert distance[0] < 0
    assert distance[1] > 0
    assert distance[2] < 0


def test_neupan_rollout_uses_obstacle_heights_without_assuming_unknown_is_clear():
    from test_neupan_avoidance import planner
    core = planner()
    arm=dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=1.,z_max=1.2)
    core.set_dynamic_geometry([arm])
    state=np.zeros((3,1))
    point=np.array([[.55],[.5]])
    high=core._candidate_distances(state,point,point_heights=np.array([1.1]))
    low=core._candidate_distances(state,point,point_heights=np.array([.2]))
    unknown=core._candidate_distances(state,point)
    assert high[0,0] < 0 and low[0,0] > 0
    assert unknown[0,0] == pytest.approx(high[0,0])


def test_solver_accepts_height_tagged_cloud_and_retains_original_points():
    from test_neupan_avoidance import planner
    core = planner()
    core.set_dynamic_geometry([dict(polygon_xy=[[.2,.4],[.8,.4],[.8,.6],[.2,.6]],
                                    z_min=1.,z_max=1.2)])
    points=np.array([[1.5,2.],[1.5,1.5]])
    heights=np.array([.2,1.1])
    original=points.copy()
    action,info=core(np.zeros((3,1)),points,measured_velocity=np.zeros((2,1)),
                     point_heights=heights)
    assert np.isfinite(action).all() and not info["stop"]
    np.testing.assert_array_equal(points,original)


def test_point_scope_scope_excludes_arms_without_erasing_obstacle():
    geometry=DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
                              [[.4,.4],[.7,.4],[.7,.6],[.4,.6]]])
    point=np.array([[.55,.55],[.5,.5]])
    distance,_,_=geometry.distance_and_normal(point,point_scope=[False,True])
    assert distance[0] < 0 and distance[1] > 0


def test_broadphase_keeps_exact_union_distances_even_for_overlapping_parts():
    rng=np.random.default_rng(7)
    square=np.array([[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]])
    components=[square+offset for offset in rng.uniform(-.5,.5,(12,2))]
    points=rng.uniform(-1,1,(2,300))
    expected=np.min(np.stack([DynamicGeometry([c]).distance_and_normal(points)[0]
                              for c in components]),axis=0)
    actual,_,_=DynamicGeometry(components).distance_and_normal(points)
    np.testing.assert_allclose(actual,expected,atol=1e-12)


def test_large_cloud_broadphase_keeps_point_near_extended_arm():
    import torch
    from neupan.blocks.pan import PAN
    geometry=DynamicGeometry([[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
                              [[1.,.4],[1.4,.4],[1.4,.6],[1.,.6]]])
    points=torch.stack((torch.linspace(2.,8.,2000),torch.full((2000,),3.)))
    points=torch.cat((points,torch.tensor([[1.41],[.5]])),dim=1)
    indices=PAN.select_obstacle_indices(torch.zeros((3,2)),points,torch.zeros_like(points),
        20,.25,robot_geometry=geometry,point_heights=torch.full((2001,),float("nan")))
    assert 2000 in indices.tolist()
    assert len(indices)==20 and len(set(indices.tolist()))==20


def test_swept_aabb_never_excludes_an_actual_sampled_collision():
    rng=np.random.default_rng(19)
    square=np.array([[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]])
    components=[square+offset for offset in rng.uniform(-.5,.5,(8,2))]
    geometry=DynamicGeometry(components)
    points=rng.uniform(-2,2,(2,2000))
    states=np.vstack((np.linspace(0.,.8,12),np.linspace(0.,.5,12),np.linspace(0.,1.,12)))
    candidate=geometry.trajectory_collision_candidates(states,points)
    for x,y,yaw in states.T:
        dx,dy=points[0]-x,points[1]-y
        local=np.vstack((np.cos(yaw)*dx+np.sin(yaw)*dy,
                         -np.sin(yaw)*dx+np.cos(yaw)*dy))
        distance,_,_=geometry.distance_and_normal(local)
        assert np.all(candidate[distance<=0.])


def test_robot_scope_does_not_hide_corresponding_three_dimensional_arm_return():
    from r1_neupan_integration.obstacle_scene import assemble_scene
    body=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]]
    arm=dict(polygon_xy=[[.4,.4],[.7,.4],[.7,.6],[.4,.6]],z_min=1.,z_max=1.2)
    geometry=DynamicGeometry([body,arm])
    point=np.array([[.55],[.5]])
    cloud=np.array([[.55],[.5],[1.1]])
    pts,z,scope,complete=assemble_scene(point,cloud,np.empty((3,0)),
                                        arm_geometry=geometry,visibility_proven=True)
    dist,_,_=geometry.distance_and_normal(pts,z,scope)
    assert complete and dist[0]>0 and dist[1]<0
