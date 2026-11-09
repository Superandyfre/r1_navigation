import pytest
from r1_neupan_integration.arm_geometry_snapshot import ArmGeometrySnapshot


def payload():
    return dict(schema="r1_arm_geometry_v1",stamp_ns=1000000000,sequence=1,geometry_version=2,
                components=[dict(frame="left_link7",polygon_xy=[[0,0],[1,0],[1,1],[0,1]],
                                 z_min=.2,z_max=.8)])


def test_snapshot_keeps_source_time_and_is_immutable():
    p=payload()
    snap=ArmGeometrySnapshot.decode(p,100.)
    p["components"][0]["polygon_xy"][0][0]=100
    assert snap.components[0].polygon_xy[0][0] == 0
    assert snap.age(1250000000) == pytest.approx(.25)
    assert snap.age(999999999) is None
    # Re-receiving the same sample does not change the source age.
    assert ArmGeometrySnapshot.decode(payload(),200.).age(1250000000) == pytest.approx(.25)


def test_invalid_snapshots_are_not_treated_as_empty_robot():
    for mutate in (lambda x:x.update(stamp_ns=0),
                   lambda x:x.update(components=[]),
                   lambda x:x["components"][0].update(z_min=2),
                   lambda x:x["components"][0].update(polygon_xy=[[0,0],[1,0],[2,0]])):
        p=payload();mutate(p)
        with pytest.raises((ValueError,KeyError)):
            ArmGeometrySnapshot.decode(p,100.)



def test_arm_motion_rates_are_independent_and_do_not_refresh_source_time():
    import copy
    import math
    p=payload()
    p["components"][0].update(position_xyz=[0.,0.,0.],orientation_xyzw=[0.,0.,0.,1.],local_radius_m=.5)
    right=copy.deepcopy(p["components"][0]);right["frame"]="right_link7"
    p["components"].append(right)
    old=ArmGeometrySnapshot.decode(p,100.)
    newer=copy.deepcopy(p);newer["stamp_ns"]+=1_000_000_000;newer["sequence"]+=1
    newer["components"][0]["position_xyz"]=[.2,0.,0.]
    newer["components"][1]["orientation_xyzw"]=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)]
    current=ArmGeometrySnapshot.decode(newer,101.)
    rates=[part["uncertainty_rate_mps"] for part in current.planner_components(old)]
    assert rates == pytest.approx([.2,math.pi/4])
    assert current.age(2_500_000_000)==pytest.approx(.5)
    assert all(part["uncertainty_rate_mps"] is None for part in current.planner_components())
