import struct
from types import SimpleNamespace
import numpy as np
import pytest
from r1_neupan_integration.obstacle_cloud import decode_xyz_timestamps


def message(data, fields, width=2, point_step=20, row_step=None):
    return SimpleNamespace(fields=[SimpleNamespace(name=n,offset=o,datatype=d,count=1) for n,o,d in fields],
        width=width,height=1,point_step=point_step,row_step=row_step or width*point_step,
        data=data,is_bigendian=False,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100,nanosec=0)))


def test_preserves_three_dimensional_points_and_acquisition_times():
    data=struct.pack("<fff d",1.,2.,3.,100000000000.)+struct.pack("<fff d",4.,5.,6.,100005000000.)
    cloud=message(data,[("x",0,7),("y",4,7),("z",8,7),("timestamp",12,8)])
    xyz,times,per_point=decode_xyz_timestamps(cloud)
    np.testing.assert_array_equal(xyz,[[1.,4.],[2.,5.],[3.,6.]])
    np.testing.assert_array_equal(times,[100000000000,100005000000])
    assert per_point


def test_nan_points_are_dropped_and_missing_time_stays_header_time():
    data=struct.pack("<fff",1.,2.,3.)+struct.pack("<fff",float("nan"),5.,6.)
    cloud=message(data,[("x",0,7),("y",4,7),("z",8,7)],point_step=12)
    xyz,times,per_point=decode_xyz_timestamps(cloud)
    np.testing.assert_array_equal(xyz,[[1.],[2.],[3.]])
    np.testing.assert_array_equal(times,[100000000000])
    assert not per_point
    cloud.data=b""
    with pytest.raises(ValueError):
        decode_xyz_timestamps(cloud)



def test_cloud_projection_uses_each_point_time_and_never_changes_input():
    from r1_neupan_integration.obstacle_cloud import project_cloud_to_world
    xyz=np.array([[1.,1.],[0.,0.],[.4,1.2]])
    before=xyz.copy()
    stamps=np.array([100000000000,100005000000])
    looked_up=[]
    def tf_at(stamp):
        looked_up.append(stamp)
        tf=np.eye(4);tf[0,3]=(stamp-100000000000)*1e-9
        return tf
    world=project_cloud_to_world(xyz,stamps,100000000000,5000000,200000000,tf_at)
    np.testing.assert_allclose(world,[[1.0025,1.0075],[0.,0.],[.4,1.2]])
    np.testing.assert_array_equal(xyz,before)
    assert looked_up == [100002500000,100007500000]
    with pytest.raises(ValueError):
        project_cloud_to_world(xyz,stamps+300000000,100000000000,5000000,200000000,tf_at)


def test_rear_laser_world_height_uses_its_actual_transform_and_beam_time():
    from r1_neupan_integration.obstacle_cloud import project_scan_to_world
    scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100,nanosec=0)),
        ranges=[1.,float("inf"),2.],angle_min=0.,angle_increment=np.pi/2,
        time_increment=.005,range_min=.1,range_max=4.)
    seen=[]
    def tf_at(stamp):
        seen.append(stamp)
        tf=np.eye(4);tf[2,3]=.35
        return tf
    result=project_scan_to_world(scan,5000000,200000000,tf_at)
    np.testing.assert_allclose(result,[[1.,-2.],[0.,0.],[.35,.35]],atol=1e-12)
    assert seen == [100002500000,100012500000]


def test_sensor_transform_keeps_world_height_and_normalizes_quaternion():
    from r1_neupan_integration.obstacle_cloud import transform_matrix
    transform=SimpleNamespace(translation=SimpleNamespace(x=1.,y=2.,z=.3),
        rotation=SimpleNamespace(x=0.,y=0.,z=np.sqrt(2),w=np.sqrt(2)))
    matrix=transform_matrix(transform)
    np.testing.assert_allclose(matrix @ [1.,0.,.5,1.],[1.,3.,.8,1.],atol=1e-12)
    transform.rotation.w=0.;transform.rotation.z=0.
    with pytest.raises(ValueError):
        transform_matrix(transform)
