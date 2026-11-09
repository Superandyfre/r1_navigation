"""Concurrency and freshness tests for read-only 3-D scene preprocessing."""
import struct
import time
from types import SimpleNamespace
import numpy as np
from r1_neupan_integration.arm_obstacle_scenes import LatestArmObstacleScenes


def cloud(sec,x):
    field=lambda name,offset:SimpleNamespace(name=name,offset=offset,datatype=7,count=1)
    return SimpleNamespace(fields=[field("x",0),field("y",4),field("z",8)],
        width=1,height=1,point_step=12,row_step=12,data=struct.pack("<fff",x,0.,.5),
        is_bigendian=False,header=SimpleNamespace(frame_id="lidar",stamp=SimpleNamespace(sec=sec,nanosec=0)))


def test_source_timestamps_are_not_refreshed_and_invalid_updates_are_ignored():
    worker=LatestArmObstacleScenes(lambda frame,stamp:np.eye(4))
    try:
        assert worker.submit("front",cloud(100,1.),received_monotonic=10.)
        end=time.monotonic()+2
        while "front" not in worker.snapshot() and time.monotonic()<end:time.sleep(.01)
        first=worker.snapshot()["front"]
        assert first["stamp_ns"]==100_000_000_000 and first["received_monotonic"]==10.
        assert worker.submit("front",cloud(99,4.),received_monotonic=20.)
        time.sleep(.1)
        assert worker.snapshot()["front"]["stamp_ns"]==100_000_000_000
        np.testing.assert_array_equal(worker.snapshot()["front"]["points"],[[1.],[0.],[.5]])
    finally:
        assert worker.close()
