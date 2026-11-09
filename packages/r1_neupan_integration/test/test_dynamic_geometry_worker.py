"""Actual spawned NeuPAN IPC worker with timestamped arm geometry."""
import tempfile
import time
from pathlib import Path
import numpy as np
from r1_neupan_integration.n1_16_bridge import ProductionWholeCoreClient

ROOT = Path(__file__).resolve().parents[3]


def test_worker_uses_transmitted_arm_geometry():
    with tempfile.TemporaryDirectory() as temp:
        client=ProductionWholeCoreClient(temp,
            str(ROOT/'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml'),
            str(ROOT/'packages/r1_neupan_integration/config/models/dune_model_5000.pth'),
            {},None,torch_intraop=1,torch_interop=1,warmup_count=0,timeout_sec=10.)
        try:
            x=np.linspace(0.,4.,41)
            path=np.vstack((x,np.zeros_like(x),np.zeros_like(x),np.ones_like(x)))
            state=np.zeros((3,1))
            points=np.array([[1.5,1.6],[1.5,1.6]])
            client.startup(path,state,points,1,navigation_loop_mode="terminal")
            snapshot=dict(schema="r1_arm_geometry_v1",stamp_ns=1000000000,sequence=1,geometry_version=1,
                components=[dict(frame="left_link7",
                    polygon_xy=[[.2,.32],[.65,.32],[.65,.55],[.2,.55]],z_min=0.,z_max=1.)])
            output,record=client.solve(state,points,path,1,1,1.,cycle_id=1,goal_version=0,
                navigation_loop_mode="terminal",measured_velocity=np.zeros((2,1)),
                dynamic_geometry=dict(snapshot=snapshot,received_monotonic=time.monotonic()),
                point_heights=np.array([np.nan,.8]),point_scope=np.array([0,2],dtype=np.uint8))
            assert record["finite"]
            assert np.isfinite(output["action"]).all()
        finally:
            client.close()
