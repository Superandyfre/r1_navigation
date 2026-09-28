#!/usr/bin/env python3
"""Emit a narrow PyTorch CPU profile for the frozen DUNE model input."""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import torch

from n1_8_dune_attribution import planner_from_config, set_path

def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument('--snapshot',required=True); p.add_argument('--config',required=True); p.add_argument('--checkpoint',required=True); p.add_argument('--threads',type=int,required=True); p.add_argument('--output',required=True)
    a=p.parse_args(); torch.set_num_threads(a.threads)
    import numpy as np
    x=np.load(a.snapshot); planner,_=planner_from_config(a.config,a.checkpoint,'cpu',0.2); set_path(planner,x['initial_path'].astype(float))
    from neupan.configuration import np_to_tensor
    nom=planner.ipath.generate_nom_ref_state(x['robot_state'].astype(float),planner.cur_vel_array,planner.ref_speed)
    flow,_,_=planner.pan.generate_point_flow(np_to_tensor(nom[0]),np_to_tensor(x['obstacle_points'].astype(float)),None)
    inp=torch.hstack(flow).T; model=planner.pan.dune_layer.model
    for _ in range(20):
        with torch.no_grad(): model(inp)
    with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU], record_shapes=True, profile_memory=True) as prof:
        for _ in range(50):
            with torch.no_grad(): model(inp)
    Path(a.output).write_text(prof.key_averages(group_by_input_shape=True).table(sort_by='self_cpu_time_total', row_limit=30),encoding='utf-8')
    print(Path(a.output).read_text())
    return 0
if __name__=='__main__': raise SystemExit(main())
