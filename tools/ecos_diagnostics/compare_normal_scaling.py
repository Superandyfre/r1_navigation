#!/usr/bin/env python3
"""Small deterministic normal-case comparison; no ROS or hardware."""
import argparse
import json,time,yaml,numpy as np
from pathlib import Path
from unittest.mock import patch
from neupan import neupan
from neupan.blocks.nrmp import NRMP
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
root=Path(__file__).resolve().parents[2]
config=yaml.safe_load((root/'packages/navigation_neupan_bridge/config/planner_r1_hw1.yaml').read_text())
config['pan']['dune_checkpoint']=str(root/'packages/r1_neupan_integration/config/models/dune_model_5000.pth')
config['ipath']['loop']=False
original=NRMP.construct_prob
rows=[]
for scale in [1.,.1]:
 def build(self):
  self.objective_scale=scale
  return original(self)
 with patch.object(NRMP,'construct_prob',build):
  core=neupan(**{k:v for k,v in config.items() if k not in ('robot','ipath','pan','adjust')},robot_kwargs=config['robot'],ipath_kwargs=config['ipath'],pan_kwargs=config['pan'],adjust_kwargs=config['adjust'])
 for scenario,points in [('clear',np.array([[0.,1.,2.],[2.,2.,2.]])),('distant-wall',np.array([[1.,1.,1.],[1.5,1.8,2.]]))]:
  core.update_initial_path_from_goal(np.zeros((3,1)),np.array([[2.],[0.],[0.]]))
  for cycle in range(10):
   started=time.monotonic()
   action,info=core(np.zeros((3,1)),points,measured_velocity=np.zeros((2,1)))
   rows.append(dict(scale=scale,scenario=scenario,cycle=cycle,seconds=time.monotonic()-started,action=np.asarray(action).tolist(),stop=bool(info['stop'])))
args.output.write_text(json.dumps(rows,indent=2))
for scale in [1.,.1]:
 subset=[r for r in rows if r['scale']==scale]
 print(scale,'p50/p95/max',np.quantile([r['seconds'] for r in subset],[.5,.95,1]),'stops',sum(r['stop'] for r in subset),'actions',subset[-1]['action'])
