import importlib.util
from pathlib import Path
from types import SimpleNamespace
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
root=Path(__file__).resolve().parents[2]
def load(path):
 s=importlib.util.spec_from_file_location("test_"+str(abs(hash(str(path)))),path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def txt(x,c):return perform_substitutions(c,normalize_to_list_of_substitutions(x))
def declare(m,c):
 d=m.generate_launch_description()
 for a in d.entities:
  if isinstance(a,DeclareLaunchArgument):a.execute(c)
 return d
def inc(a,c):
 return {txt(k,c):txt(v,c) for k,v in a.launch_arguments}
c=LaunchContext();c.launch_configurations.update(local_controller_mode="neupan",arm_extra_mesh_config="/tmp/held-object.yaml",mesh_3d_projection_margin_m="0.01")
sel=load(root/"navigation_2d/launch/local_and_nav_high_sampling_controller.launch.py");declare(sel,c)
a,=sel._select(c);a=inc(a,c);assert a["mesh_filter_mode"]=="3d" and a["arm_extra_mesh_config"]=="/tmp/held-object.yaml" and a["mesh_3d_projection_margin_m"]=="0.01"
c.launch_configurations.update(a)
mid=load(root/"navigation_2d/launch/local_and_nav_high_sampling_neupan.launch.py");d=declare(mid,c)
o,=[a for a in d.entities if isinstance(a,OpaqueFunction)];f=o._OpaqueFunction__function;cl=dict(zip(f.__code__.co_freevars,[cell.cell_contents for cell in f.__closure__]));actions=cl["after_arm_gate"](SimpleNamespace(returncode=0),c)
n,=[a for a in actions if isinstance(a,Node) and a._Node__node_name is not None and txt(a._Node__node_name,c)=="ui7_runtime_supervisor"]
p={}
for row in evaluate_parameters(c,n._Node__parameters):
 if isinstance(row,dict):p.update(row)
assert p["mesh_filter_mode"]=="3d" and p["arm_extra_mesh_config"]=="/tmp/held-object.yaml" and p["mesh_3d_projection_margin_m"]=="0.01"
own=load(root/"navigation_neupan_bridge/launch/ui7_owned_runtime.launch.py");d=declare(own,c)
rows=[inc(a,c) for a in d.entities if isinstance(a,IncludeLaunchDescription)];r,=[a for a in rows if "mesh_filter_mode" in a]
assert r["mesh_filter_mode"]=="3d" and r["arm_extra_mesh_config"]=="/tmp/held-object.yaml" and r["mesh_3d_projection_margin_m"]=="0.01"
print("selector -> supervisor -> owned motion launch forwards 3d and attachment config")
