#!/usr/bin/env python3
"""Exercise the real callbacks without starting nodes or publishing motion."""
from types import SimpleNamespace
from threading import Lock
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from r1_neupan_integration.n1_15_node import N115Node

def fake(authoritative=True):
    return SimpleNamespace(state_lock=Lock(), include_path_direction=True,
        goal_from_path_only=authoritative, active_gear=1, path_version=0,
        nav2_path_version=0, goal_version=-1, goal=None,
        _planner_input_updated=lambda:None)
def path(x):
    m=Path()
    for v in [0.0,x]:
        p=PoseStamped();p.pose.position.x=v;p.pose.orientation.w=1.0
        m.poses.append(p)
    return m
def main():
    for order in ('goal_first','path_first'):
        n=fake()
        for index,x in enumerate([1.0,2.0,2.0,3.0],1):
            m=path(x)
            if order=='goal_first':N115Node.goal_callback(n,m.poses[-1])
            N115Node.path_callback(n,m)
            if order=='path_first':N115Node.goal_callback(n,m.poses[-1])
            assert (n.nav2_path_version,n.goal_version)==(index,index-1), (
                order,n.nav2_path_version,n.goal_version)
        N115Node.goal_callback(n,path(1.0).poses[-1])
        assert n.goal[0,0]==3.0 and n.goal_version==3, 'late companion goal replaced current context'
    n=fake(False)
    N115Node.goal_callback(n,path(4.0).poses[-1])
    assert n.goal_version==0 and n.goal[0,0]==4.0, 'standalone goal compatibility'
    print('nav2_goal_order_unit PASS')
if __name__=='__main__':main()
