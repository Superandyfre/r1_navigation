#!/usr/bin/env python3
"""Offline checks for bounded reverse on a stopped NeuPAN solve."""

import numpy as np

from r1_neupan_integration.stop_reverse_recovery import (
    StopReverseRecovery, stopped_reverse_velocity,
)


def main() -> None:
    assert stopped_reverse_velocity(np.array([[-0.15, -0.08], [0.12, 0.20]])) == (-0.10, 0.12)
    assert stopped_reverse_velocity(np.array([[0.10], [0.0]])) is None
    assert stopped_reverse_velocity(np.array([[-0.10, 0.12]])) is None
    assert stopped_reverse_velocity(np.array([[float("nan")], [0.0]])) is None

    recovery = StopReverseRecovery()
    goal = (1.0, 0.0, 0.0)
    assert recovery.permit(goal_pose=goal, pose=(0.0, 0.0), active_gear=1, now=0.0)
    assert recovery.permit(goal_pose=goal, pose=(0.0, 0.0), active_gear=-1, now=1.0)
    assert recovery.permit(goal_pose=goal, pose=(-0.20, 0.0), active_gear=-1, now=2.0)
    assert not recovery.permit(goal_pose=goal, pose=(-0.26, 0.0), active_gear=-1, now=3.0)
    assert not recovery.permit(goal_pose=goal, pose=(-0.20, 0.0), active_gear=-1, now=3.1)
    assert recovery.permit(goal_pose=goal, pose=(0.40, 0.0), active_gear=1, now=4.0)

    timeout = StopReverseRecovery()
    assert timeout.permit(goal_pose=goal, pose=(0.0, 0.0), active_gear=1, now=0.0)
    assert not timeout.permit(goal_pose=goal, pose=(0.0, 0.0), active_gear=1, now=8.0)
    assert not timeout.permit(goal_pose=goal, pose=(0.0, 0.0), active_gear=1, now=8.1)
    assert not timeout.permit(goal_pose=(1.00005, 0.0, 0.001),
                              pose=(0.0, 0.0), active_gear=1, now=8.2)
    next_goal = (2.0, 0.0, 0.0)
    assert timeout.permit(goal_pose=next_goal, pose=(0.0, 0.0), active_gear=-1, now=9.0)
    assert not timeout.permit(goal_pose=next_goal, pose=(0.0, 0.0), active_gear=-1, now=13.0)
    print("stop_reverse_recovery_unit PASS")


if __name__ == "__main__":
    main()
