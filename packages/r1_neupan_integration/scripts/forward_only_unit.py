#!/usr/bin/env python3
"""Deterministic unit coverage for the production forward-only adapter."""

from __future__ import annotations

import math

import numpy as np

from r1_neupan_integration.forward_only import ForwardOnlyConfig, ForwardOnlyController


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    config = ForwardOnlyConfig()
    controller = ForwardOnlyController(config)
    state = np.asarray([[0.0], [0.0], [0.0]])
    ahead_path = np.asarray([[0.0, 0.6, 1.2], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    left_path = np.asarray([[0.0, 0.0, 0.0], [0.0, 0.6, 1.2], [math.pi / 2] * 3])
    forward = controller.decide(
        raw_linear_x=0.12, raw_angular_z=0.03, stop_requested=False, arrived=False,
        state=state, path=ahead_path, path_version=1, goal_version=1,
    )
    check(forward.state == "TRACK" and forward.linear_x == 0.12,
          "positive clear command must track forward")

    positive_but_off_path = controller.decide(
        raw_linear_x=0.12, raw_angular_z=0.0, stop_requested=False, arrived=False,
        state=state, path=left_path, path_version=2, goal_version=2,
    )
    check(
        positive_but_off_path.state == "TRACK" and
        positive_but_off_path.linear_x == 0.12 and
        positive_but_off_path.angular_z == 0.0,
        "forward NeuPAN command must not be replaced by a path-heading command",
    )

    reverse = controller.decide(
        raw_linear_x=-0.15, raw_angular_z=-0.4, stop_requested=False, arrived=False,
        state=state, path=left_path, path_version=3, goal_version=3,
    )
    check(reverse.state == "REORIENT" and reverse.linear_x == 0.0 and reverse.angular_z == -0.4,
          "reverse request must retain NeuPAN turn while clamping only reverse linear velocity")

    waiting = controller.decide(
        raw_linear_x=0.12, raw_angular_z=0.3, stop_requested=True, arrived=False,
        state=state, path=ahead_path, path_version=4, goal_version=4,
    )
    check(waiting.state == "WAIT" and waiting.linear_x == 0.0 and waiting.angular_z == 0.0,
          "explicit NeuPAN stop must remain a complete stop")

    for decision in (forward, positive_but_off_path, reverse, waiting):
        check(decision.linear_x >= 0.0, "adapter must never emit reverse velocity")
    print("forward_only_unit PASS: NeuPAN-owned heading, no reverse, explicit stop")


if __name__ == "__main__":
    main()
