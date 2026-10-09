#!/usr/bin/python3
"""Wait for both real AR5 joint-state streams before admitting navigation."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def valid_joint_state(message):
    """Accept only a complete finite AR5 state sample."""
    return (
        len(message.name) == 7
        and len(message.position) == 7
        and set(message.name) == {f"joint{index}" for index in range(1, 8)}
        and all(math.isfinite(value) for value in message.position)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-seconds", type=float, default=45.0)
    parser.add_argument("--required-samples", type=int, default=3)
    # launch_ros appends --ros-args to every Node executable.  This helper has
    # no ROS parameters, so intentionally ignore that tail.
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Node("navigation_arm_state_gate")
    deadline = time.monotonic() + args.timeout_seconds
    received = {"left": 0, "right": 0}

    def record(side):
        def callback(message):
            if valid_joint_state(message):
                received[side] += 1
        return callback

    left_subscription = node.create_subscription(
        JointState, "/left/joint_states", record("left"), 20
    )
    right_subscription = node.create_subscription(
        JointState, "/right/joint_states", record("right"), 20
    )
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if (received["left"] >= args.required_samples and
                    received["right"] >= args.required_samples):
                node.get_logger().info("Both real arm joint-state streams are live and valid")
                return 0
            rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().error(
            "Timed out waiting for complete real arm joint-state samples "
            f"(left={received['left']}, right={received['right']})"
        )
        return 1
    finally:
        node.destroy_subscription(left_subscription)
        node.destroy_subscription(right_subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
