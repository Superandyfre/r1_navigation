#!/usr/bin/python3
"""Expose a hardware joint-state stream under an R1 arm-specific name."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrefixer(Node):
    def __init__(self):
        super().__init__("joint_state_prefixer")
        self.input_topic = self.declare_parameter("input_topic", "/right/joint_states").value
        self.output_topic = self.declare_parameter("output_topic", "/right_arm/joint_states").value
        self.prefix = self.declare_parameter("prefix", "right_").value
        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.subscription = self.create_subscription(
            JointState, self.input_topic, self.callback, 10
        )

    def callback(self, message):
        output = JointState()
        output.header = message.header
        output.name = [
            name if name.startswith(self.prefix) else self.prefix + name
            for name in message.name
        ]
        output.position = list(message.position)
        output.velocity = list(message.velocity)
        output.effort = list(message.effort)
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = JointStatePrefixer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
