#!/usr/bin/env python3
"""Publish one frozen forward path in the robot's actual odom frame."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FixedPathPublisher(Node):
    def __init__(self):
        super().__init__('r1_n1_fixed_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/r1_test_path')
        self.declare_parameter('path_length', 4.0)
        self.declare_parameter('path_spacing', 0.5)
        self.declare_parameter('publish_hz', 2.0)

        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('path_topic').value,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_hz').value),
            self.publish_path,
        )
        self._origin = None
        self._logged = False

    def odom_callback(self, message):
        if self._origin is not None:
            return
        frame_id = message.header.frame_id or 'odom'
        if frame_id != 'odom':
            self.get_logger().warn(
                f'Expected odom frame, received {frame_id}; preserving actual frame'
            )
        self._origin = (
            frame_id,
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            quaternion_to_yaw(message.pose.pose.orientation),
        )
        self.get_logger().info(
            'Frozen path origin: frame=%s x=%.4f y=%.4f yaw=%.4f'
            % self._origin
        )

    def publish_path(self):
        if not rclpy.ok():
            return
        if self._origin is None:
            return
        frame_id, x0, y0, yaw = self._origin
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        spacing = float(self.get_parameter('path_spacing').value)
        length = float(self.get_parameter('path_length').value)
        count = int(round(length / spacing))
        for index in range(count + 1):
            distance = min(index * spacing, length)
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x0 + distance * math.cos(yaw)
            pose.pose.position.y = y0 + distance * math.sin(yaw)
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        self.path_pub.publish(path)
        if not self._logged:
            self.get_logger().info(
                f'Published frozen {len(path.poses)}-pose path on {self.get_parameter("path_topic").value}'
            )
            self._logged = True


def main(args=None):
    rclpy.init(args=args)
    node = FixedPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # rclpy can invalidate the timer's pybind context concurrently with
        # launch shutdown.  Treat only that shutdown race as normal exit.
        if 'Unable to convert call argument' not in str(error):
            raise
    except Exception as error:
        # A launch-wide SIGINT can invalidate this helper's wait set before
        # its executor returns.  Only suppress that specific shutdown race.
        if type(error).__name__ != 'RCLError':
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
