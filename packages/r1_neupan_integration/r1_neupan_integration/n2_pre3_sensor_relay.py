"""Optional odom/TF relay used to test freshness loss without killing cmd_vel."""

from __future__ import annotations

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class SensorRelay(Node):
    """Forward selected sensors, with an optional freshness-drop point."""

    def __init__(self) -> None:
        super().__init__("n2_pre3_sensor_relay")
        self.declare_parameter("odom_input", "/odom")
        self.declare_parameter("odom_output", "/n2_pre3/odom")
        self.declare_parameter("tf_input", "/tf")
        self.declare_parameter("tf_output", "/n2_pre3/tf")
        self.declare_parameter("scan_input", "/scan_neupan")
        self.declare_parameter("scan_output", "/n2_pre3/scan")
        self.declare_parameter("relay_scan", False)
        self.declare_parameter("relay_odom_tf", False)
        self.declare_parameter("drop_after_sec", 0.0)
        self._started = time.monotonic()
        sensor_qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_output").value), 50
        )
        self.tf_pub = self.create_publisher(
            TFMessage, str(self.get_parameter("tf_output").value), 50
        )
        self.scan_pub = self.create_publisher(
            LaserScan, str(self.get_parameter("scan_output").value), 50
        )
        if bool(self.get_parameter("relay_odom_tf").value):
            self.create_subscription(
                Odometry, str(self.get_parameter("odom_input").value),
                self._odom_cb, sensor_qos,
            )
            self.create_subscription(
                TFMessage, str(self.get_parameter("tf_input").value),
                self._tf_cb, sensor_qos,
            )
        if bool(self.get_parameter("relay_scan").value):
            self.create_subscription(
                LaserScan, str(self.get_parameter("scan_input").value),
                self._scan_cb, sensor_qos,
            )

    def _forwarding(self) -> bool:
        drop_after = float(self.get_parameter("drop_after_sec").value)
        return drop_after <= 0.0 or time.monotonic() - self._started < drop_after

    def _odom_cb(self, message: Odometry) -> None:
        if self._forwarding():
            self.odom_pub.publish(message)

    def _tf_cb(self, message: TFMessage) -> None:
        if self._forwarding():
            self.tf_pub.publish(message)

    def _scan_cb(self, message: LaserScan) -> None:
        if self._forwarding():
            self.scan_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
