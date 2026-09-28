"""Inject controlled commands for runtime no-reverse and rotation proofs."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class NoReverseProbe(Node):
    """Publish negative upstream commands while the gate owns actuator safety."""

    def __init__(self) -> None:
        super().__init__("r1_n2_pre_no_reverse_probe")
        self.declare_parameter("command_topic", "/neupan_cmd_vel")
        self.declare_parameter("timing_topic", "/neupan_timing")
        self.declare_parameter("start_delay_sec", 3.0)
        self.declare_parameter("negative_duration_sec", 5.0)
        self.declare_parameter("stop_duration_sec", 1.5)
        self.declare_parameter("probe_mode", "no_reverse")
        self.declare_parameter("probe_angular_z", 0.2)
        self._started = time.monotonic()
        self._command_pub = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 20
        )
        self._timing_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("timing_topic").value), 20
        )
        self._timer = self.create_timer(
            0.02, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _publish(self) -> None:
        elapsed = time.monotonic() - self._started
        start = float(self.get_parameter("start_delay_sec").value)
        negative_end = start + float(self.get_parameter("negative_duration_sec").value)
        stop_end = negative_end + float(self.get_parameter("stop_duration_sec").value)
        command = Twist()
        if start <= elapsed < negative_end:
            mode = str(self.get_parameter("probe_mode").value)
            if mode == "no_reverse":
                phase = int((elapsed - start) / 1.5) % 3
                command.linear.x = (-0.05, -0.15, -0.15)[phase]
                command.angular.z = (0.0, 0.2, -0.2)[phase]
            elif mode == "rotate":
                command.angular.z = float(self.get_parameter("probe_angular_z").value)
            else:
                raise RuntimeError(f"unknown probe_mode: {mode}")
        self._command_pub.publish(command)

        # Keep the production gate's timing freshness input valid without a
        # planner. The motion probe does not make planner or runtime claims.
        timing = Float64MultiArray()
        timing.data = [elapsed] * 47
        self._timing_pub.publish(timing)
        if elapsed >= stop_end:
            self._timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NoReverseProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
