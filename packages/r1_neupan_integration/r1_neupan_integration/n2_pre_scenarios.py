"""Publish deterministic paths for N2-PRE and forward-only N2 runs."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Int8


# Local coordinates are transformed from the first odometry pose.  Every N2
# scenario uses gear +1; the no-reverse policy is therefore exercised without
# introducing a dynamic gear transition into navigation.
N2_SCENARIO_SPECS = {
    "a1_straight": {
        "family": "basic_navigation", "label": "A1 Straight",
        "path": [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)],
        "goal_yaw": 0.0,
    },
    "a2_turn_forward": {
        "family": "basic_navigation", "label": "A2 Turn-then-forward",
        "path": [(0.0, 0.0), (0.55, 0.0), (0.75, 0.25), (0.9, 0.65), (1.25, 1.0)],
        "goal_yaw": 0.79,
    },
    "a3_l_path": {
        "family": "basic_navigation", "label": "A3 L-shaped reference",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.0, 0.0), (1.0, 0.55), (1.0, 1.0), (1.6, 1.0)],
        "goal_yaw": 0.0,
    },
    "a4_multi_waypoint": {
        "family": "basic_navigation", "label": "A4 Multi-waypoint",
        "path": [(0.0, 0.0), (0.55, 0.0), (1.0, 0.0), (1.15, 0.4),
                 (1.15, 0.85), (1.65, 0.95), (2.05, 0.95)],
        "goal_yaw": 0.0,
    },
    "b1_single_table": {
        "family": "static_obstacle", "label": "B1 Single table",
        "path": [(0.0, 0.0), (0.45, 0.0), (0.6, -0.8), (1.8, -0.8), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
    "b2_wide_channel": {
        "family": "static_obstacle", "label": "B2 Wide furniture channel",
        "path": [(0.0, 0.0), (0.65, 0.0), (1.4, 0.0), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
    "b3_table_legs": {
        "family": "static_obstacle", "label": "B3 Discrete table legs",
        "path": [(0.0, 0.0), (0.65, 0.0), (1.35, 0.0), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
    "b4_center_obstacle": {
        "family": "static_obstacle", "label": "B4 Central obstacle",
        "path": [(0.0, 0.0), (0.45, 0.0), (0.65, 0.85), (1.75, 0.85), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
    "b5_one_side_open": {
        "family": "static_obstacle", "label": "B5 One side open",
        "path": [(0.0, 0.0), (0.65, 0.0), (1.35, 0.0), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
    "c1_wide_corridor": {
        "family": "corridor", "label": "C1 Wide corridor",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.5, 0.0), (2.5, 0.0), (3.1, 0.0)],
        "goal_yaw": 0.0,
    },
    "c2_medium_corridor": {
        "family": "corridor", "label": "C2 Medium corridor",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.5, 0.0), (2.5, 0.0), (3.1, 0.0)],
        "goal_yaw": 0.0,
    },
    "c3_narrow_corridor": {
        "family": "corridor", "label": "C3 Narrow corridor",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.5, 0.0), (2.5, 0.0), (3.1, 0.0)],
        "goal_yaw": 0.0,
    },
    "d1_corner_obstacle": {
        "family": "hard_local", "label": "D1 Obstacle near turn",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.0, 0.0), (1.0, 0.7), (1.0, 1.2), (1.7, 1.2)],
        "goal_yaw": 0.0,
    },
    "d2_u_space": {
        "family": "hard_local", "label": "D2 U-shaped local space",
        "path": [(0.0, 0.0), (0.7, 0.0), (1.2, 0.0), (1.2, 0.8), (0.7, 0.8)],
        "goal_yaw": 3.14,
    },
    "d3_goal_near_furniture": {
        "family": "hard_local", "label": "D3 Goal near furniture",
        "path": [(0.0, 0.0), (0.6, 0.0), (1.2, 0.0), (1.65, 0.55)],
        "goal_yaw": 0.79,
    },
    "d4_rotate_then_forward": {
        "family": "hard_local", "label": "D4 Rotate-then-forward",
        "path": [(0.0, 0.0), (-0.15, 0.45), (-0.2, 0.95), (-0.2, 1.55)],
        "goal_yaw": 1.57,
    },
    "d5_one_side_recovery": {
        "family": "hard_local", "label": "D5 One-side recovery",
        "path": [(0.0, 0.0), (0.5, 0.0), (0.7, -0.75), (1.5, -0.75), (2.2, 0.0)],
        "goal_yaw": 0.0,
    },
}


def n2_scenario_spec(name: str):
    """Return an immutable-enough copy for audit/report consumers."""
    spec = N2_SCENARIO_SPECS.get(str(name))
    if spec is None:
        return None
    return {
        "family": spec["family"], "label": spec["label"],
        "path": [tuple(point) for point in spec["path"]],
        "goal_yaw": float(spec["goal_yaw"]),
    }


def _quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    value = Quaternion()
    value.z = math.sin(yaw / 2.0)
    value.w = math.cos(yaw / 2.0)
    return value


class N2ScenarioPublisher(Node):
    def __init__(self) -> None:
        super().__init__("r1_n2_pre_scenario")
        self.declare_parameter("scenario", "forward")
        self.declare_parameter("path_topic", "/r1_test_path")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("gear_topic", "/r1_motion_gear")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("odom_topic", "/odom")
        self._pose = None
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom, 10)
        self.pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 10)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.gear_pub = self.create_publisher(Int8, str(self.get_parameter("gear_topic").value), 10)
        self._goal_publish_count = 0
        self._publish_count = 0
        self.timer = self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self._publish)

    def _odom(self, message: Odometry) -> None:
        if self._pose is not None:
            return
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (message.header.frame_id or "odom", float(message.pose.pose.position.x),
                      float(message.pose.pose.position.y), yaw)

    def _coordinates(self):
        if self._pose is None:
            return None
        frame, x0, y0, yaw0 = self._pose
        scenario = str(self.get_parameter("scenario").value)
        n2_spec = n2_scenario_spec(scenario)
        if n2_spec is not None:
            local_xy = n2_spec["path"]
            local = []
            for index, (x, y) in enumerate(local_xy):
                if index + 1 < len(local_xy):
                    nx, ny = local_xy[index + 1]
                    yaw = math.atan2(ny - y, nx - x)
                elif index:
                    px, py = local_xy[index - 1]
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0
                local.append((x, y, yaw))
        elif scenario in {"forward", "r0"}:
            local = [(0.0, 0.0, 0.0), (0.25, 0.0, 0.0), (0.55, 0.0, 0.0), (0.9, 0.0, 0.0)]
        elif scenario == "m1_forward":
            # The unmodified planner uses a 0.5 m arrival threshold. Keep
            # this endpoint beyond that threshold so release produces motion.
            local = [(0.0, 0.0, 0.0), (0.25, 0.0, 0.0), (0.55, 0.0, 0.0), (0.9, 0.0, 0.0)]
        elif scenario == "reverse":
            local = [(0.0, 0.0, math.pi), (-0.15, 0.0, math.pi), (-0.35, 0.0, math.pi), (-0.5, 0.0, math.pi)]
        elif scenario == "m2_reverse":
            local = [(0.0, 0.0, math.pi), (-0.25, 0.0, math.pi), (-0.55, 0.0, math.pi), (-0.9, 0.0, math.pi)]
        elif scenario == "m3_left_rotate":
            # The bounded R1 cyclic admission window exercises this short
            # orientation-only path before any terminal arrival is expected.
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.35), (0.01, 0.0, 0.8), (0.01, 0.0, 1.2)]
        elif scenario == "m4_right_rotate":
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, -0.35), (0.01, 0.0, -0.8), (0.01, 0.0, -1.2)]
        elif scenario == "r3":
            local = [(0.0, 0.0, math.pi), (-0.25, 0.0, math.pi), (-0.55, 0.0, math.pi), (-0.9, 0.0, math.pi)]
        elif scenario == "r1":
            local = [(0.0, 0.0, 0.0), (-0.15, 0.0, 0.0), (-0.35, 0.0, 0.0), (-0.5, 0.0, 0.0)]
        elif scenario == "r2":
            local = [(0.0, 0.0, math.pi), (-0.15, 0.0, math.pi), (-0.35, 0.0, math.pi), (-0.5, 0.0, math.pi)]
        elif scenario == "left":
            local = [(0.0, 0.0, math.pi / 2), (0.15, 0.02, math.pi / 2), (0.28, 0.12, math.pi / 2), (0.30, 0.35, math.pi / 2)]
        elif scenario == "right":
            local = [(0.0, 0.0, -math.pi / 2), (0.15, -0.02, -math.pi / 2), (0.28, -0.12, -math.pi / 2), (0.30, -0.35, -math.pi / 2)]
        elif scenario in {"arrive", "a5"}:
            # A zero-length path is invalid input for the upstream sampler.
            # Keep the goal inside the existing arrival threshold instead.
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.02, 0.0, 0.0), (0.03, 0.0, 0.0)]
        elif scenario == "a0":
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0)]
        elif scenario == "a1":
            local = [(0.0, 0.0, 0.0), (0.03, 0.0, 0.0)]
        elif scenario == "a2":
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.10)]
        elif scenario == "a3":
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, math.pi)]
        elif scenario == "a4":
            # Non-zero segments with an endpoint exactly matching the start.
            local = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.0, 0.0, 0.0)]
        elif scenario in {"front_obstacle", "left_obstacle", "right_obstacle"}:
            y = {"front_obstacle": 0.0, "left_obstacle": 0.9, "right_obstacle": -0.9}[scenario]
            local = [(0.0, 0.0, 0.0), (0.4, y * 0.25, 0.0), (0.9, y * 0.7, 0.0), (1.5, y, 0.0)]
        else:
            raise ValueError("unknown N2-PRE semantics scenario")
        c, s = math.cos(yaw0), math.sin(yaw0)
        return frame, [(x0 + c * x - s * y, y0 + s * x + c * y, yaw0 + yaw) for x, y, yaw in local]

    def _publish(self) -> None:
        coordinates = self._coordinates()
        if coordinates is None:
            return
        frame, points = coordinates
        scenario = str(self.get_parameter("scenario").value)
        self._publish_count += 1
        gear = 1
        if scenario in {"reverse", "r1", "m2_reverse"}:
            gear = -1
        elif scenario == "r3" and self._publish_count > 5:
            # R3 exercises the mandatory zero transition from +1 to -1.
            gear = -1
        gear_message = Int8()
        gear_message.data = gear
        self.gear_pub.publish(gear_message)
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = _quat(yaw)
            path.poses.append(pose)
        self.pub.publish(path)
        # Delay the goal until the planner has had time to subscribe, then
        # publish it once.  Repeating it would intentionally create the
        # goal-version race that N1.16 must reject as stale.
        publish_goal = scenario in {"arrive", "a5"}
        if publish_goal and self._goal_publish_count == 5:
            goal = PoseStamped()
            goal.header = path.header
            goal.pose = path.poses[-1].pose
            self.goal_pub.publish(goal)
            self._goal_publish_count += 1
        elif publish_goal and self._goal_publish_count < 5:
            self._goal_publish_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = N2ScenarioPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
