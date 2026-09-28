#!/usr/bin/env python3
"""Record Gazebo repeatability under a frozen D1-4 gated command history."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState, LaserScan, PointCloud2


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def header_stamp_seconds(msg: Any) -> float | None:
    """Return a ROS header stamp when the message type provides one."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def finite_min(values: Any) -> float | None:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return min(finite) if finite else None


def load_schedule(audit_path: Path, horizon_sec: float) -> list[dict[str, float]]:
    audit = json.loads(audit_path.read_text())
    release = float(audit["safety"]["release_done_t"])
    rows = [row for row in audit["command_trace"]["gated"] if float(row["audit_t"]) >= release]
    if not rows:
        raise RuntimeError("D1-4 gated command history has no rows after release")
    schedule = []
    for row in rows:
        offset = float(row["audit_t"]) - release
        if offset > horizon_sec:
            break
        schedule.append({"t_sec": max(0.0, offset), "linear_x": float(row["linear_x"]), "angular_z": float(row["angular_z"])})
    if not schedule:
        raise RuntimeError("D1-4 gated command history did not cover the requested horizon")
    if min(row["linear_x"] for row in schedule) < -1.0e-9:
        raise RuntimeError("frozen gated command history contains a negative linear command")
    return schedule


class FixedCommandRecorder(Node):
    def __init__(self, schedule: list[dict[str, float]], output: Path, horizon_sec: float) -> None:
        super().__init__("n2_4c4_fixed_command_recorder")
        self.schedule = schedule
        self.output = output
        self.horizon_sec = float(horizon_sec)
        self.started_wall = time.monotonic()
        self.start_wall: float | None = None
        self.ready_seen = False
        self.schedule_index = 0
        self.current_command = {"linear_x": 0.0, "angular_z": 0.0}
        self.commands: list[dict[str, Any]] = []
        self.odom: list[dict[str, Any]] = []
        self.joints: list[dict[str, Any]] = []
        self.scans: list[dict[str, Any]] = []
        self.clouds: list[dict[str, Any]] = []
        self.contacts: list[dict[str, Any]] = []
        self.clock: list[dict[str, Any]] = []
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 20)
        sensor_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._odom, sensor_qos)
        self.create_subscription(JointState, "/world/empty/model/r1_with_arm/joint_state", self._joint, sensor_qos)
        self.create_subscription(LaserScan, "/scan_neupan", self._scan, sensor_qos)
        self.create_subscription(PointCloud2, "/lidar/points", self._cloud, sensor_qos)
        self.create_subscription(Contacts, "/world/empty/contacts", self._contact, sensor_qos)
        # Node reserves ``_clock`` internally on Jazzy; keep the callback name
        # distinct so the bound method is not shadowed during initialization.
        self.create_subscription(Clock, "/clock", self._clock_msg, 20)
        self.timer = self.create_timer(0.02, self._tick)

    def _elapsed(self) -> float:
        return 0.0 if self.start_wall is None else time.monotonic() - self.start_wall

    def _odom(self, msg: Odometry) -> None:
        if self.start_wall is None:
            self.start_wall = time.monotonic()
            self.ready_seen = True
        self.odom.append({"t_sec": self._elapsed(), "sim_time_sec": header_stamp_seconds(msg), "x": float(msg.pose.pose.position.x), "y": float(msg.pose.pose.position.y), "yaw": yaw_from_quaternion(msg.pose.pose.orientation), "v": float(msg.twist.twist.linear.x), "w": float(msg.twist.twist.angular.z)})

    def _joint(self, msg: JointState) -> None:
        if self.start_wall is None:
            return
        names = list(msg.name)
        self.joints.append({"t_sec": self._elapsed(), "sim_time_sec": header_stamp_seconds(msg), "names": names, "positions": [float(x) for x in msg.position], "velocities": [float(x) for x in msg.velocity]})

    def _scan(self, msg: LaserScan) -> None:
        if self.start_wall is None:
            return
        values = [float(x) for x in msg.ranges]
        payload = bytearray()
        for value in values:
            payload.extend(float(value).hex().encode())
            payload.extend(b";")
        self.scans.append({"t_sec": self._elapsed(), "sim_time_sec": header_stamp_seconds(msg), "range_count": len(values), "finite_count": sum(math.isfinite(x) for x in values), "min_range": finite_min(values), "digest": sha256_bytes(bytes(payload))})

    def _cloud(self, msg: PointCloud2) -> None:
        if self.start_wall is None:
            return
        self.clouds.append({"t_sec": self._elapsed(), "sim_time_sec": header_stamp_seconds(msg), "width": int(msg.width), "height": int(msg.height), "point_step": int(msg.point_step), "row_step": int(msg.row_step), "data_digest": sha256_bytes(bytes(msg.data))})

    def _contact(self, msg: Contacts) -> None:
        if self.start_wall is None:
            return
        self.contacts.append({"t_sec": self._elapsed(), "contact_count": len(msg.contacts)})

    def _clock_msg(self, msg: Clock) -> None:
        if self.start_wall is None:
            return
        self.clock.append({"t_sec": self._elapsed(), "sec": int(msg.clock.sec), "nanosec": int(msg.clock.nanosec)})

    def _tick(self) -> None:
        if self.start_wall is None:
            self.publisher.publish(Twist())
            if time.monotonic() - self.started_wall > 30.0:
                self.finish("timeout_waiting_for_odom")
            return
        elapsed = self._elapsed()
        while self.schedule_index + 1 < len(self.schedule) and self.schedule[self.schedule_index + 1]["t_sec"] <= elapsed:
            self.schedule_index += 1
        if elapsed <= self.horizon_sec:
            self.current_command = self.schedule[self.schedule_index]
            msg = Twist()
            msg.linear.x = self.current_command["linear_x"]
            msg.angular.z = self.current_command["angular_z"]
            self.publisher.publish(msg)
            self.commands.append({"t_sec": elapsed, **self.current_command})
        else:
            self.publisher.publish(Twist())
            self.finish("completed")

    def finish(self, status: str) -> None:
        if getattr(self, "finished", False):
            return
        self.finished = True
        self.timer.cancel()
        max_contact = max((row["contact_count"] for row in self.contacts), default=0)
        command_min = min((row["linear_x"] for row in self.commands), default=0.0)
        report = {
            "schema": "r1_n2_4c4_fixed_command_replay_v1",
            "status": status,
            "audit_source": "n2_4c2_phenotype_runs/d1_04/audit.json",
            "schedule": self.schedule,
            "horizon_sec": self.horizon_sec,
            "safety_boundary": {"command_linear_min": command_min, "command_linear_nonnegative": command_min >= -1.0e-9, "contact_max": max_contact, "contact_zero": max_contact == 0, "planner_running": False, "gated_queue": "not applicable; direct frozen command replay", "stale": "not applicable; no planner worker", "nan_inf": "not applicable; finite ROS message fields recorded"},
            "counts": {"commands": len(self.commands), "odom": len(self.odom), "joint_states": len(self.joints), "scans": len(self.scans), "clouds": len(self.clouds), "contacts": len(self.contacts), "clock": len(self.clock)},
            "commands": self.commands,
            "odom": self.odom,
            "joint_states": self.joints,
            "scan": self.scans,
            "cloud": self.clouds,
            "contacts": self.contacts,
            "clock": self.clock,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        self.get_logger().info(f"wrote {self.output}")
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--horizon-sec", type=float, default=5.0)
    args = parser.parse_args()
    schedule = load_schedule(Path(args.audit), args.horizon_sec)
    rclpy.init()
    node = FixedCommandRecorder(schedule, Path(args.output), args.horizon_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
