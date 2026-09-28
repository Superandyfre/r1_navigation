#!/usr/bin/env python3
"""N2.4C5 fixed-command replay recorder with sim-time anchoring.

C4's recorder anchored the frozen D1-4 schedule and the 0.1 s sensor slots on
the wall clock at the first /odom receipt (audit finding H1/H2), so run-to-run
differences could come from wall/sim phase jitter rather than simulation
physics.  This recorder re-anchors everything on simulation time:

* wait for /clock, hold zero /cmd_vel until sim time reaches ``--settle-sec``;
* t_rel = sim_time - settle anchor; the frozen schedule is applied inside the
  /clock callback so every command transition lands within one 1 ms step at
  the same simulation time in every run;
* sensor frames are binned into 0.1 s slots on t_rel;
* the first scan/cloud after t_rel=0 is additionally dumped raw
  (``raw_slot0_scan.npz`` / ``raw_slot0_cloud.bin``) for the offline frozen
  preprocess determinism test.

Everything else (frozen audit source, horizon, safety boundary) matches
``n2_4c4_fixed_command_replay.py`` so the arms are directly comparable.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
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
        raise RuntimeError("frozen gated command history did not cover the requested horizon")
    if min(row["linear_x"] for row in schedule) < -1.0e-9:
        raise RuntimeError("frozen gated command history contains a negative linear command")
    return schedule


class SimTimeFixedCommandRecorder(Node):
    def __init__(
        self,
        schedule: list[dict[str, float]],
        output: Path,
        horizon_sec: float,
        settle_sec: float,
        raw_dir: Path,
    ) -> None:
        super().__init__("n2_4c5_fixed_command_recorder")
        self.schedule = schedule
        self.output = output
        self.horizon_sec = float(horizon_sec)
        self.settle_sec = float(settle_sec)
        self.raw_dir = raw_dir
        self.raw_dir.mkdir(parents=True, exist_ok=True)

        self.sim_t: float | None = None
        self.t0: float | None = None
        self.schedule_index = 0
        self.published_index: int | None = None
        self.current_command = {"linear_x": 0.0, "angular_z": 0.0}
        self.slot0_scan_saved = False
        self.slot0_cloud_saved = False

        self.commands: list[dict[str, Any]] = []
        self.odom: list[dict[str, Any]] = []
        self.joints: list[dict[str, Any]] = []
        self.scans: list[dict[str, Any]] = []
        self.clouds: list[dict[str, Any]] = []
        self.contacts: list[dict[str, Any]] = []
        self.clock: list[dict[str, Any]] = []
        self.clock_hold: list[dict[str, Any]] = []

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 20)
        sensor_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._odom, sensor_qos)
        self.create_subscription(JointState, "/world/empty/model/r1_with_arm/joint_state", self._joint, sensor_qos)
        self.create_subscription(LaserScan, "/scan_neupan", self._scan, sensor_qos)
        self.create_subscription(PointCloud2, "/lidar/points", self._cloud, sensor_qos)
        self.create_subscription(Contacts, "/world/empty/contacts", self._contact, sensor_qos)
        self.create_subscription(Clock, "/clock", self._clock_msg, 20)
        # Wall-clock guard only: fail fast if the sim never starts clocking.
        self.started_wall = time.monotonic()
        self.guard = self.create_timer(5.0, self._guard)

    # ---- helpers -------------------------------------------------------
    def _t_rel(self) -> float | None:
        return None if (self.sim_t is None or self.t0 is None) else self.sim_t - self.t0

    def _guard(self) -> None:
        if self.t0 is not None and self._t_rel() is not None and self._t_rel() > self.horizon_sec + 30.0:
            self.finish("timeout_overrun")
        elif time.monotonic() - self.started_wall > 180.0:
            self.finish("timeout_no_clock_or_odom")

    # ---- subscriptions -------------------------------------------------
    def _odom(self, msg: Odometry) -> None:
        t_rel = self._t_rel()
        if t_rel is None:
            return
        self.odom.append({"t_sec": t_rel, "sim_time_sec": header_stamp_seconds(msg), "x": float(msg.pose.pose.position.x), "y": float(msg.pose.pose.position.y), "yaw": yaw_from_quaternion(msg.pose.pose.orientation), "v": float(msg.twist.twist.linear.x), "w": float(msg.twist.twist.angular.z)})

    def _joint(self, msg: JointState) -> None:
        t_rel = self._t_rel()
        if t_rel is None:
            return
        self.joints.append({"t_sec": t_rel, "sim_time_sec": header_stamp_seconds(msg), "names": list(msg.name), "positions": [float(x) for x in msg.position], "velocities": [float(x) for x in msg.velocity]})

    def _scan(self, msg: LaserScan) -> None:
        t_rel = self._t_rel()
        if t_rel is None:
            return
        values = np.asarray(msg.ranges, dtype=np.float64)
        payload = np.float64(values).tobytes().hex().encode("ascii")
        row = {"t_sec": t_rel, "sim_time_sec": header_stamp_seconds(msg), "range_count": int(values.size), "finite_count": int(np.isfinite(values).sum()), "min_range": finite_min(values), "digest": sha256_bytes(payload)}
        self.scans.append(row)
        if not self.slot0_scan_saved and t_rel >= 0.0:
            np.savez(
                self.raw_dir / "raw_slot0_scan.npz",
                ranges=values,
                angle_min=np.float64(msg.angle_min),
                angle_max=np.float64(msg.angle_max),
                angle_increment=np.float64(msg.angle_increment),
                range_min=np.float64(msg.range_min),
                range_max=np.float64(msg.range_max),
                scan_time=np.float64(msg.scan_time),
                frame_id=np.array(str(msg.header.frame_id)),
                sim_time_sec=np.float64(header_stamp_seconds(msg) or 0.0),
            )
            self.slot0_scan_saved = True

    def _cloud(self, msg: PointCloud2) -> None:
        t_rel = self._t_rel()
        if t_rel is None:
            return
        data = bytes(msg.data)
        self.clouds.append({"t_sec": t_rel, "sim_time_sec": header_stamp_seconds(msg), "width": int(msg.width), "height": int(msg.height), "point_step": int(msg.point_step), "row_step": int(msg.row_step), "data_digest": sha256_bytes(data), "data_bytes": len(data)})
        if not self.slot0_cloud_saved and t_rel >= 0.0:
            (self.raw_dir / "raw_slot0_cloud.bin").write_bytes(data)
            (self.raw_dir / "raw_slot0_cloud.meta.json").write_text(json.dumps({
                "width": int(msg.width), "height": int(msg.height),
                "point_step": int(msg.point_step), "row_step": int(msg.row_step),
                "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype} for f in msg.fields],
                "is_bigendian": bool(msg.is_bigendian), "is_dense": bool(msg.is_dense),
                "sim_time_sec": header_stamp_seconds(msg),
                "data_digest": sha256_bytes(data),
            }, indent=2), encoding="utf-8")
            self.slot0_cloud_saved = True

    def _contact(self, msg: Contacts) -> None:
        t_rel = self._t_rel()
        if t_rel is None:
            return
        self.contacts.append({"t_sec": t_rel, "contact_count": len(msg.contacts)})

    def _clock_msg(self, msg: Clock) -> None:
        sim = float(msg.clock.sec) + 1.0e-9 * float(msg.clock.nanosec)
        self.sim_t = sim
        if self.t0 is None:
            self.clock_hold.append({"sim_time_sec": sim})
            if sim < self.settle_sec:
                return
            self.t0 = self.settle_sec
        t_rel = sim - self.t0
        while self.schedule_index + 1 < len(self.schedule) and self.schedule[self.schedule_index + 1]["t_sec"] <= t_rel:
            self.schedule_index += 1
        entry = self.schedule[self.schedule_index] if t_rel <= self.horizon_sec else None
        key = self.schedule_index if entry is not None else -1
        # Publish and record only on schedule transitions (plus the first
        # tick): DiffDrive holds the last command, so per-tick publishing at
        # 1 kHz only adds bridge jitter, not information.
        if key != self.published_index:
            out = Twist()
            if entry is not None:
                out.linear.x = entry["linear_x"]
                out.angular.z = entry["angular_z"]
            self.publisher.publish(out)
            self.published_index = key
            self.current_command = {"linear_x": out.linear.x, "angular_z": out.angular.z}
            self.commands.append({"t_sec": t_rel, "sim_time_sec": sim, "schedule_index": key, "linear_x": out.linear.x, "angular_z": out.angular.z})
        self.clock.append({"t_sec": t_rel, "sim_time_sec": sim})
        if t_rel > self.horizon_sec:
            self.finish("completed")

    # ---- output --------------------------------------------------------
    def finish(self, status: str) -> None:
        if getattr(self, "finished", False):
            return
        self.finished = True
        self.guard.cancel()
        max_contact = max((row["contact_count"] for row in self.contacts), default=0)
        command_min = min((row["linear_x"] for row in self.commands), default=0.0)
        report = {
            "schema": "r1_n2_4c5_fixed_command_replay_v1",
            "status": status,
            "audit_source": "n2_4c2_phenotype_runs/d1_04/audit.json",
            "anchor": {"mode": "sim_time", "settle_sec": self.settle_sec, "horizon_sec": self.horizon_sec, "clock_hold_count": len(self.clock_hold)},
            "schedule": self.schedule,
            "safety_boundary": {"command_linear_min": command_min, "command_linear_nonnegative": command_min >= -1.0e-9, "contact_max": max_contact, "contact_zero": max_contact == 0, "planner_running": False, "gated_queue": "not applicable; direct frozen command replay", "stale": "not applicable; no planner worker", "nan_inf": "not applicable; finite ROS message fields recorded"},
            "counts": {"commands": len(self.commands), "odom": len(self.odom), "joint_states": len(self.joints), "scans": len(self.scans), "clouds": len(self.clouds), "contacts": len(self.contacts), "clock": len(self.clock), "clock_hold": len(self.clock_hold)},
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
        self.get_logger().info(f"wrote {self.output} status={status}")
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--horizon-sec", type=float, default=5.0)
    parser.add_argument("--settle-sec", type=float, default=8.0)
    parser.add_argument("--raw-dir", default="")
    args = parser.parse_args()
    schedule = load_schedule(Path(args.audit), args.horizon_sec)
    raw_dir = Path(args.raw_dir) if args.raw_dir else Path(args.output).parent / "raw"
    rclpy.init()
    node = SimTimeFixedCommandRecorder(schedule, Path(args.output), args.horizon_sec, args.settle_sec, raw_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("interrupted")
    except rclpy.executors.ExternalShutdownException:
        # finish() shuts rclpy down from inside the clock callback; the
        # subsequent spin iteration raises here. replay.json is already on
        # disk when that happens, so this is a normal exit path.
        pass
    finally:
        if not getattr(node, "finished", False):
            node.finish("spinning_stopped")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
