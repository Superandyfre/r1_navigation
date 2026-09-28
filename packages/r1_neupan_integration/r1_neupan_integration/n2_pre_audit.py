"""Record N2-PRE command, freshness, gate and odometry evidence."""

from __future__ import annotations

import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage


class N2Audit(Node):
    def __init__(self) -> None:
        super().__init__("r1_n2_pre_audit")
        self.declare_parameter("scenario", "forward")
        self.declare_parameter("duration_sec", 20.0)
        self.declare_parameter("output_file", "n2_pre_results/audit.json")
        self.declare_parameter("input_topic", "/neupan_cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("gate_topic", "/r1_safety_gate_status")
        self.declare_parameter("scan_topic", "/scan_neupan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("timing_topic", "/neupan_timing")
        self._started = time.monotonic()
        self._last = {}
        self._records = []
        self._odom = []
        self._scan_seen = -1.0
        self._odom_seen = -1.0
        self._tf_seen = -1.0
        self._timing_seen = -1.0
        self._semantics = []
        self._contacts = []
        self._sub(Twist, str(self.get_parameter("input_topic").value), self._input)
        self._sub(Twist, str(self.get_parameter("output_topic").value), self._output)
        self._sub(String, str(self.get_parameter("gate_topic").value), self._gate)
        self._sub(LaserScan, str(self.get_parameter("scan_topic").value), self._scan)
        self._sub(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb)
        self._sub(TFMessage, str(self.get_parameter("tf_topic").value), self._tf)
        self._sub(Float64MultiArray, str(self.get_parameter("timing_topic").value), self._timing)
        self._sub(String, "/neupan_semantics", self._semantics_cb)
        self.timer = self.create_timer(0.02, self._sample)

    def _sub(self, kind, topic, callback):
        qos = 50
        if kind is LaserScan:
            qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(kind, topic, callback, qos)

    def _record(self, kind, value):
        self._last[kind] = (time.monotonic(), value)

    def _input(self, msg): self._record("input", [float(msg.linear.x), float(msg.angular.z)])
    def _output(self, msg): self._record("output", [float(msg.linear.x), float(msg.angular.z)])
    def _gate(self, msg):
        try: self._record("gate", json.loads(msg.data))
        except (TypeError, ValueError): self._record("gate", {"malformed": True})
    def _scan(self, _msg): self._scan_seen = time.monotonic()
    def _odom_cb(self, msg):
        now = time.monotonic()
        self._odom_seen = now
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom.append([now, float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw])
    def _tf(self, msg):
        if any(item.child_frame_id == "base_link" for item in msg.transforms): self._tf_seen = time.monotonic()
    def _timing(self, msg):
        if len(msg.data) >= 12: self._timing_seen = time.monotonic()
    def _semantics_cb(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = {"malformed": True}
        self._semantics.append({"t": time.monotonic() - self._started, "value": value})

    def _sample(self):
        now = time.monotonic()
        sample = {"t": now - self._started}
        for kind, (stamp, value) in self._last.items():
            sample[kind] = {"age_sec": now - stamp, "value": value}
        sample["freshness"] = {
            "scan_age_sec": now - self._scan_seen if self._scan_seen >= 0 else None,
            "odom_age_sec": now - self._odom_seen if self._odom_seen >= 0 else None,
            "tf_age_sec": now - self._tf_seen if self._tf_seen >= 0 else None,
            "timing_age_sec": now - self._timing_seen if self._timing_seen >= 0 else None,
        }
        self._records.append(sample)
        if now - self._started >= float(self.get_parameter("duration_sec").value):
            self._write()
            raise SystemExit(0)

    @staticmethod
    def _stats(values):
        total = len(values)
        finite = [v for v in values if math.isfinite(v)]
        if not finite: return {"count": 0, "total": total, "nonfinite": total}
        return {"count": len(finite), "total": total, "nonfinite": total - len(finite), "min": min(finite), "max": max(finite),
                "mean": statistics.fmean(finite), "p95": sorted(finite)[int(0.95 * (len(finite) - 1))]}

    def _rate(self, kind):
        times = [item["t"] for item in self._records if kind in item]
        if len(times) < 2 or times[-1] <= times[0]:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0])

    def _write(self):
        output = Path(str(self.get_parameter("output_file").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        inputs = [item["input"]["value"] for item in self._records if "input" in item]
        outputs = [item["output"]["value"] for item in self._records if "output" in item]
        gate = [item["gate"]["value"] for item in self._records if "gate" in item and isinstance(item["gate"]["value"], dict)]
        summary = {
            "schema": "r1_n2_pre_audit_v1",
            "scenario": str(self.get_parameter("scenario").value),
            "duration_sec": time.monotonic() - self._started,
            "sample_count": len(self._records),
            "input": {"linear_x": self._stats([v[0] for v in inputs]), "angular_z": self._stats([v[1] for v in inputs])},
            "output": {"linear_x": self._stats([v[0] for v in outputs]), "angular_z": self._stats([v[1] for v in outputs])},
            "rates_hz": {"input": self._rate("input"), "output": self._rate("output"), "gate": self._rate("gate")},
            "gate": {
                "clamp_count": sum(1 for item in gate if item.get("clamped")),
                "zero_count": sum(1 for item in gate if item.get("zeroed")),
                "reasons": sorted({reason for item in gate for reason in item.get("reasons", [])}),
                "last": gate[-1] if gate else None,
            },
            "odom": {
                "samples": len(self._odom),
                "start": self._odom[0] if self._odom else None,
                "end": self._odom[-1] if self._odom else None,
            },
            "freshness": {
                "scan_age_sec": self._records[-1]["freshness"]["scan_age_sec"] if self._records else None,
                "odom_age_sec": self._records[-1]["freshness"]["odom_age_sec"] if self._records else None,
                "tf_age_sec": self._records[-1]["freshness"]["tf_age_sec"] if self._records else None,
                "timing_age_sec": self._records[-1]["freshness"]["timing_age_sec"] if self._records else None,
            },
            "semantics": self._semantics,
            "records": self._records,
        }
        output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = N2Audit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        if not node._records or not Path(str(node.get_parameter("output_file").value)).exists(): node._write()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
