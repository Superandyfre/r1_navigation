"""Fixed-rate relay for the N2.3 planner-result envelope."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from r1_neupan_integration.n2_3_contract import ContractError, relay_envelope, validate_envelope


class PlannerCommandRelay(Node):
    """Repeat the latest valid result without changing its source identity."""

    def __init__(self) -> None:
        super().__init__("r1_n2_3_command_relay")
        self.declare_parameter("source_topic", "/neupan_planner_result")
        self.declare_parameter("relay_topic", "/neupan_cmd_envelope")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("status_topic", "/neupan_relay_status")
        self._latest = None
        self._sequence = 0
        self._source_count = 0
        self._relay_count = 0
        self._invalid_count = 0
        self._last_source_identity = None
        self.relay_pub = self.create_publisher(
            String, str(self.get_parameter("relay_topic").value), 20
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.source_sub = self.create_subscription(
            String, str(self.get_parameter("source_topic").value),
            self._source_callback, 20,
        )
        hz = float(self.get_parameter("publish_hz").value)
        if hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        self.timer = self.create_timer(1.0 / hz, self._relay_callback)
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat)

    def _heartbeat(self) -> None:
        self.get_logger().info(
            "relay heartbeat "
            f"source={self._source_count} relay={self._relay_count} "
            f"invalid={self._invalid_count} latest={self._latest is not None}")

    def _status(self, code: str, message: str) -> None:
        status = String()
        status.data = json.dumps({
            "schema": "r1_n2_3_relay_status_v1",
            "code": code,
            "message": str(message),
            "monotonic": time.monotonic(),
            "source_count": self._source_count,
            "relay_count": self._relay_count,
            "invalid_count": self._invalid_count,
        }, sort_keys=True)
        self.status_pub.publish(status)

    def _source_callback(self, message: String) -> None:
        receive_monotonic = time.monotonic()
        try:
            payload = json.loads(message.data)
            validate_envelope(payload)
        except (TypeError, ValueError, json.JSONDecodeError, ContractError) as exc:
            self._invalid_count += 1
            self.get_logger().error(f"Rejected planner result: {exc}")
            self._status("invalid_source", str(exc))
            return
        # Diagnostic-only metadata. It does not alter planner_result_monotonic
        # and is copied unchanged across repeated relay publications.
        payload["relay_source_receive_monotonic"] = receive_monotonic
        self._latest = payload
        self._source_count += 1
        self._last_source_identity = tuple(payload[field] for field in (
            "worker_generation", "request_id", "cycle_id", "scan_counter",
            "planner_result_monotonic", "path_version", "goal_version",
        ))

    def _relay_callback(self) -> None:
        if self._latest is None:
            return
        self._sequence += 1
        publish_monotonic = time.monotonic()
        try:
            payload = relay_envelope(self._latest, self._sequence, publish_monotonic)
        except ContractError as exc:
            self._invalid_count += 1
            self.get_logger().error(f"Failed to relay cached planner result: {exc}")
            self._status("invalid_cached_source", str(exc))
            return
        message = String()
        message.data = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.relay_pub.publish(message)
        self._relay_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerCommandRelay()
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
