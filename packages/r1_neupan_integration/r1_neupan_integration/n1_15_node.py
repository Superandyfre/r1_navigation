"""ROS I/O side of the N1.15 whole-core isolation experiment."""

from __future__ import annotations

import atexit
import json
import os
import time
from collections import deque
from typing import Any

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Int8

from neupan_ros2.utils import quat_to_yaw, yaw_to_quat

from r1_neupan_integration.n1_15_whole_core import (
    N115_SCHEMA,
    WholeCoreClient,
)
from r1_neupan_integration.n2_pre_semantics import (
    DEFAULT_GEAR,
    apply_uniform_gear,
    collapse_invalid_stationary_prefix,
    navigation_loop,
    transition_requires_zero,
    validate_gear,
    validate_navigation_mode,
)
from r1_neupan_integration.scan_self_filter import (
    outside_polygon_footprint,
    outside_rectangular_footprint,
    polygon_vertices_from_flat_xy,
)


N115_TIMING_FIELDS = [
    "schema", "cycle_id", "scan_counter", "scan_stamp_sec",
    "scan_receive_monotonic", "path_version", "tf_stamp_sec",
    "cycle_start_monotonic", "worker_core_start_monotonic",
    "worker_core_end_monotonic", "output_publish_monotonic",
    "main_input_extraction_ms", "main_request_prep_ms",
    "main_request_serialization_ms", "main_request_send_ms", "main_wait_ms",
    "main_response_receive_ms", "main_reconstruction_ms", "main_ipc_roundtrip_ms",
    "main_publish_ms", "main_full_cycle_ms", "worker_decode_input_ms",
    "worker_dune_ms", "worker_nrmp_ms", "worker_cvxpy_ms", "worker_pan_ms",
    "worker_core_total_ms", "worker_core_cpu_ms", "worker_result_pack_ms",
    "worker_result_serialization_ms", "worker_response_send_ms",
    "obstacle_points_count", "worker_pan_iterations",
    "worker_pan_early_converged", "worker_dune_effective_points_count",
    "worker_nrmp_effective_points_count", "dropped_obsolete_scans",
    "duplicate_scan", "queue_depth", "request_id", "worker_sequence",
    "worker_pid", "worker_failure", "finite", "nan_inf",
]


def _rotation(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.asarray([[c, -s], [s, c]], dtype=np.float64)


class N115Node(Node):
    """ROS subscriptions and one synchronous latest-only IPC flight."""

    NODE_NAME = "n1_15_whole_core"
    CLIENT_CLASS = WholeCoreClient

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)
        self.state_lock = __import__("threading").Lock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("robot_config_dir", "")
        self.declare_parameter("planner_config_file", "planner.yaml")
        self.declare_parameter("dune_checkpoint_file", "models/dune_model_5000.pth")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan_neupan")
        self.declare_parameter("plan_input_topic", "/r1_test_path")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("gear_topic", "/r1_motion_gear")
        self.declare_parameter("default_gear", DEFAULT_GEAR)
        self.declare_parameter("navigation_mode", "cyclic")
        self.declare_parameter("cmd_vel_topic", "/neupan_cmd_vel")
        self.declare_parameter("plan_output_topic", "/neupan_plan")
        self.declare_parameter("ref_state_topic", "/neupan_ref_state")
        self.declare_parameter("timing_topic", "/neupan_timing")
        self.declare_parameter("scan_range_min", 0.15)
        self.declare_parameter("scan_range_max", 8.0)
        self.declare_parameter("scan_angle_min", -np.pi)
        self.declare_parameter("scan_angle_max", np.pi)
        self.declare_parameter("scan_downsample", 1)
        self.declare_parameter("flip_angle", False)
        self.declare_parameter("self_filter_enabled", False)
        self.declare_parameter("self_filter_front_m", 0.0)
        self.declare_parameter("self_filter_rear_m", 0.0)
        self.declare_parameter("self_filter_left_m", 0.0)
        self.declare_parameter("self_filter_right_m", 0.0)
        self.declare_parameter(
            "self_filter_polygon_xy", Parameter.Type.DOUBLE_ARRAY
        )
        self.declare_parameter("self_filter_polygon_margin_m", 0.0)
        self.declare_parameter("include_initial_path_direction", True)
        self.declare_parameter("refresh_initial_path", False)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("control_frequency", 50.0)
        self.declare_parameter("latest_scan_only", True)
        self.declare_parameter("n115_output_dir", "n1_results/n1_15_run")
        self.declare_parameter("n115_exact_capture_dir", "")
        self.declare_parameter("n115_exact_capture_max_records", 512)
        self.declare_parameter("n115_capture_scene", "")
        self.declare_parameter("n115_capture_metadata", "{}")
        self.declare_parameter("n115_warmup_count", 30)
        self.declare_parameter("n115_timeout_sec", 30.0)
        self.declare_parameter("planner_receding_override", 0)
        self.declare_parameter("planner_iter_num_override", 0)
        self.declare_parameter("planner_dune_max_num_override", 0)
        self.declare_parameter("planner_nrmp_max_num_override", 0)
        self.declare_parameter("planner_iter_threshold_override", -1.0)
        self.declare_parameter("torch_num_threads", 1)
        self.declare_parameter("torch_num_interop_threads", 24)
        self.declare_client_parameters()

        config_dir = str(self.get_parameter("robot_config_dir").value)
        config_name = str(self.get_parameter("planner_config_file").value)
        checkpoint_name = str(self.get_parameter("dune_checkpoint_file").value)
        config_path = os.path.join(config_dir, config_name)
        checkpoint_path = os.path.join(config_dir, checkpoint_name)
        if not os.path.isfile(config_path) or not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(f"N1.15 planner inputs missing: {config_path}, {checkpoint_path}")

        import yaml
        with open(config_path, "r", encoding="utf-8") as stream:
            yaml_config = yaml.safe_load(stream) or {}
        pan = dict(yaml_config.get("pan") or {})
        for name, key in (
            ("planner_iter_num_override", "iter_num"),
            ("planner_dune_max_num_override", "dune_max_num"),
            ("planner_nrmp_max_num_override", "nrmp_max_num"),
        ):
            value = int(self.get_parameter(name).value)
            if value > 0:
                pan[key] = value
        threshold = float(self.get_parameter("planner_iter_threshold_override").value)
        if threshold >= 0.0:
            pan["iter_threshold"] = threshold
        receding = int(self.get_parameter("planner_receding_override").value)
        try:
            capture_metadata = json.loads(
                str(self.get_parameter("n115_capture_metadata").value)
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            raise ValueError("n115_capture_metadata must be a JSON object")
        if not isinstance(capture_metadata, dict):
            raise ValueError("n115_capture_metadata must decode to a JSON object")
        capture_metadata.setdefault(
            "scene", str(self.get_parameter("n115_capture_scene").value)
        )
        self.client = self.make_client(
            str(self.get_parameter("n115_output_dir").value),
            config_path,
            checkpoint_path,
            pan,
            receding if receding > 0 else None,
            int(self.get_parameter("torch_num_threads").value),
            int(self.get_parameter("torch_num_interop_threads").value),
            int(self.get_parameter("n115_warmup_count").value),
            float(self.get_parameter("n115_timeout_sec").value),
            str(self.get_parameter("n115_exact_capture_dir").value),
            int(self.get_parameter("n115_exact_capture_max_records").value),
            capture_metadata,
        )
        atexit.register(self.finalize)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        self.scan_range = np.asarray([
            float(self.get_parameter("scan_range_min").value),
            float(self.get_parameter("scan_range_max").value),
        ])
        self.scan_angle_range = np.asarray([
            float(self.get_parameter("scan_angle_min").value),
            float(self.get_parameter("scan_angle_max").value),
        ])
        self.scan_downsample = int(self.get_parameter("scan_downsample").value)
        if self.scan_downsample <= 0:
            raise ValueError("scan_downsample must be positive")
        self.flip_angle = bool(self.get_parameter("flip_angle").value)
        self.self_filter_enabled = bool(self.get_parameter("self_filter_enabled").value)
        self.self_filter_extents = {
            "front_m": float(self.get_parameter("self_filter_front_m").value),
            "rear_m": float(self.get_parameter("self_filter_rear_m").value),
            "left_m": float(self.get_parameter("self_filter_left_m").value),
            "right_m": float(self.get_parameter("self_filter_right_m").value),
        }
        # The HW0 rectangle profile deliberately omits this optional HW1
        # polygon. A type-only declaration is uninitialized in Humble, so use
        # the established rectangular fallback rather than aborting startup.
        try:
            polygon_values = self.get_parameter("self_filter_polygon_xy").value
        except ParameterUninitializedException:
            polygon_values = []
        self.self_filter_polygon = (
            polygon_vertices_from_flat_xy(polygon_values)
            if polygon_values else None
        )
        self.self_filter_polygon_margin_m = float(
            self.get_parameter("self_filter_polygon_margin_m").value
        )
        if self.self_filter_enabled and self.lidar_frame != self.base_frame:
            raise ValueError(
                "self_filter_enabled requires scan coordinates in base_frame"
            )
        if self.self_filter_enabled:
            if self.self_filter_polygon is None:
                outside_rectangular_footprint(
                    np.empty(0), np.empty(0), **self.self_filter_extents
                )
            elif self.self_filter_polygon_margin_m < 0.0:
                raise ValueError("self_filter_polygon_margin_m must be nonnegative")
        self.include_path_direction = bool(self.get_parameter("include_initial_path_direction").value)
        self.refresh_initial_path = bool(self.get_parameter("refresh_initial_path").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.latest_scan_only = bool(self.get_parameter("latest_scan_only").value)

        self.robot_state = None
        self.obstacle_points = None
        self.latest_scan_ranges = None
        self.latest_scan_stamp_sec = -1.0
        self.latest_scan_receive_monotonic = -1.0
        self.latest_scan_update_monotonic = -1.0
        self.latest_scan_frame_id = ""
        self.latest_scan_meta = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.latest_scan_metrics = {}
        self.latest_scan_filter = {}
        self.scan_counter = 0
        self.last_processed_scan = 0
        self.scans_received = 0
        self.unique_processed = 0
        self.obsolete_dropped = 0
        self.duplicate_scans = 0
        self.regressed_scan_stamps = 0
        # A received LaserScan can be replayed after newer scans have already
        # arrived. Keep a bounded history instead of comparing only the last
        # stamp so neither form can reach the planner twice.
        self._recent_scan_stamps = deque()
        self._recent_scan_stamp_set = set()
        self._last_accepted_scan_stamp_ns = -1
        self.path_matrix = None
        self.path_version = 0
        self.goal = None
        self.goal_version = -1
        self.navigation_mode = validate_navigation_mode(
            str(self.get_parameter("navigation_mode").value)
        )
        self.navigation_loop = navigation_loop(self.navigation_mode)
        self.active_gear = validate_gear(self.get_parameter("default_gear").value)
        self.requested_gear = self.active_gear
        self.pending_gear = None
        self.gear_transition_count = 0
        self._raw_path_matrix = None
        self.cycle_id = 0
        self._finalized = False

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.plan_pub = self.create_publisher(Path, str(self.get_parameter("plan_output_topic").value), 10)
        self.ref_pub = self.create_publisher(Path, str(self.get_parameter("ref_state_topic").value), 10)
        self.timing_pub = self.create_publisher(Float64MultiArray, str(self.get_parameter("timing_topic").value), 50)
        qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self.scan_callback, qos, callback_group=self.callback_group)
        self.create_subscription(Path, str(self.get_parameter("plan_input_topic").value), self.path_callback, 10, callback_group=self.callback_group)
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self.goal_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Int8, str(self.get_parameter("gear_topic").value), self.gear_callback, 10, callback_group=self.callback_group)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        frequency = float(self.get_parameter("control_frequency").value)
        self.create_timer(1.0 / frequency, self.run, callback_group=self.control_group)
        self.get_logger().info("N1.15 ROS I/O node ready; planner exists only in the spawn worker")

    def declare_client_parameters(self) -> None:
        """Hook for a production bridge subclass to add client-only options."""

    def make_client(self, output_dir: str, config_path: str, checkpoint_path: str,
                    pan: dict[str, Any], receding: int | None,
                    torch_intraop: int, torch_interop: int, warmup_count: int,
                    timeout_sec: float, exact_capture_dir: str = "",
                    exact_capture_max_records: int = 512,
                    capture_metadata: dict[str, Any] | None = None) -> WholeCoreClient:
        return self.CLIENT_CLASS(
            output_dir, config_path, checkpoint_path, pan, receding,
            torch_intraop, torch_interop, warmup_count, timeout_sec,
            exact_capture_dir=exact_capture_dir,
            exact_capture_max_records=exact_capture_max_records,
            capture_metadata=capture_metadata,
        )

    def _robot_transform(self) -> bool:
        # N1.16 may deliberately fail only this consumer-side lookup during a
        # shadow safety test. It never changes TF, localization, or sensors.
        if getattr(self, "_shadow_force_robot_tf_failure", False):
            return False
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        with self.state_lock:
            self.robot_state = np.asarray([
                [float(trans.transform.translation.x)],
                [float(trans.transform.translation.y)],
                [quat_to_yaw(trans.transform.rotation)],
            ], dtype=np.float64)
            self.tf_stamp_sec = float(trans.header.stamp.sec) + float(trans.header.stamp.nanosec) * 1e-9
        return True

    def scan_callback(self, message: LaserScan) -> None:
        started = time.perf_counter()
        receive = time.monotonic()
        ranges = np.asarray(message.ranges, dtype=np.float64)
        stamp_sec = int(message.header.stamp.sec)
        stamp_nanosec = int(message.header.stamp.nanosec)
        stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
        stamp_key = (stamp_sec, stamp_nanosec, str(message.header.frame_id))
        rejected_reason = None
        with self.state_lock:
            self.scans_received += 1
            if stamp_key in self._recent_scan_stamp_set:
                self.duplicate_scans += 1
                rejected_reason = "duplicate"
            elif (self._last_accepted_scan_stamp_ns >= 0
                  and stamp_ns < self._last_accepted_scan_stamp_ns):
                self.regressed_scan_stamps += 1
                rejected_reason = "timestamp_regressed"
            else:
                self._recent_scan_stamps.append(stamp_key)
                self._recent_scan_stamp_set.add(stamp_key)
                if len(self._recent_scan_stamps) > 128:
                    self._recent_scan_stamp_set.discard(self._recent_scan_stamps.popleft())
                self._last_accepted_scan_stamp_ns = stamp_ns
                self.scan_counter += 1
                self.latest_scan_stamp_sec = float(stamp_sec) + float(stamp_nanosec) * 1e-9
        if rejected_reason is not None:
            self.get_logger().warning(
                "discarded " + rejected_reason
                + " LaserScan stamp=" + str(stamp_sec)
                + "." + f"{stamp_nanosec:09d}"
                + " frame=" + str(message.header.frame_id)
            )
            return
        with self.state_lock:
            self.latest_scan_receive_monotonic = receive
            self.latest_scan_ranges = ranges.copy()
            self.latest_scan_frame_id = str(message.header.frame_id)
            self.latest_scan_meta = (
                float(message.angle_min), float(message.angle_max),
                float(message.angle_increment), float(message.range_min),
                float(message.range_max),
            )
            state = None if self.robot_state is None else self.robot_state.copy()
        if state is None:
            return
        conversion_started = time.perf_counter()
        angles = np.linspace(float(message.angle_min), float(message.angle_max), len(ranges))
        if self.flip_angle:
            angles = np.flip(angles)
        conversion_ms = (time.perf_counter() - conversion_started) * 1000.0
        filter_started = time.perf_counter()
        indices = np.arange(len(ranges))
        mask = (
            (indices % self.scan_downsample == 0)
            & (ranges >= self.scan_range[0])
            & (ranges <= self.scan_range[1])
            & (angles > self.scan_angle_range[0])
            & (angles < self.scan_angle_range[1])
        )
        valid_ranges, valid_angles = ranges[mask], angles[mask]
        filter_ms = (time.perf_counter() - filter_started) * 1000.0
        filter_shape = "no_valid_points"
        filter_input_count = 0
        retained_count = 0
        nearest_distance = None
        nearest_xy = None
        if valid_ranges.size == 0:
            points = None
            transform_ms = 0.0
            tf_lookup_ms = 0.0
        else:
            transform_started = time.perf_counter()
            local_points = np.vstack([
                valid_ranges * np.cos(valid_angles),
                valid_ranges * np.sin(valid_angles),
            ])
            # PointCloudToLaserScan now preserves the real lidar_link frame to
            # avoid its timestamp-sensitive TF filter. Convert locally here so
            # the R1 footprint filter and map projection remain base-frame
            # correct without delaying live scans.
            tf_started = time.perf_counter()
            try:
                scan_frame = str(message.header.frame_id) or self.lidar_frame
                if scan_frame != self.lidar_frame:
                    scan_to_lidar = self.tf_buffer.lookup_transform(
                        self.lidar_frame, scan_frame, rclpy.time.Time()
                    )
                    scan_yaw = quat_to_yaw(scan_to_lidar.transform.rotation)
                    scan_translation = np.asarray([
                        float(scan_to_lidar.transform.translation.x),
                        float(scan_to_lidar.transform.translation.y),
                    ])
                    local_points = (
                        _rotation(float(scan_yaw)) @ local_points
                        + scan_translation[:, None]
                    )
                filter_input_count = int(local_points.shape[1])
                if self.self_filter_enabled:
                    local_ranges = np.hypot(local_points[0], local_points[1])
                    local_angles = np.arctan2(local_points[1], local_points[0])
                    if self.self_filter_polygon is None:
                        keep = outside_rectangular_footprint(
                            local_ranges, local_angles, **self.self_filter_extents
                        )
                        filter_shape = "rectangle"
                    else:
                        keep = outside_polygon_footprint(
                            local_points, self.self_filter_polygon,
                            margin_m=self.self_filter_polygon_margin_m,
                        )
                        filter_shape = "polygon"
                    local_points = local_points[:, keep]
                else:
                    filter_shape = "disabled"
                retained_count = int(local_points.shape[1])
                if retained_count:
                    nearest_index = int(np.argmin(np.hypot(local_points[0], local_points[1])))
                    nearest_point = local_points[:, nearest_index]
                    nearest_distance = float(np.hypot(nearest_point[0], nearest_point[1]))
                    nearest_xy = [float(nearest_point[0]), float(nearest_point[1])]
                else:
                    nearest_distance = None
                    nearest_xy = None
                trans = self.tf_buffer.lookup_transform(self.map_frame, self.lidar_frame, rclpy.time.Time())
                tf_lookup_ms = (time.perf_counter() - tf_started) * 1000.0
                yaw = quat_to_yaw(trans.transform.rotation)
                pose = np.asarray([
                    float(trans.transform.translation.x),
                    float(trans.transform.translation.y), yaw,
                ])
                points = _rotation(float(pose[2])) @ local_points + pose[:2, None]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                points = None
                tf_lookup_ms = (time.perf_counter() - tf_started) * 1000.0
            transform_ms = (time.perf_counter() - transform_started) * 1000.0
        with self.state_lock:
            self.obstacle_points = None if points is None else points.copy()
            self.latest_scan_update_monotonic = time.monotonic()
            self.latest_scan_metrics = {
                "scan_processing_ms": (time.perf_counter() - started) * 1000.0,
                "scan_conversion_ms": conversion_ms,
                "scan_filter_ms": filter_ms,
                "scan_tf_lookup_ms": tf_lookup_ms,
                "scan_transform_ms": transform_ms,
            }
            self.latest_scan_filter = {
                "shape": filter_shape if valid_ranges.size else "no_valid_points",
                "input_points": int(valid_ranges.size),
                "self_rejected_points": int(filter_input_count - retained_count)
                    if valid_ranges.size else 0,
                "retained_points": retained_count if valid_ranges.size else 0,
                "nearest_retained_base_link": nearest_xy if valid_ranges.size else None,
                "nearest_retained_distance_m": nearest_distance if valid_ranges.size else None,
            }

    def path_callback(self, message: Path) -> None:
        if not message.poses:
            return
        coordinates = []
        if self.include_path_direction:
            coordinates = [
                (p.pose.position.x, p.pose.position.y, quat_to_yaw(p.pose.orientation))
                for p in message.poses
            ]
        else:
            xy = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
            xs, ys = np.asarray(xy, dtype=np.float64).T
            dx, dy = np.diff(xs, append=xs[-1]), np.diff(ys, append=ys[-1])
            theta = np.arctan2(dy, dx)
            if len(theta) > 1:
                theta[-1] = theta[-2]
            coordinates = list(zip(xs, ys, theta))
        matrix = np.vstack([
            np.asarray([value[0] for value in coordinates]),
            np.asarray([value[1] for value in coordinates]),
            np.asarray([value[2] for value in coordinates]),
            np.full(len(coordinates), self.active_gear, dtype=np.float64),
        ]).astype(np.float64)
        sanitized_matrix = collapse_invalid_stationary_prefix(matrix)
        if sanitized_matrix.shape[1] != matrix.shape[1]:
            self.get_logger().warn(
                "Collapsed inconsistent stationary prefix in NeuPAN input path: "
                f"{matrix.shape[1]} -> {sanitized_matrix.shape[1]} poses")
        matrix = sanitized_matrix
        with self.state_lock:
            self._raw_path_matrix = matrix[:3, :].copy()
            # The Nav2 bridge publishes a path when its terminal goal changes
            # and when it re-arms a previously failed action.  The latter can
            # deliberately have the same terminal pose, but is still a new
            # controller context.  Advance both versions on every bridge path
            # publication so retried actions cannot reject fresh envelopes as
            # context-mismatched.
            path_goal = matrix[:3, -1:].copy()
            self.path_matrix = matrix
            self.path_version += 1
            # Treat the endpoint as the authoritative companion goal.  The
            # separate /neupan_goal callback sees the same pose and therefore
            # does not consume a second version.
            self.goal = path_goal
            self.goal_version += 1

    def gear_callback(self, message: Int8) -> None:
        try:
            requested = validate_gear(message.data)
        except ValueError as exc:
            self.get_logger().error(f"Ignoring invalid R1 gear request: {exc}")
            return
        with self.state_lock:
            self.requested_gear = requested
            if transition_requires_zero(self.active_gear, requested):
                self.pending_gear = requested
            else:
                self.pending_gear = None

    def _consume_gear_transition(self) -> dict[str, Any] | None:
        """Emit one adapter zero, then make the requested gear active."""
        with self.state_lock:
            if self.pending_gear is None:
                return None
            old_gear = self.active_gear
            new_gear = validate_gear(self.pending_gear)
            self.active_gear = new_gear
            self.pending_gear = None
            if self._raw_path_matrix is not None:
                self.path_matrix = apply_uniform_gear(self._raw_path_matrix, new_gear)
                self.path_version += 1
            self.gear_transition_count += 1
            return {
                "old_gear": old_gear,
                "new_gear": new_gear,
                "zero_command_emitted": True,
                "transition_index": self.gear_transition_count,
                "path_version": int(self.path_version),
            }

    def goal_callback(self, message: PoseStamped) -> None:
        goal = np.asarray([
            [float(message.pose.position.x)],
            [float(message.pose.position.y)],
            [quat_to_yaw(message.pose.orientation)],
        ], dtype=np.float64)
        with self.state_lock:
            # A goal derived from the just received path is the same logical
            # context and must not consume a second version.
            if self.goal is None or not np.allclose(
                    self.goal, goal, rtol=0.0, atol=1e-4):
                self.goal = goal
                self.goal_version += 1

    @staticmethod
    def _path_message(array: np.ndarray, frame: str, stamp: Any) -> Path:
        message = Path()
        message.header.frame_id = frame
        message.header.stamp = stamp
        if array.size == 0:
            return message
        for x, y, yaw in array[:3, :].T:
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation = yaw_to_quat(float(yaw))
            message.poses.append(pose)
        return message

    def run(self) -> None:
        cycle_start = time.monotonic()
        transition = self._consume_gear_transition()
        if transition is not None:
            self.cmd_pub.publish(Twist())
            return
        if not self._robot_transform():
            return
        with self.state_lock:
            if self.path_matrix is None or self.robot_state is None:
                return
            scan_counter = self.scan_counter
            if self.latest_scan_only and scan_counter <= self.last_processed_scan:
                return
            state = self.robot_state.copy()
            points = np.empty((2, 0), dtype=np.float64) if self.obstacle_points is None else self.obstacle_points.copy()
            path = self.path_matrix.copy()
            path_version = self.path_version
            goal = None if self.goal is None else self.goal.copy()
            goal_version = self.goal_version
            scan_stamp = self.latest_scan_stamp_sec
            scan_receive = self.latest_scan_receive_monotonic
            scan_metrics = dict(self.latest_scan_metrics)
            tf_stamp = getattr(self, "tf_stamp_sec", -1.0)
            active_gear = int(self.active_gear)
            requested_gear = int(self.requested_gear)
            gear_transition_pending = self.pending_gear is not None
            previous_scan = self.last_processed_scan
            self.last_processed_scan = scan_counter
        obsolete = 0 if previous_scan == 0 else max(0, scan_counter - previous_scan - 1)
        self.obsolete_dropped += obsolete
        self.unique_processed += 1
        self.cycle_id += 1
        try:
            self.client.startup(
                path, state, points, path_version,
                navigation_loop_mode=self.navigation_mode,
                active_gear=active_gear,
            )
            # Worker construction and warmup are startup evidence, not a
            # formal planner cycle.  Begin the cycle clock only after READY.
            cycle_start = time.monotonic()
            input_finished = time.perf_counter()
            outputs, record = self.client.solve(
                state, points, path, path_version, scan_counter, scan_stamp,
                goal=goal, goal_version=goal_version, cycle_id=self.cycle_id,
                navigation_loop_mode=self.navigation_mode,
                active_gear=active_gear,
            )
        except Exception as exc:
            self.get_logger().error(f"N1.15 worker request failed: {type(exc).__name__}: {exc}")
            return
        timing = dict(record["timings"])
        timing["main_input_extraction_ms"] = (input_finished - cycle_start) * 1000.0
        timing["dropped_obsolete_scans"] = float(obsolete)
        timing["duplicate_scan"] = 0.0
        timing["queue_depth"] = 1.0
        timing["request_id"] = float(record["request_id"])
        timing["worker_sequence"] = float(record.get("worker_sequence", -1))
        timing["worker_pid"] = float(record.get("worker_pid", -1))
        timing["worker_failure"] = 0.0
        timing["finite"] = 1.0 if record.get("finite") else 0.0
        timing["nan_inf"] = 0.0 if record.get("finite") else 1.0
        timing["obstacle_points_count"] = float(points.shape[1])
        timing.update(scan_metrics)
        publish_started = time.perf_counter()
        stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(self._path_message(outputs["opt_state"], self.map_frame, stamp))
        self.ref_pub.publish(self._path_message(outputs["ref_state"], self.map_frame, stamp))
        action = outputs["action"]
        command = Twist()
        stop_requested = bool(record.get("info", {}).get("stop"))
        arrived = bool(record.get("info", {}).get("arrive"))
        if not arrived and action.size >= 2:
            candidate = action
            if stop_requested:
                candidate = outputs.get("optimized_velocity", action)
            if candidate.size >= 2:
                command.linear.x = float(candidate.reshape(-1)[0])
                command.angular.z = float(candidate.reshape(-1)[1])
                if stop_requested and not self.allow_reverse:
                    # Retain the planner's turn-away direction while a stop
                    # result asks for reverse. Never translate on that path.
                    command.linear.x = 0.0
                elif not self.allow_reverse and command.linear.x < 0.0:
                    command.linear.x = 0.0
        self.cmd_pub.publish(command)
        output_publish = time.monotonic()
        timing["main_publish_ms"] = (time.perf_counter() - publish_started) * 1000.0
        timing["main_full_cycle_ms"] = (output_publish - cycle_start) * 1000.0
        record["timings"] = timing
        record["r1_semantics"] = {
            "navigation_mode": self.navigation_mode,
            "loop": self.navigation_loop,
            "active_gear": active_gear,
            "requested_gear": requested_gear,
            "gear_transition_pending": gear_transition_pending,
            "gear_values": [int(value) for value in path[3, :].tolist()],
        }
        timing_record = {
            "schema": N115_SCHEMA,
            "cycle_id": float(self.cycle_id),
            "scan_counter": float(scan_counter),
            "scan_stamp_sec": float(scan_stamp),
            "scan_receive_monotonic": float(scan_receive),
            "path_version": float(path_version),
            "tf_stamp_sec": float(tf_stamp),
            "cycle_start_monotonic": float(cycle_start),
            "worker_core_start_monotonic": float(timing.get("worker_core_start_monotonic", cycle_start)),
            "worker_core_end_monotonic": float(timing.get("worker_core_end_monotonic", output_publish)),
            "output_publish_monotonic": float(output_publish),
        }
        for field in N115_TIMING_FIELDS:
            if field not in timing_record:
                timing_record[field] = float(timing.get(field, 0.0))
        message = Float64MultiArray()
        message.data = [float(timing_record[field]) for field in N115_TIMING_FIELDS]
        self.timing_pub.publish(message)
        record["timing_record"] = timing_record

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self.client.set_scan_accounting(
            scans_received=self.scans_received,
            unique_processed=self.unique_processed,
            obsolete_dropped=self.obsolete_dropped,
            duplicate=self.duplicate_scans,
        )
        self.client.finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = N115Node()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
