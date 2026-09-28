#!/usr/bin/env python

"""
NeupanCore is the main ROS2 node for the NeuPAN navigation algorithm.

This node subscribes to laser scan and localization data, executes the NeuPAN
planning algorithm, and publishes velocity commands to control the robot.

Developer: Han Ruihua <hanrh@connect.hku.hk>  Li Chengyang <kevinladlee@gmail.com>
Date: 2025.04.08
"""
import atexit
import hashlib
import json
import os
import subprocess
import threading
import time
import traceback
from typing import Optional, Tuple, Dict, Any, List

import yaml

import numpy as np
import numpy.typing as npt
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

try:
    from neupan import neupan
    from neupan.util import get_transform
except ImportError as e:
    raise ImportError(
        f"Failed to import 'neupan' package: {e}. "
        "Please install NeuPAN first."
    ) from e

# Import local modules
from neupan_ros2.visualization_manager import VisualizationManager
from neupan_ros2.utils import yaw_to_quat, quat_to_yaw

try:
    from r1_neupan_integration import n1_11_diagnostics
except ImportError:
    n1_11_diagnostics = None

try:
    from r1_neupan_integration import n1_13_online_diagnostics
except ImportError:
    n1_13_online_diagnostics = None

try:
    from r1_neupan_integration import n1_14_process_isolation
except ImportError:
    n1_14_process_isolation = None


def _array_value(value: Any, shape: Optional[tuple[int, ...]] = None) -> np.ndarray:
    """Convert diagnostic values without retaining Torch graphs."""
    if value is None:
        return np.empty(shape or (0,), dtype=float)
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def _read_text(path: str) -> Optional[str]:
    try:
        with open(path, 'r', encoding='ascii') as stream:
            return stream.read().strip()
    except (OSError, UnicodeError):
        return None


def _runtime_snapshot() -> Dict[str, Any]:
    """Collect low-cost scheduler and host evidence at a selected cycle."""
    status = {}
    raw_status = _read_text('/proc/self/status') or ''
    for line in raw_status.splitlines():
        if line.startswith(('voluntary_ctxt_switches:', 'nonvoluntary_ctxt_switches:')):
            key, value = line.split(':', 1)
            try:
                status[key] = int(value.strip())
            except ValueError:
                status[key] = value.strip()
    schedstat = _read_text('/proc/self/schedstat')
    frequencies = {
        path.parts[-3]: _read_text(str(path))
        for path in sorted(
            __import__('pathlib').Path('/sys/devices/system/cpu').glob(
                'cpu[0-9]*/cpufreq/scaling_cur_freq'
            )
        )[:24]
    }
    governors = {
        path.parts[-3]: _read_text(str(path))
        for path in sorted(
            __import__('pathlib').Path('/sys/devices/system/cpu').glob(
                'cpu[0-9]*/cpufreq/scaling_governor'
            )
        )[:24]
    }
    thermal = {
        path.name: _read_text(str(path / 'temp'))
        for path in __import__('pathlib').Path('/sys/class/thermal').glob('thermal_zone*')
    }
    return {
        'load_average': list(os.getloadavg()),
        'cpu_frequency_raw': frequencies,
        'cpu_governors': governors,
        'cpu_temperature_raw': thermal,
        'process': {
            'pid': os.getpid(),
            'voluntary_context_switches': status.get('voluntary_ctxt_switches'),
            'involuntary_context_switches': status.get('nonvoluntary_ctxt_switches'),
            'schedstat': schedstat,
            'process_time_sec': time.process_time(),
        },
        'background_heavy_processes': _read_heavy_processes(),
    }


def _read_heavy_processes() -> Any:
    try:
        output = subprocess.run(
            ['ps', '-eo', 'pid,pcpu,comm', '--sort=-pcpu'],
            text=True, capture_output=True, timeout=0.5, check=False,
        ).stdout.splitlines()
        return output[:8]
    except (OSError, subprocess.TimeoutExpired):
        return 'not collected'


def _file_sha256(path: str) -> Optional[str]:
    digest = hashlib.sha256()
    try:
        with open(path, 'rb') as stream:
            for block in iter(lambda: stream.read(1 << 20), b''):
                digest.update(block)
    except OSError:
        return None
    return digest.hexdigest()


class NeupanCore(Node):
    """ROS2 node for NeuPAN navigation algorithm.

    This node integrates the NeuPAN planner with ROS2, handling sensor data,
    executing planning, and publishing control commands.
    """

    def __init__(self) -> None:
        super().__init__("neupan_node")

        # Thread lock protecting shared state: robot_state, obstacle_points, stop, arrive
        # These are accessed by both control thread (run) and callback thread (scan/path/goal)
        self._state_lock = threading.Lock()

        # Callback groups for multi-threaded execution
        # Control group: MutuallyExclusive for timer (run) - ensures run() executes alone
        self.control_group = MutuallyExclusiveCallbackGroup()
        # Callback group: Reentrant for all subscriptions - allows concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Package directory for accessing config files and models
        self.pkg_dir = get_package_share_directory("neupan_ros2")

        # Robot identification and configuration directory
        self.declare_parameter("robot_type", "")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_config_dir", "")  # Set by launch file
        self.declare_parameter("planner_config_file", "planner.yaml")
        self.declare_parameter("dune_checkpoint_file", "models/dune_model_5000.pth")

        # Legacy parameter name (for backward compatibility)
        self.declare_parameter("neupan_config_file", "NOT SET")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "laser_link")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("marker_z", 1.0)
        self.declare_parameter("scan_angle_max", 3.14)
        self.declare_parameter("scan_angle_min", -3.14)
        self.declare_parameter("scan_downsample", 1)
        self.declare_parameter("scan_range_min", 0.1)
        self.declare_parameter("scan_range_max", 5.0)
        self.declare_parameter("refresh_initial_path", False)
        self.declare_parameter("flip_angle", False)
        self.declare_parameter("include_initial_path_direction", False)
        self.declare_parameter("control_frequency", 50.0)  # Control loop frequency in Hz
        self.declare_parameter("latest_scan_only", False)

        # N1.5 diagnostic overrides.  A non-positive value means keep the YAML
        # value, so the normal N1 configuration remains unchanged.
        self.declare_parameter("planner_receding_override", 0)
        self.declare_parameter("planner_iter_num_override", 0)
        self.declare_parameter("planner_dune_max_num_override", 0)
        self.declare_parameter("planner_nrmp_max_num_override", 0)
        self.declare_parameter("planner_iter_threshold_override", -1.0)
        self.declare_parameter("input_capture_file", "")
        # N1.10 attribution capture.  These parameters only record existing
        # planner inputs and timings; they do not alter planning behavior.
        self.declare_parameter("n110_snapshot_dir", "")
        self.declare_parameter("n110_normal_max_ms", 20.0)
        self.declare_parameter("n110_medium_min_ms", 50.0)
        self.declare_parameter("n110_medium_max_ms", 200.0)
        self.declare_parameter("n110_spike_min_ms", 300.0)
        self.declare_parameter("n111_snapshot_dir", "")
        self.declare_parameter("n113_output_dir", "")
        self.declare_parameter("n113_shadow_enabled", True)
        self.declare_parameter("n114_output_dir", "")
        self.declare_parameter("n114_warmup_count", 30)
        self.declare_parameter("n114_timeout_sec", 30.0)

        # Visualization control parameters
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("enable_dune_markers", True)
        self.declare_parameter("enable_nrmp_markers", True)
        self.declare_parameter("enable_robot_marker", True)

        # Topic names (configurable for flexibility)
        self.declare_parameter("cmd_vel_topic", "/neupan_cmd_vel")
        self.declare_parameter("plan_output_topic", "/neupan_plan")
        self.declare_parameter("ref_state_topic", "/neupan_ref_state")
        self.declare_parameter("initial_path_topic", "/neupan_initial_path")
        self.declare_parameter("dune_markers_topic", "/dune_point_markers")
        self.declare_parameter("robot_marker_topic", "/robot_marker")
        self.declare_parameter("nrmp_markers_topic", "/nrmp_point_markers")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("plan_input_topic", "/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        # Optional timing stream for integration validation.  The stream is
        # diagnostic-only and does not alter planner inputs or outputs.
        self.declare_parameter("timing_topic", "/neupan_timing")

        # === Configuration Loading ===
        # Get robot configuration directory (set by launch file)
        robot_config_dir = (
            self.get_parameter("robot_config_dir")
            .get_parameter_value().string_value
        )

        # Validate robot config directory exists
        if not robot_config_dir or not os.path.isdir(robot_config_dir):
            raise ValueError(
                f"Invalid robot_config_dir: '{robot_config_dir}'. "
                "Must be set by launch file to a valid robot config directory."
            )

        # Get robot type for logging
        robot_type = (
            self.get_parameter("robot_type")
            .get_parameter_value().string_value
        )
        robot_description = (
            self.get_parameter("robot_description")
            .get_parameter_value().string_value
        )

        self.get_logger().info(f"Loading robot configuration: {robot_type}")
        self.get_logger().info(f"Description: {robot_description}")
        self.get_logger().info(f"Config directory: {robot_config_dir}")

        # Load planner configuration (relative to robot config dir)
        planner_config_file = (
            self.get_parameter("planner_config_file")
            .get_parameter_value().string_value
        )
        self.planner_config_file = os.path.join(robot_config_dir, planner_config_file)

        # Load DUNE checkpoint (relative to robot config dir)
        dune_checkpoint_file = (
            self.get_parameter("dune_checkpoint_file")
            .get_parameter_value().string_value
        )
        self.dune_checkpoint = os.path.join(robot_config_dir, dune_checkpoint_file)

        # Validate configuration files exist
        if not os.path.isfile(self.planner_config_file):
            raise FileNotFoundError(
                f"Planner config not found: {self.planner_config_file}"
            )
        if not os.path.isfile(self.dune_checkpoint):
            raise FileNotFoundError(
                f"DUNE checkpoint not found: {self.dune_checkpoint}"
            )

        self.get_logger().info(f"Planner config: {self.planner_config_file}")
        self.get_logger().info(f"DUNE checkpoint: {self.dune_checkpoint}")

        # Load other parameters
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.marker_z = self.get_parameter("marker_z").get_parameter_value().double_value

        self.scan_range = np.array([
            self.get_parameter("scan_range_min").get_parameter_value().double_value,
            self.get_parameter("scan_range_max").get_parameter_value().double_value
        ])

        self.scan_angle_range = np.array([
            self.get_parameter("scan_angle_min").get_parameter_value().double_value,
            self.get_parameter("scan_angle_max").get_parameter_value().double_value
        ])

        self.scan_downsample = (
            self.get_parameter("scan_downsample")
            .get_parameter_value().integer_value
        )

        self.refresh_initial_path = (
            self.get_parameter("refresh_initial_path")
            .get_parameter_value().bool_value
        )
        self.flip_angle = (
            self.get_parameter("flip_angle")
            .get_parameter_value().bool_value
        )
        self.include_initial_path_direction = (
            self.get_parameter("include_initial_path_direction")
            .get_parameter_value().bool_value
        )

        self.enable_visualization = (
            self.get_parameter("enable_visualization")
            .get_parameter_value().bool_value
        )
        self.enable_dune_markers = (
            self.get_parameter("enable_dune_markers")
            .get_parameter_value().bool_value
        )
        self.enable_nrmp_markers = (
            self.get_parameter("enable_nrmp_markers")
            .get_parameter_value().bool_value
        )
        self.enable_robot_marker = (
            self.get_parameter("enable_robot_marker")
            .get_parameter_value().bool_value
        )

        if self.refresh_initial_path:
            self.get_logger().info("Refresh initial path is enabled")

        if not self.planner_config_file:
            raise ValueError(
                "No planner config file provided! "
                "Please set the parameter 'config_file'"
            )

        with open(self.planner_config_file, 'r', encoding='utf-8') as stream:
            planner_yaml = yaml.safe_load(stream) or {}
        pan = dict(planner_yaml.get('pan') or {})
        pan['dune_checkpoint'] = self.dune_checkpoint

        planner_overrides = {
            'receding': self.get_parameter('planner_receding_override').value,
            'iter_num': self.get_parameter('planner_iter_num_override').value,
            'dune_max_num': self.get_parameter('planner_dune_max_num_override').value,
            'nrmp_max_num': self.get_parameter('planner_nrmp_max_num_override').value,
            'iter_threshold': self.get_parameter('planner_iter_threshold_override').value,
        }
        effective_overrides = {}
        for key, value in planner_overrides.items():
            if key == 'receding' and int(value) > 0:
                effective_overrides[key] = int(value)
            elif key != 'receding' and key != 'iter_threshold' and int(value) > 0:
                pan[key] = int(value)
                effective_overrides[key] = int(value)
            elif key == 'iter_threshold' and float(value) >= 0.0:
                pan[key] = float(value)
                effective_overrides[key] = float(value)

        self._planner_overrides = effective_overrides
        init_kwargs = {'pan': pan}
        if 'receding' in effective_overrides:
            init_kwargs['receding'] = effective_overrides['receding']
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file, **init_kwargs
        )
        self._n111_snapshot_dir = str(self.get_parameter('n111_snapshot_dir').value)
        self._n111_cycle_record = None
        self._n111_hook_status = {
            'enabled': False,
            'reason': 'empty_output_dir',
        }
        if self._n111_snapshot_dir and n1_11_diagnostics is not None:
            self._n111_hook_status = n1_11_diagnostics.install(
                self._n111_snapshot_dir
            )
        self._n113_output_dir = str(self.get_parameter('n113_output_dir').value)
        self._n113_shadow_enabled = bool(
            self.get_parameter('n113_shadow_enabled').value
        )
        self._n113_status = {'enabled': False, 'reason': 'empty_output_dir'}
        self._n114_output_dir = str(self.get_parameter('n114_output_dir').value)
        self._n114_client = None
        self._n114_status = {'enabled': False, 'reason': 'empty_output_dir'}
        self._install_core_timing_hooks()

        # Log robot dimensions for verification
        self.get_logger().info(
            f"Robot dimensions - Length: {self.neupan_planner.robot.length:.3f}m, "
            f"Width: {self.neupan_planner.robot.width:.3f}m"
        )
        if hasattr(self.neupan_planner.robot, 'wheelbase') and self.neupan_planner.robot.wheelbase is not None:
            self.get_logger().info(
                f"Robot wheelbase: {self.neupan_planner.robot.wheelbase:.3f}m"
            )
        self.get_logger().info(f"Robot kinematics: {self.neupan_planner.robot.kinematics}")
        self.get_logger().info("NeuPAN planner initialized successfully")

        # Shared state protected by _state_lock (accessed by multiple threads)
        # Write access: scan_callback (obstacle_points), _get_robot_transform (robot_state)
        # Read access: _execute_planning (all), generate_twist_msg (stop, arrive)
        # Planning copies data before execution to minimize lock holding time
        self.obstacle_points: Optional[npt.NDArray] = None  # (2, n) obstacle points in map frame
        self.robot_state: Optional[npt.NDArray] = None  # (3, 1) [x, y, theta] in map frame
        self.stop: bool = False  # Emergency stop flag from collision detection
        self.arrive: bool = False  # Goal reached flag
        self.goal: Optional[npt.NDArray] = None  # (3, 1) target goal [x, y, theta]

        self.latest_scan_only = bool(self.get_parameter('latest_scan_only').value)
        self._cycle_counter = 0
        self._last_processed_scan_counter = 0
        self._skipped_no_fresh_scan_cycles = 0
        self._path_version = 0
        self._last_path_callback_ms = 0.0
        self._latest_scan_metrics = {
            'scan_processing_ms': 0.0,
            'scan_conversion_ms': 0.0,
            'scan_filter_ms': 0.0,
            'scan_tf_lookup_ms': 0.0,
            'scan_transform_ms': 0.0,
            'obstacle_points_count': 0.0,
        }
        self._latest_scan_ranges = None
        self._latest_scan_angle_min = 0.0
        self._latest_scan_angle_max = 0.0
        self._latest_scan_range_min = 0.0
        self._latest_scan_range_max = 0.0
        self._latest_scan_angle_increment = 0.0
        self._latest_scan_frame_id = ''
        self._input_capture_done = False
        self._n110_snapshot_dir = str(self.get_parameter('n110_snapshot_dir').value)
        self._n110_records = []
        self._n110_saved_ids = set()
        self._n110_writer_threads = []
        self._n110_manifest_lock = threading.Lock()
        self._n110_manifest_dirty = False
        if self._n110_snapshot_dir:
            os.makedirs(os.path.abspath(self._n110_snapshot_dir), exist_ok=True)
            atexit.register(self._write_n110_final)
        self._core_stage_timing = {
            'core_forward_ms': 0.0,
            'pan_ms': 0.0,
            'dune_ms': 0.0,
            'nrmp_ms': 0.0,
        }

        if self._n114_output_dir:
            if n1_14_process_isolation is None:
                raise RuntimeError('N1.14 process-isolation module is unavailable')
            # Reconstruct the exact pan input used by the main planner.  The
            # worker receives configuration only at startup; cycle IPC stays
            # numeric-only.
            with open(self.planner_config_file, 'r', encoding='utf-8') as stream:
                yaml_config = yaml.safe_load(stream) or {}
            pan_config = dict(yaml_config.get('pan') or {})
            pan_config['dune_checkpoint'] = self.dune_checkpoint
            self._n114_client = n1_14_process_isolation.IsolatedSolverClient(
                self._n114_output_dir,
                self.planner_config_file,
                self.dune_checkpoint,
                pan_config,
                self._planner_overrides.get('receding'),
                torch_intraop=1,
                torch_interop=24,
                warmup_count=int(self.get_parameter('n114_warmup_count').value),
                timeout_sec=float(self.get_parameter('n114_timeout_sec').value),
            )
            self._n114_status = {
                'enabled': True,
                'output_dir': self._n114_output_dir,
                'start_method': 'spawn',
                'worker_pid': self._n114_client.ready.get('pid'),
                'warmup_count': self._n114_client.ready.get('warmup_count'),
            }
            atexit.register(self._n114_client.finalize)
        self._solver_metrics = {
            'pan_iterations': 0.0,
            'pan_early_converged': 0.0,
            'ecos_solve_ms': 0.0,
            'ecos_solve_cpu_ms': 0.0,
            'ecos_solve_calls': 0.0,
            'solver_failures': 0.0,
            'n114_status': 0.0,
        }
        self._solver_events = []
        self._last_timing_scan_counter = None

        # Latest scan bookkeeping is deliberately separate from the planner
        # state.  It lets the N1 monitor distinguish fresh scans from repeated
        # latest-scan processing without queueing old scans.
        self._scan_counter = 0
        self._latest_scan_stamp_sec = -1.0
        self._latest_scan_receive_monotonic = -1.0

        self.vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10
        )
        self.plan_pub = self.create_publisher(
            Path,
            self.get_parameter("plan_output_topic").get_parameter_value().string_value,
            10
        )
        self.ref_state_pub = self.create_publisher(
            Path,
            self.get_parameter("ref_state_topic").get_parameter_value().string_value,
            10
        )
        self.ref_path_pub = self.create_publisher(
            Path,
            self.get_parameter("initial_path_topic").get_parameter_value().string_value,
            10
        )
        self.timing_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("timing_topic").get_parameter_value().string_value,
            50
        )

        # Initialize visualization manager (handles all visualization independently)
        viz_config = {
            'enable_visualization': self.enable_visualization,
            'enable_dune_markers': self.enable_dune_markers,
            'enable_nrmp_markers': self.enable_nrmp_markers,
            'enable_robot_marker': self.enable_robot_marker,
            'map_frame': self.map_frame,
            'marker_size': self.marker_size,
            'marker_z': self.marker_z,
            'dune_markers_topic': (
                self.get_parameter("dune_markers_topic")
                .get_parameter_value().string_value
            ),
            'nrmp_markers_topic': (
                self.get_parameter("nrmp_markers_topic")
                .get_parameter_value().string_value
            ),
            'robot_marker_topic': (
                self.get_parameter("robot_marker_topic")
                .get_parameter_value().string_value
            ),
            'state_lock': self._state_lock
        }
        self.viz_manager = VisualizationManager(self, viz_config)

        # TF listener for coordinate transformations (default 10s buffer)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        scan_qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").get_parameter_value().string_value,
            self.scan_callback,
            scan_qos_profile,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Path,
            self.get_parameter("plan_input_topic").get_parameter_value().string_value,
            self.path_callback,
            10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").get_parameter_value().string_value,
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )

        # Control loop timer: frequency configurable via parameter
        self.control_frequency = (
            self.get_parameter("control_frequency")
            .get_parameter_value().double_value
        )
        if self.control_frequency <= 0:
            raise ValueError(
                f"Invalid control_frequency: {self.control_frequency}. "
                "Must be > 0 Hz"
            )

        time_period = 1.0 / self.control_frequency
        self.get_logger().info(
            f"Control loop frequency: {self.control_frequency} Hz "
            f"({time_period*1000:.1f} ms period)"
        )
        self.create_timer(time_period, self.run, callback_group=self.control_group)

    def _install_core_timing_hooks(self) -> None:
        """Measure NeuPAN core submodules without modifying the core package."""
        self._core_stage_timing = {
            'core_forward_ms': 0.0,
            'pan_ms': 0.0,
            'dune_ms': 0.0,
            'nrmp_ms': 0.0,
        }

        def wrap_forward(module, attribute, timing_key):
            if module is None:
                return
            original = getattr(module, attribute, None)
            if original is None:
                return

            def timed_forward(*args, **kwargs):
                started = time.perf_counter()
                try:
                    return original(*args, **kwargs)
                finally:
                    self._core_stage_timing[timing_key] += (
                        time.perf_counter() - started
                    ) * 1000.0

            setattr(module, attribute, timed_forward)

        wrap_forward(self.neupan_planner, 'forward', 'core_forward_ms')
        pan = getattr(self.neupan_planner, 'pan', None)
        if pan is not None:
            original_stop_criteria = getattr(pan, 'stop_criteria', None)
            original_pan_forward = getattr(pan, 'forward', None)
            if original_stop_criteria is not None and original_pan_forward is not None:
                def counted_stop_criteria(*args, **kwargs):
                    self._solver_metrics['pan_iterations'] += 1.0
                    result = original_stop_criteria(*args, **kwargs)
                    if result:
                        self._solver_metrics['pan_early_converged'] = 1.0
                    return result

                def measured_pan_forward(*args, **kwargs):
                    self._solver_metrics['pan_iterations'] = 0.0
                    self._solver_metrics['pan_early_converged'] = 0.0
                    pan.stop_criteria = counted_stop_criteria
                    return original_pan_forward(*args, **kwargs)

                pan.forward = measured_pan_forward

            nrmp = getattr(pan, 'nrmp_layer', None)
            solver_layer = getattr(nrmp, 'nrmp_layer', None) if nrmp is not None else None
            solver_forward = getattr(solver_layer, 'forward', None)
            if solver_layer is not None and solver_forward is not None:
                if n1_13_online_diagnostics is not None and self._n113_output_dir:
                    # Identify the live layer and its owner without retaining
                    # planner tensors in the diagnostics module.
                    solver_layer._n113_nrmp_owner = nrmp
                    diagnostic_problem = nrmp.construct_prob()
                    self._n113_status = n1_13_online_diagnostics.install(
                        self._n113_output_dir, solver_layer, diagnostic_problem,
                        self._n113_shadow_enabled,
                    )
                    atexit.register(n1_13_online_diagnostics.finalize)

                def measured_solver_forward(*args, **kwargs):
                    started = time.perf_counter()
                    cpu_started = time.process_time()
                    self._solver_metrics['ecos_solve_calls'] += 1.0
                    n113_token = None
                    if n1_13_online_diagnostics is not None and self._n113_output_dir:
                        n113_token = n1_13_online_diagnostics.primary_start(
                            solver_layer, args, kwargs.get('solver_args')
                        )
                    if n1_11_diagnostics is not None:
                        n1_11_diagnostics.layer_forward_start(
                            solver_layer,
                            args,
                            kwargs.get('solver_args'),
                        )
                    event = {
                        'solver': str(getattr(solver_layer, 'solver', 'ECOS')),
                        'status': 'return-success',
                    }
                    try:
                        if self._n114_client is not None:
                            result, n114_record = self._n114_client.solve(
                                tuple(args), kwargs.get('solver_args')
                            )
                            n114_timing = n114_record['timings']
                            event['n114'] = n114_record
                            for key in (
                                'parameter_count', 'parameter_elements', 'payload_bytes',
                                'parameter_extraction_ms', 'request_serialization_ms',
                                'request_send_ms', 'worker_queue_wait_ms',
                                'worker_decode_ms', 'worker_solve_wall_ms',
                                'worker_solve_cpu_ms', 'worker_result_serialization_ms',
                                'response_return_ms', 'main_reconstruction_ms',
                                'ipc_roundtrip_ms', 'main_wait_ms', 'full_call_ms',
                            ):
                                self._solver_metrics[f'n114_{key}'] = float(
                                    n114_timing.get(key, 0.0)
                                )
                            self._solver_metrics['n114_worker_sequence'] = float(
                                n114_record.get('solver_worker_sequence', -1)
                            )
                            self._solver_metrics['n114_status'] = 1.0
                        else:
                            result = solver_forward(*args, **kwargs)
                    except Exception as exc:
                        self._solver_metrics['solver_failures'] += 1.0
                        event['status'] = 'exception'
                        event['exception'] = f'{type(exc).__name__}: {exc}'
                        if n113_token is not None:
                            n1_13_online_diagnostics.primary_end(n113_token, None, exc)
                        raise
                    finally:
                        if n1_11_diagnostics is not None:
                            n1_11_diagnostics.layer_forward_end()
                        wall_ms = (time.perf_counter() - started) * 1000.0
                        cpu_ms = (time.process_time() - cpu_started) * 1000.0
                        self._solver_metrics['ecos_solve_ms'] += wall_ms
                        self._solver_metrics['ecos_solve_cpu_ms'] += cpu_ms
                        event.update({
                            'wall_ms': wall_ms,
                            'cpu_ms': cpu_ms,
                            'status_detail': 'not exposed by cvxpylayers',
                            'iterations': 'not exposed',
                            'objective': 'not exposed',
                            'primal_residual': 'not exposed',
                            'dual_residual': 'not exposed',
                            'gap': 'not exposed',
                            'setup_ms': 'not exposed',
                        })
                        self._solver_events.append(event)
                    if n113_token is not None:
                        n1_13_online_diagnostics.primary_end(n113_token, result, None)
                        try:
                            # Shadow work is strictly after the primary
                            # returned and is never allowed to affect it.
                            n1_13_online_diagnostics.after_primary(
                                n113_token, solver_forward, result
                            )
                        except Exception as exc:
                            self.get_logger().error(
                                f'N1.13 diagnostic failure: {type(exc).__name__}: {exc}'
                            )
                    return result

                solver_layer.forward = measured_solver_forward

        wrap_forward(pan, 'forward', 'pan_ms')
        if pan is not None:
            wrap_forward(getattr(pan, 'dune_layer', None), 'forward', 'dune_ms')
            wrap_forward(getattr(pan, 'nrmp_layer', None), 'forward', 'nrmp_ms')

    def _get_robot_transform(self, timing: Optional[Dict[str, Any]] = None) -> bool:
        """Get robot transform from TF and update robot_state.

        Returns:
            bool: True if transform successfully obtained, False otherwise

        """
        try:
            # TF query is thread-safe, no lock needed
            lookup_started = time.perf_counter()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            lookup_finished = time.perf_counter()

            if timing is not None:
                timing['tf_lookup_ms'] = (
                    lookup_finished - lookup_started
                ) * 1000.0
                timing['tf_stamp_sec'] = (
                    float(trans.header.stamp.sec)
                    + float(trans.header.stamp.nanosec) * 1e-9
                )

            assembly_started = time.perf_counter()
            yaw = quat_to_yaw(trans.transform.rotation)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            new_state = np.array([x, y, yaw]).reshape(3, 1)

            # Lock only for writing shared state
            with self._state_lock:
                self.robot_state = new_state
            if timing is not None:
                timing['robot_state_assembly_ms'] = (
                    time.perf_counter() - assembly_started
                ) * 1000.0

            self.get_logger().info(
                f"Robot state initialized - x: {new_state[0,0]:.2f}m, "
                f"y: {new_state[1,0]:.2f}m, yaw: {new_state[2,0]:.2f}rad",
                once=True
            )
            return True

        except tf2_ros.LookupException:
            self.get_logger().debug(
                f"Waiting for transform from {self.base_frame} to {self.map_frame}",
                throttle_duration_sec=1.0,
            )
            return False
        except tf2_ros.ConnectivityException:
            self.get_logger().warn(
                "ConnectivityException: Transform not available, waiting for connection",
                throttle_duration_sec=1.0
            )
            return False
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                f"TF extrapolation error: {e}. Check TF timestamps and buffer size.",
                throttle_duration_sec=1.0
            )
            return False

    def _validate_planning_prerequisites(self) -> bool:
        """Validate all prerequisites for planning are met.

        Returns:
            bool: True if all prerequisites are met, False otherwise

        """
        with self._state_lock:
            if self.robot_state is None:
                self.get_logger().debug("Waiting for robot state", throttle_duration_sec=1.0)
                return False

            # Initialize path from waypoints on first run if no path received
            if (len(self.neupan_planner.waypoints) >= 1
                    and self.neupan_planner.initial_path is None):
                self.neupan_planner.set_initial_path_from_state(
                    self.robot_state
                )
                self.get_logger().info(
                    f'Initialized path with '
                    f'{len(self.neupan_planner.waypoints)} waypoints'
                )

            if self.neupan_planner.initial_path is None:
                self.get_logger().debug("Waiting for initial path", throttle_duration_sec=1.0)
                return False

        return True

    def _execute_planning(
            self, timing: Dict[str, Any]
    ) -> Tuple[Optional[npt.NDArray], Dict[str, Any], Dict[str, float], Optional[Dict[str, Any]]]:
        """Execute planning and update state.

        Returns:
            tuple: (action, info, timing) from neupan planner, or (None, None, None) on failure

        """
        # Publish reference path (generate message needs to read planner state)
        path_message_started = time.perf_counter()
        with self._state_lock:
            initial_path = self.neupan_planner.initial_path

        # Publishing is thread-safe, do outside lock
        self.ref_path_pub.publish(self.generate_path_msg(initial_path))
        timing['path_message_publish_ms'] = (
            time.perf_counter() - path_message_started
        ) * 1000.0

        # Step 1: Fast data copy inside lock to minimize lock holding time
        state_copy_started = time.perf_counter()
        with self._state_lock:
            # Copy state data for planning
            # (allows other threads to access shared state)
            robot_state_copy = (
                self.robot_state.copy()
                if self.robot_state is not None else None
            )
            obstacle_points_copy = (
                self.obstacle_points.copy()
                if self.obstacle_points is not None else None
            )

            # Check for obstacles
            has_obstacles = obstacle_points_copy is not None
            scan_counter = self._scan_counter
            scan_stamp_sec = self._latest_scan_stamp_sec
            scan_receive_monotonic = self._latest_scan_receive_monotonic
            path_version = self._path_version
            path_callback_ms = self._last_path_callback_ms
            scan_metrics = dict(self._latest_scan_metrics)
            raw_scan_ranges = (
                self._latest_scan_ranges.copy()
                if self._latest_scan_ranges is not None else None
            )
            raw_scan_meta = (
                self._latest_scan_angle_min,
                self._latest_scan_angle_max,
                self._latest_scan_range_min,
                self._latest_scan_range_max,
            )
            scan_angle_increment = self._latest_scan_angle_increment
            scan_frame_id = self._latest_scan_frame_id
            planner_velocity = _array_value(
                getattr(self.neupan_planner, 'cur_vel_array', None)
            ).copy()
            goal_copy = (
                self.goal.copy() if self.goal is not None else np.empty((0, 0))
            )
            initial_path_matrix = np.hstack([
                _array_value(point)[:3, :] for point in initial_path
            ]) if initial_path else np.empty((3, 0))
            initial_path_matrix = np.vstack([
                initial_path_matrix,
                np.ones((1, initial_path_matrix.shape[1])),
            ]) if initial_path_matrix.size else np.empty((4, 0))
            pan = getattr(self.neupan_planner, 'pan', None)
            current_nom_values = getattr(pan, 'current_nom_values', None)
            pan_state_before = {}
            if current_nom_values and current_nom_values[0] is not None:
                pan_state_before['pan_current_nom_s'] = _array_value(
                    current_nom_values[0]
                ).copy()
                pan_state_before['pan_current_nom_u'] = _array_value(
                    current_nom_values[1]
                ).copy()
                pan_state_before['pan_current_nom_mu'] = np.concatenate([
                    _array_value(value) for value in current_nom_values[2]
                ], axis=0)
                pan_state_before['pan_current_nom_lam'] = np.concatenate([
                    _array_value(value) for value in current_nom_values[3]
                ], axis=0)
            capture_file = str(self.get_parameter('input_capture_file').value)
            should_capture = bool(capture_file and not self._input_capture_done)
            if should_capture:
                self._input_capture_done = True
        if should_capture and robot_state_copy is not None and initial_path is not None:
            capture_path = os.path.abspath(capture_file)
            os.makedirs(os.path.dirname(capture_path), exist_ok=True)
            np.savez_compressed(
                capture_path,
                robot_state=robot_state_copy,
                obstacle_points=(
                    obstacle_points_copy
                    if obstacle_points_copy is not None else np.empty((2, 0))
                ),
                initial_path=initial_path_matrix,
                scan_ranges=(
                    raw_scan_ranges
                    if raw_scan_ranges is not None else np.empty((0,))
                ),
                scan_angle_min=raw_scan_meta[0],
                scan_angle_max=raw_scan_meta[1],
                scan_range_min=raw_scan_meta[2],
                scan_range_max=raw_scan_meta[3],
                scan_angle_increment=scan_angle_increment,
                scan_counter=scan_counter,
                scan_stamp_sec=scan_stamp_sec,
            )
            self.get_logger().info(f'Captured N1 input snapshot: {capture_path}')
        timing['state_copy_ms'] = (time.perf_counter() - state_copy_started) * 1000.0
        timing['scan_counter'] = float(scan_counter)
        timing['scan_stamp_sec'] = float(scan_stamp_sec)
        timing['path_version'] = float(path_version)
        timing['path_callback_ms'] = float(path_callback_ms)
        timing.update(scan_metrics)
        timing['obstacle_points_count'] = float(
            obstacle_points_copy.shape[1] if obstacle_points_copy is not None else 0
        )
        previous_scan_counter = self._last_timing_scan_counter
        if previous_scan_counter is None:
            timing['scan_delta_since_previous'] = 1.0
            timing['dropped_obsolete_scans'] = 0.0
        else:
            scan_delta = max(0, scan_counter - previous_scan_counter)
            timing['scan_delta_since_previous'] = float(scan_delta)
            timing['dropped_obsolete_scans'] = float(max(0, scan_delta - 1))
        self._last_timing_scan_counter = scan_counter

        # Step 2: Execute planning OUTSIDE lock (10-50ms)
        # (allows other threads to access shared state)
        self._core_stage_timing = {
            'core_forward_ms': 0.0,
            'pan_ms': 0.0,
            'dune_ms': 0.0,
            'nrmp_ms': 0.0,
        }
        self._solver_metrics = {
            'pan_iterations': 0.0,
            'pan_early_converged': 0.0,
            'ecos_solve_cpu_ms': 0.0,
            'ecos_solve_ms': 0.0,
            'ecos_solve_calls': 0.0,
            'solver_failures': 0.0,
            'n114_status': 0.0,
        }
        self._solver_events = []
        planner_start_monotonic = time.monotonic()
        planner_cpu_started = time.process_time()
        if n1_13_online_diagnostics is not None and self._n113_output_dir:
            n1_13_online_diagnostics.begin_cycle(
                int(timing.get('cycle_id', 0.0)),
                int(timing.get('scan_counter', -1.0)),
                float(timing.get('scan_stamp_sec', -1.0)),
            )
        if self._n114_client is not None:
            self._n114_client.begin_cycle(
                int(timing.get('cycle_id', 0.0)),
                int(timing.get('scan_counter', -1.0)),
                float(timing.get('scan_stamp_sec', -1.0)),
            )
        if n1_11_diagnostics is not None and self._n111_snapshot_dir:
            n1_11_diagnostics.begin_cycle(int(timing.get('cycle_id', 0.0)))
        try:
            action, info = self.neupan_planner(
                robot_state_copy, obstacle_points_copy
            )
        finally:
            if n1_11_diagnostics is not None and self._n111_snapshot_dir:
                self._n111_cycle_record = n1_11_diagnostics.end_cycle()
            else:
                self._n111_cycle_record = None
        planner_end_monotonic = time.monotonic()
        timing['core_forward_cpu_ms'] = (
            time.process_time() - planner_cpu_started
        ) * 1000.0
        timing.update(self._core_stage_timing)
        timing.update(self._solver_metrics)
        pan = getattr(self.neupan_planner, 'pan', None)
        dune = getattr(pan, 'dune_layer', None) if pan is not None else None
        nrmp = getattr(pan, 'nrmp_layer', None) if pan is not None else None
        dune_points = getattr(dune, 'obstacle_points', None) if dune is not None else None
        nrmp_points = getattr(nrmp, 'obstacle_points', None) if nrmp is not None else None
        timing['dune_effective_points_count'] = float(
            dune_points.shape[1] if dune_points is not None else 0
        )
        timing['nrmp_effective_points_count'] = float(
            nrmp_points.shape[1] if nrmp_points is not None else 0
        )
        timing['core_other_ms'] = max(
            0.0,
            timing['core_forward_ms']
            - timing['dune_ms']
            - timing['nrmp_ms'],
        )
        if n1_13_online_diagnostics is not None and self._n113_output_dir:
            n1_13_online_diagnostics.annotate_cycle(
                int(timing.get('cycle_id', 0.0)), timing
            )
        input_snapshot = None
        if self._n110_snapshot_dir or self._n111_snapshot_dir:
            input_snapshot = {
            'robot_state': robot_state_copy.copy(),
            'robot_velocity': planner_velocity,
            'obstacle_points': (
                obstacle_points_copy.copy()
                if obstacle_points_copy is not None else np.empty((2, 0))
            ),
            'scan_ranges': (
                raw_scan_ranges.copy()
                if raw_scan_ranges is not None else np.empty((0,))
            ),
            'initial_path': initial_path_matrix.copy(),
            'goal': goal_copy,
            'dune_input_points': _array_value(dune_points, (2, 0)).copy(),
            'nrmp_input_points': _array_value(nrmp_points, (2, 0)).copy(),
            'reference_state': _array_value(info.get('ref_state_tensor')).copy(),
            'reference_speed': _array_value(info.get('ref_speed_tensor')).copy(),
            'optimized_state': _array_value(info.get('state_tensor')).copy(),
            'optimized_velocity': _array_value(info.get('vel_tensor')).copy(),
            'distance': _array_value(info.get('distance_tensor')).copy(),
            'action': _array_value(action).copy(),
            'scan_angle_min': raw_scan_meta[0],
            'scan_angle_max': raw_scan_meta[1],
            'scan_angle_increment': scan_angle_increment,
            'scan_range_min': raw_scan_meta[2],
            'scan_range_max': raw_scan_meta[3],
            'scan_frame_id': scan_frame_id,
            'scan_counter': scan_counter,
            'scan_stamp_sec': scan_stamp_sec,
            'path_version': path_version,
            'tf_stamp_sec': timing.get('tf_stamp_sec', -1.0),
            'solver_events': list(self._solver_events),
        }
            input_snapshot.update(pan_state_before)

        # Step 3: Write back results inside lock (< 0.1 μs)
        with self._state_lock:
            self.stop = info["stop"]
            self.arrive = info["arrive"]

        # Logging outside lock
        if not has_obstacles:
            self.get_logger().info(
                "No obstacle points detected, performing path tracking only",
                throttle_duration_sec=1.0,
            )

        # Log arrival
        if info["arrive"]:
            self.get_logger().info("Arrived at target", once=True)

        # Log stop condition
        if info["stop"]:
            # Read min_distance and threshold outside lock
            # (assume read-only access is safe)
            self.get_logger().warn(
                f"Collision risk detected - "
                f"min distance: {self.neupan_planner.min_distance:.2f}m, "
                f"threshold: {self.neupan_planner.collision_threshold:.2f}m",
                throttle_duration_sec=1.0,
            )

        return action, info, {
            "scan_counter": float(scan_counter),
            "scan_stamp_sec": float(scan_stamp_sec),
            "scan_receive_monotonic": float(scan_receive_monotonic),
            "planner_start_monotonic": float(planner_start_monotonic),
            "planner_end_monotonic": float(planner_end_monotonic),
            "core_forward_cpu_ms": timing.get('core_forward_cpu_ms', 0.0),
        }, input_snapshot

    def _write_n110_snapshot_file(self, record: Dict[str, Any]) -> None:
        directory = os.path.abspath(self._n110_snapshot_dir)
        path = os.path.join(directory, f"{record['snapshot_id']}.npz")
        metadata = record['metadata']
        payload = {
            key: value for key, value in record['arrays'].items()
            if isinstance(value, np.ndarray)
        }
        payload['metadata_json'] = np.asarray(
            json.dumps(metadata, sort_keys=True, allow_nan=False)
        )
        try:
            np.savez_compressed(path, **payload)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'N1.10 snapshot write failed: {exc}')

    def _write_n110_manifest(self) -> None:
        if not self._n110_snapshot_dir:
            return
        records = list(self._n110_records)
        normal_max = float(self.get_parameter('n110_normal_max_ms').value)
        medium_min = float(self.get_parameter('n110_medium_min_ms').value)
        medium_max = float(self.get_parameter('n110_medium_max_ms').value)
        spike_min = float(self.get_parameter('n110_spike_min_ms').value)

        def category_rows(name):
            rows = []
            for record in records:
                value = record['nrmp_ms']
                if name == 'normal' and value < normal_max:
                    rows.append(record)
                elif name == 'medium' and medium_min <= value <= medium_max:
                    rows.append(record)
                elif name == 'spike' and value > spike_min:
                    rows.append(record)
            return rows

        def in_category(name, record):
            value = record['nrmp_ms']
            if name == 'normal':
                return value < normal_max
            if name == 'medium':
                return medium_min <= value <= medium_max
            return value > spike_min

        selected = {}
        selected_ids = set()
        replayable_records = [
            record for record in records
            if 'pan_current_nom_s' in record['arrays']
        ]
        for name in ('normal', 'medium', 'spike'):
            rows = [
                row for row in replayable_records
                if in_category(name, row)
            ][:3]
            selected[name] = [row['snapshot_id'] for row in rows]
            selected_ids.update(row['snapshot_id'] for row in rows)
        slowest = sorted(
            replayable_records, key=lambda row: row['nrmp_ms'], reverse=True
        )[:5]
        selected['slowest_5_online_nrmp'] = [row['snapshot_id'] for row in slowest]
        selected_ids.update(row['snapshot_id'] for row in slowest)

        for record in records:
            if record['snapshot_id'] not in selected_ids:
                continue
            if record['snapshot_id'] in self._n110_saved_ids:
                continue
            self._n110_saved_ids.add(record['snapshot_id'])
            writer = threading.Thread(
                target=self._write_n110_snapshot_file,
                args=(record,),
                daemon=True,
            )
            self._n110_writer_threads.append(writer)
            writer.start()

        manifest = {
            'schema': 'r1_n1_10_snapshot_manifest_v1',
            'classification': {
                'normal': f'nrmp_ms < {normal_max}',
                'medium': f'{medium_min} <= nrmp_ms <= {medium_max}',
                'spike': f'nrmp_ms > {spike_min}',
            },
            'counts': {
                'planner_cycles': len(records),
                'normal': len(category_rows('normal')),
                'medium': len(category_rows('medium')),
                'spike': len(category_rows('spike')),
            },
            'selected': selected,
            'cycle_summary': [
                {
                    'snapshot_id': row['snapshot_id'],
                    'cycle_id': row['metadata']['cycle_id'],
                    'scan_counter': row['metadata']['scan_counter'],
                    'nrmp_ms': row['nrmp_ms'],
                    'dune_ms': row['metadata']['timing'].get('dune_ms'),
                    'core_forward_ms': row['metadata']['timing'].get('core_forward_ms'),
                    'core_forward_cpu_ms': row['metadata']['timing'].get('core_forward_cpu_ms'),
                    'ecos_solve_cpu_ms': row['metadata']['timing'].get('ecos_solve_cpu_ms'),
                    'ecos_solve_calls': row['metadata']['timing'].get('ecos_solve_calls'),
                    'pan_iterations': row['metadata']['timing'].get('pan_iterations'),
                }
                for row in records
            ],
        }
        path = os.path.join(os.path.abspath(self._n110_snapshot_dir), 'manifest.json')
        try:
            with self._n110_manifest_lock:
                with open(path, 'w', encoding='utf-8') as stream:
                    json.dump(manifest, stream, indent=2, sort_keys=True)
            self._n110_manifest_dirty = False
        except OSError as exc:
            self.get_logger().error(f'N1.10 manifest write failed: {exc}')

    def _write_n110_final(self) -> None:
        if not self._n110_snapshot_dir:
            return
        self._write_n110_manifest()
        for writer in self._n110_writer_threads:
            writer.join(timeout=10.0)

    def _capture_n110_cycle(
            self, input_snapshot: Optional[Dict[str, Any]], info: Dict[str, Any],
            timing: Dict[str, Any], output_publish_monotonic: float
    ) -> None:
        if input_snapshot is None:
            return
        cycle_id = int(timing.get('cycle_id', 0.0))
        pan = getattr(self.neupan_planner, 'pan', None)
        nrmp_layer = getattr(pan, 'nrmp_layer', None) if pan is not None else None
        solver = getattr(nrmp_layer, 'solver', 'ECOS') if nrmp_layer is not None else 'ECOS'
        metadata = {
            'schema': 'r1_n1_10_replay_snapshot_v1',
            'cycle_id': cycle_id,
            'scan_counter': int(input_snapshot['scan_counter']),
            'scan_stamp_sec': input_snapshot['scan_stamp_sec'],
            'scan_frame': input_snapshot['scan_frame_id'],
            'online_class': 'unclassified',
            'timing': {
                key: float(value) for key, value in timing.items()
                if isinstance(value, (int, float, np.integer, np.floating))
            },
            'output_publish_monotonic': output_publish_monotonic,
            'solver': {
                'name': str(solver),
                'status': 'not exposed by cvxpylayers',
                'iteration_count': 'not exposed',
                'objective': 'not exposed',
                'residuals': 'not exposed',
                'events': input_snapshot['solver_events'],
            },
            'planner': {
                'config_path': self.planner_config_file,
                'config_sha256': _file_sha256(self.planner_config_file),
                'checkpoint_path': self.dune_checkpoint,
                'checkpoint_sha256': _file_sha256(self.dune_checkpoint),
                'receding': int(self.neupan_planner.T),
                'iter_num': int(pan.iter_num) if pan is not None else None,
                'iter_threshold': float(pan.iter_threshold) if pan is not None else None,
                'dune_max_num': int(pan.dune_max_num) if pan is not None else None,
                'nrmp_max_num': int(pan.nrmp_max_num) if pan is not None else None,
                'dtype': str(torch.get_default_dtype()),
                'device': str(torch.device('cuda' if torch.cuda.is_available() else 'cpu')),
                'torch_num_threads': int(torch.get_num_threads()),
                'torch_num_interop_threads': int(torch.get_num_interop_threads()),
                'random_seed': int(torch.initial_seed()),
                'numpy_random_state_head': int(np.random.get_state()[1][0]),
            },
            'input': {
                'scan': {
                    'angle_min': input_snapshot['scan_angle_min'],
                    'angle_max': input_snapshot['scan_angle_max'],
                    'angle_increment': input_snapshot['scan_angle_increment'],
                    'range_min': input_snapshot['scan_range_min'],
                    'range_max': input_snapshot['scan_range_max'],
                },
                'obstacle_points_count': int(input_snapshot['obstacle_points'].shape[1]),
                'dune_points_count': int(input_snapshot['dune_input_points'].shape[1]),
                'nrmp_points_count': int(input_snapshot['nrmp_input_points'].shape[1]),
                'path_version': input_snapshot['path_version'],
                'tf_stamp_sec': input_snapshot['tf_stamp_sec'],
            },
            'runtime': _runtime_snapshot(),
            'replay_fields': sorted(
                key for key, value in input_snapshot.items()
                if isinstance(value, np.ndarray)
            ),
        }
        nrmp_ms = float(timing.get('nrmp_ms', 0.0))
        if nrmp_ms < float(self.get_parameter('n110_normal_max_ms').value):
            metadata['online_class'] = 'normal'
        elif nrmp_ms <= float(self.get_parameter('n110_medium_max_ms').value) and nrmp_ms >= float(self.get_parameter('n110_medium_min_ms').value):
            metadata['online_class'] = 'medium'
        elif nrmp_ms > float(self.get_parameter('n110_spike_min_ms').value):
            metadata['online_class'] = 'spike'
        arrays = {
            key: value for key, value in input_snapshot.items()
            if isinstance(value, np.ndarray)
        }
        record = {
            'snapshot_id': f'cycle_{cycle_id:06d}',
            'nrmp_ms': float(timing.get('nrmp_ms', 0.0)),
            'metadata': metadata,
            'arrays': arrays,
        }
        self._n110_records.append(record)
        self._n110_manifest_dirty = True
        self._write_n110_manifest()

    def _publish_planning_results(
            self, action: Optional[npt.NDArray], info: Dict[str, Any],
            timing: Dict[str, float],
            input_snapshot: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Publish planning results and visualization markers.

        Args:
            action: Control action from planner
            info: Planning info dictionary

        """
        # Publish path messages (info is local, thread-safe)
        trajectory_started = time.perf_counter()
        self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
        self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
        timing['trajectory_publish_ms'] = (
            time.perf_counter() - trajectory_started
        ) * 1000.0

        # Generate twist message using info dict (avoid reading shared state)
        cmd_started = time.perf_counter()
        vel_msg = self.generate_twist_msg(action, info["stop"], info["arrive"])
        self.vel_pub.publish(vel_msg)
        timing['cmd_publish_ms'] = (time.perf_counter() - cmd_started) * 1000.0

        # Visualization (delegated to visualization manager)
        visualization_started = time.perf_counter()
        self.viz_manager.publish_visualization(
            self.neupan_planner, self.robot_state
        )
        timing['visualization_ms'] = (
            time.perf_counter() - visualization_started
        ) * 1000.0

        output_publish_monotonic = time.monotonic()
        timing_msg = Float64MultiArray()
        timing_msg.data = [
            2.0,
            timing["cycle_id"],
            timing["scan_counter"],
            timing["scan_stamp_sec"],
            timing["scan_receive_monotonic"],
            timing["path_version"],
            timing.get("tf_stamp_sec", -1.0),
            timing["cycle_start_monotonic"],
            timing["planner_start_monotonic"],
            timing["planner_end_monotonic"],
            output_publish_monotonic,
            timing.get("scan_processing_ms", 0.0),
            timing.get("scan_conversion_ms", 0.0),
            timing.get("scan_filter_ms", 0.0),
            timing.get("scan_tf_lookup_ms", 0.0),
            timing.get("scan_transform_ms", 0.0),
            timing.get("tf_lookup_ms", 0.0),
            timing.get("robot_state_assembly_ms", 0.0),
            timing.get("path_callback_ms", 0.0),
            0.0,  # path transform: path and planner frame are both odom in N1
            timing.get("state_copy_ms", 0.0),
            timing.get("path_message_publish_ms", 0.0),
            timing.get("core_forward_ms", 0.0),
            timing.get("pan_ms", 0.0),
            timing.get("dune_ms", 0.0),
            timing.get("nrmp_ms", 0.0),
            timing.get("core_other_ms", 0.0),
            timing.get("trajectory_publish_ms", 0.0),
            timing.get("cmd_publish_ms", 0.0),
            timing.get("visualization_ms", 0.0),
            timing.get("obstacle_points_count", 0.0),
            timing.get("scan_delta_since_previous", 0.0),
            timing.get("dropped_obsolete_scans", 0.0),
            timing.get("dune_effective_points_count", 0.0),
            timing.get("nrmp_effective_points_count", 0.0),
            timing.get("pan_iterations", 0.0),
            timing.get("pan_early_converged", 0.0),
            timing.get("ecos_solve_ms", 0.0),
            timing.get("ecos_solve_cpu_ms", 0.0),
            timing.get("ecos_solve_calls", 0.0),
            timing.get("solver_failures", 0.0),
            timing.get("core_forward_cpu_ms", 0.0),
            timing.get("n114_parameter_count", 0.0),
            timing.get("n114_parameter_elements", 0.0),
            timing.get("n114_payload_bytes", 0.0),
            timing.get("n114_parameter_extraction_ms", 0.0),
            timing.get("n114_request_serialization_ms", 0.0),
            timing.get("n114_request_send_ms", 0.0),
            timing.get("n114_worker_queue_wait_ms", 0.0),
            timing.get("n114_worker_decode_ms", 0.0),
            timing.get("n114_worker_solve_wall_ms", 0.0),
            timing.get("n114_worker_solve_cpu_ms", 0.0),
            timing.get("n114_worker_result_serialization_ms", 0.0),
            timing.get("n114_response_return_ms", 0.0),
            timing.get("n114_main_reconstruction_ms", 0.0),
            timing.get("n114_ipc_roundtrip_ms", 0.0),
            timing.get("n114_main_wait_ms", 0.0),
            timing.get("n114_full_call_ms", 0.0),
            timing.get("n114_worker_sequence", -1.0),
            timing.get("n114_status", 0.0),
        ]
        timing_started = time.perf_counter()
        self.timing_pub.publish(timing_msg)
        timing['timing_publish_ms'] = (time.perf_counter() - timing_started) * 1000.0
        self._capture_n110_cycle(
            input_snapshot, info, timing, output_publish_monotonic
        )
        self._capture_n111_cycle(input_snapshot, timing)

    def _capture_n111_cycle(
            self, input_snapshot: Optional[Dict[str, Any]], timing: Dict[str, Any]
    ) -> None:
        """Persist canonical-level evidence after the planner call returns."""
        if not self._n111_snapshot_dir or input_snapshot is None:
            return
        if self._n111_cycle_record is None or n1_11_diagnostics is None:
            return
        cycle_id = int(timing.get('cycle_id', 0.0))
        diagnostic = n1_11_diagnostics.write_cycle(
            self._n111_cycle_record, self._n111_snapshot_dir
        )
        if diagnostic is None:
            return
        directory = os.path.abspath(self._n111_snapshot_dir)
        high_level_path = os.path.join(directory, f'cycle_{cycle_id:06d}_high_level.npz')
        metadata = {
            'schema': 'r1_n1_11_high_level_cycle_v1',
            'cycle_id': cycle_id,
            'timing': {
                key: float(value) for key, value in timing.items()
                if isinstance(value, (int, float, np.integer, np.floating))
            },
            'canonical_cycle': diagnostic,
            'n111_hook_status': self._n111_hook_status,
            'replay_fields': sorted(
                key for key, value in input_snapshot.items()
                if isinstance(value, np.ndarray)
            ),
        }
        payload = {
            key: value for key, value in input_snapshot.items()
            if isinstance(value, np.ndarray)
        }
        payload['metadata_json'] = np.asarray(
            json.dumps(metadata, sort_keys=True, allow_nan=False)
        )
        try:
            np.savez_compressed(high_level_path, **payload)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'N1.11 high-level snapshot write failed: {exc}')

    def run(self) -> None:
        """Execute main control loop at fixed frequency.

        Note: Fine-grained locking is handled within each helper method.
        """
        cycle_start_monotonic = time.monotonic()
        timing = {'cycle_start_monotonic': cycle_start_monotonic}

        # Step 1: Get robot transform (locks internally for robot_state write)
        if not self._get_robot_transform(timing):
            return

        # Step 2: Validate planning prerequisites (locks internally for state read)
        if not self._validate_planning_prerequisites():
            return

        with self._state_lock:
            current_scan_counter = self._scan_counter
            if self.latest_scan_only:
                if current_scan_counter <= self._last_processed_scan_counter:
                    self._skipped_no_fresh_scan_cycles += 1
                    return
                self._last_processed_scan_counter = current_scan_counter
            self._cycle_counter += 1
            timing['cycle_id'] = float(self._cycle_counter)

        # Step 3: Execute planning (locks internally for planning execution)
        action, info, planning_timing, input_snapshot = self._execute_planning(timing)
        timing.update(planning_timing)
        timing['scan_counter'] = float(timing.get('scan_counter', current_scan_counter))
        if self._n114_client is not None:
            self._n114_client.annotate_cycle(
                int(timing.get('cycle_id', 0.0)), timing
            )

        # Step 4: Publish results (locks internally for marker generation)
        self._publish_planning_results(action, info, timing, input_snapshot)

    def scan_callback(self, scan_msg: LaserScan) -> Optional[npt.NDArray]:
        """Process laser scan data and update obstacle points in map frame.

        Args:
            scan_msg: LaserScan message from sensor

        Returns:
            Transformed obstacle points or None if processing failed

        """
        processing_started = time.perf_counter()
        receive_monotonic = time.monotonic()
        stamp_sec = float(scan_msg.header.stamp.sec) + (
            float(scan_msg.header.stamp.nanosec) * 1e-9
        )
        with self._state_lock:
            self._scan_counter += 1
            self._latest_scan_stamp_sec = stamp_sec
            self._latest_scan_receive_monotonic = receive_monotonic
            self._latest_scan_ranges = np.asarray(scan_msg.ranges, dtype=float)
            self._latest_scan_frame_id = str(scan_msg.header.frame_id)
            self._latest_scan_angle_min = float(scan_msg.angle_min)
            self._latest_scan_angle_max = float(scan_msg.angle_max)
            self._latest_scan_angle_increment = float(scan_msg.angle_increment)
            self._latest_scan_range_min = float(scan_msg.range_min)
            self._latest_scan_range_max = float(scan_msg.range_max)

        # Quick check if robot state is available (lock briefly)
        with self._state_lock:
            if self.robot_state is None:
                return None

        conversion_started = time.perf_counter()
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        if self.flip_angle:
            angles = np.flip(angles)
        conversion_ms = (time.perf_counter() - conversion_started) * 1000.0

        # Vectorized filtering: Apply downsampling, range, and angle constraints
        filter_started = time.perf_counter()
        indices = np.arange(len(ranges))
        downsample_mask = (indices % self.scan_downsample) == 0
        range_mask = (ranges >= self.scan_range[0]) & (ranges <= self.scan_range[1])
        angle_mask = (angles > self.scan_angle_range[0]) & (angles < self.scan_angle_range[1])

        valid_mask = downsample_mask & range_mask & angle_mask
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        filter_ms = (time.perf_counter() - filter_started) * 1000.0

        if len(valid_ranges) == 0:
            # Update obstacle_points with lock
            with self._state_lock:
                self.obstacle_points = None
                self._latest_scan_metrics = {
                    'scan_processing_ms': (time.perf_counter() - processing_started) * 1000.0,
                    'scan_conversion_ms': conversion_ms,
                    'scan_filter_ms': filter_ms,
                    'scan_tf_lookup_ms': 0.0,
                    'scan_transform_ms': 0.0,
                    'obstacle_points_count': 0.0,
                }
            self.get_logger().warn(
                "No valid scan points after filtering",
                throttle_duration_sec=1.0
            )
            return None

        # Vectorized coordinate computation (faster than loop)
        transform_started = time.perf_counter()
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        point_array = np.vstack([x_coords, y_coords])
        scan_transform_ms = (time.perf_counter() - transform_started) * 1000.0

        try:
            scan_tf_started = time.perf_counter()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.lidar_frame, rclpy.time.Time()
            )
            scan_tf_lookup_ms = (time.perf_counter() - scan_tf_started) * 1000.0

            yaw = quat_to_yaw(trans.transform.rotation)
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            transform_started = time.perf_counter()
            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            transformed_points = rot_matrix @ point_array + trans_matrix
            scan_transform_ms += (time.perf_counter() - transform_started) * 1000.0

            # Lock only for writing shared state
            with self._state_lock:
                self.obstacle_points = transformed_points
                self._latest_scan_metrics = {
                    'scan_processing_ms': (time.perf_counter() - processing_started) * 1000.0,
                    'scan_conversion_ms': conversion_ms,
                    'scan_filter_ms': filter_ms,
                    'scan_tf_lookup_ms': scan_tf_lookup_ms,
                    'scan_transform_ms': scan_transform_ms,
                    'obstacle_points_count': float(transformed_points.shape[1]),
                }

            self.get_logger().info(
                f"Laser scan initialized with {transformed_points.shape[1]} "
                "points", once=True
            )
            return transformed_points

        except tf2_ros.LookupException:
            self.get_logger().debug(
                f"Waiting for transform from {self.lidar_frame} to {self.map_frame}",
                throttle_duration_sec=1.0
            )
            return

    def path_callback(self, path: Path) -> None:
        """Update initial path from received path message.

        Args:
            path: Path message containing waypoints

        """
        callback_started = time.perf_counter()
        n_poses = len(path.poses)
        if n_poses == 0:
            return

        self.get_logger().info(f"Received new path with {n_poses} waypoints")

        # Optimized: single-pass extraction with transpose
        if self.include_initial_path_direction:
            # Extract x, y, and orientation in one pass
            data = [
                (p.pose.position.x, p.pose.position.y,
                 quat_to_yaw(p.pose.orientation))
                for p in path.poses
            ]
            xs, ys, thetas = np.array(data).T
        else:
            self.get_logger().debug(
                "Using path gradient for direction "
                "(include_initial_path_direction=False)", once=True
            )

            # Extract x, y in one pass
            coords = [
                (p.pose.position.x, p.pose.position.y) for p in path.poses
            ]
            xs, ys = np.array(coords).T

            # Vectorized gradient computation using np.diff
            dx = np.diff(xs, append=xs[-1])
            dy = np.diff(ys, append=ys[-1])
            thetas = np.arctan2(dy, dx)

            # For the last point, use direction from second-to-last point
            if n_poses > 1:
                thetas[-1] = thetas[-2]

        # Vectorized array construction for better performance
        ones = np.ones(n_poses)
        # Shape: (4, n_poses)
        initial_point_array = np.vstack([xs, ys, thetas, ones])

        # Convert to list of column vectors for planner API compatibility
        initial_point_list = [
            initial_point_array[:, i:i + 1] for i in range(n_poses)
        ]

        with self._state_lock:
            if (self.neupan_planner.initial_path is None
                    or self.refresh_initial_path):
                self.neupan_planner.set_initial_path(initial_point_list)
                self._path_version += 1
                self._last_path_callback_ms = (
                    time.perf_counter() - callback_started
                ) * 1000.0

    def goal_callback(self, goal: PoseStamped) -> None:
        """Update goal and regenerate initial path.

        Args:
            goal: Goal pose message

        """
        # Extract goal from message (no lock needed)
        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = quat_to_yaw(goal.pose.orientation)

        new_goal = np.array([[x], [y], [theta]])

        self.get_logger().info(
            f"New goal set - x: {x:.2f}m, y: {y:.2f}m, "
            f"theta: {theta:.2f}rad"
        )

        # Check if robot state is ready
        if self.robot_state is None:
            self.get_logger().warn(
                "Goal received but robot state not yet available. "
                "Path planning will start once robot state is received."
            )
            self.goal = new_goal
            return

        # Lock only when accessing shared state and modifying planner
        with self._state_lock:
            self.goal = new_goal

            self.get_logger().debug(
                f"Current state: {self.robot_state.tolist()}"
            )
            self.get_logger().debug(f"Target goal: {self.goal.tolist()}")

            self.neupan_planner.update_initial_path_from_goal(
                self.robot_state, self.goal
            )
            self.neupan_planner.reset()

    def generate_path_msg(self, path_list: List[npt.NDArray]) -> Path:
        """Generate ROS Path message from list of poses.

        Args:
            path_list: List of pose arrays (3, 1) or (4, 1)
                       containing [x, y, theta, ...]

        Returns:
            Path message with poses

        """
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        if len(path_list) == 0:
            return path

        # Vectorized approach: normalize all points and stack into matrix
        normalized_points = []
        for point in path_list:
            point_arr = np.array(point)
            if point_arr.ndim == 1:
                point_arr = point_arr.reshape(-1, 1)
            # Extract only first 3 elements (x, y, theta) to ensure consistent dimensions
            point_arr = point_arr[:3, :]
            normalized_points.append(point_arr)

        # Stack all points horizontally -> shape: (3, n_poses)
        points_matrix = np.hstack(normalized_points)

        # Vectorized extraction (single op instead of 3 list comps)
        xs = points_matrix[0, :].tolist()
        ys = points_matrix[1, :].tolist()
        yaws = points_matrix[2, :].tolist()

        # Create path message
        for x, y, yaw in zip(xs, ys, yaws):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = yaw_to_quat(yaw)
            path.poses.append(ps)

        return path

    def generate_twist_msg(
            self, vel: Optional[npt.NDArray], stop: bool, arrive: bool
    ) -> Twist:
        """Generate ROS Twist message from velocity command.

        Args:
            vel: Velocity array (2, 1)
                 containing [linear_speed, angular_speed], or None
            stop: Whether the robot should stop (collision risk)
            arrive: Whether the robot has arrived at goal

        Returns:
            Twist message (zero velocity if stopped/arrived or vel is None)

        """
        if vel is None:
            return Twist()

        speed = float(vel[0, 0])
        steer = float(vel[1, 0])

        if stop or arrive:
            return Twist()
        else:
            action = Twist()
            action.linear.x = speed
            action.angular.z = steer
            return action


def main(args=None):
    """Main entry point for NeuPAN node.

    Args:
        args: Command-line arguments (optional)

    """
    rclpy.init(args=args)

    neupan_node = None
    executor = None
    try:
        neupan_node = NeupanCore()

        # Use MultiThreadedExecutor with 2 threads for concurrent execution
        # Thread 1: run() timer (control loop at configurable frequency)
        # Thread 2: scan/path/goal callbacks (sensor and planning updates)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(neupan_node)

        viz_status = (
            'enabled' if neupan_node.enable_visualization else 'disabled'
        )
        neupan_node.get_logger().info(
            f"NeuPAN node started - "
            f"Control: {neupan_node.control_frequency}Hz, Threads: 2, "
            f"Visualization: {viz_status}"
        )
        executor.spin()

    except KeyboardInterrupt:
        if neupan_node:
            neupan_node.get_logger().info(
                "NeuPAN node shutting down due to "
                "KeyboardInterrupt (Ctrl+C)."
            )
        pass
    except Exception as e:
        if neupan_node:
            neupan_node.get_logger().error(
                f'Unhandled exception: {e}\n{traceback.format_exc()}'
            )
        raise
    finally:
        if executor:
            executor.shutdown()
        if neupan_node:
            neupan_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
