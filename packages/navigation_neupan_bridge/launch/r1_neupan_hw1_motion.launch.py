"""NeuPAN HW1 motion chain for Humble.

The chassis driver remains externally owned. NeuPAN publishes through the
Nav2 controller, velocity smoother, and final scan-based collision guard.
"""

import json
import os
import shlex

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_lidar_calibration(calibration):
    """Reject an HW1 record that disagrees with the robot TF calibration."""
    calibration_path = os.path.join(
        get_package_share_directory("r1_description"),
        "config", "r1_lidar_extrinsics.yaml")
    try:
        with open(calibration_path, encoding="utf-8") as calibration_file:
            canonical = yaml.safe_load(calibration_file)["base_link_to_lidar_link"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError("canonical R1 lidar calibration is unreadable: " + str(error)) from error

    record = calibration["lidar_to_base"]
    fields = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
    mismatches = [
        field for field in fields
        if field not in record or abs(float(record[field]) - float(canonical[field])) > 1e-4
    ]
    if mismatches:
        raise RuntimeError(
            "HW1 lidar calibration differs from r1_description for: " + ", ".join(mismatches))


def _build(context):
    bridge_share = get_package_share_directory("navigation_neupan_bridge")
    integration_share = get_package_share_directory("r1_neupan_integration")
    guard_share = get_package_share_directory("r1_collision_guard")
    python_executable = LaunchConfiguration("python_executable").perform(context)
    input_cloud = LaunchConfiguration("input_cloud").perform(context)
    scan_source = LaunchConfiguration("scan_source").perform(context)
    command_profile = LaunchConfiguration("command_profile").perform(context)
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic").perform(context)
    calibration_record = LaunchConfiguration("calibration_record").perform(context)
    if not os.path.isfile(python_executable):
        raise RuntimeError(
            "NeuPAN Humble venv is unavailable: " + python_executable +
            ". Create it and install the vendored NeuPAN core before HW1."
        )
    if not calibration_record or not os.path.isfile(calibration_record):
        raise RuntimeError(
            "HW1 calibration record is required. Supply calibration_record:=/path/to/r1_hw1_calibration.json"
        )
    try:
        with open(calibration_record, encoding="utf-8") as record_file:
            calibration = json.load(record_file)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("HW1 calibration record is unreadable: " + str(error)) from error
    if calibration.get("schema") != "r1_hw1_calibration_record_v1" or not calibration.get("approved_for_hw1"):
        raise RuntimeError("HW1 calibration record is not approved for HW1")
    if scan_source not in {"livox_only", "dual"}:
        raise RuntimeError("scan_source must be livox_only or dual")
    if command_profile not in {"direct", "smoothed"}:
        raise RuntimeError("command_profile must be direct or smoothed")
    if not cmd_vel_out_topic.startswith("/"):
        raise RuntimeError("cmd_vel_out_topic must be an absolute ROS topic")
    required_sections = {
        "footprint", "drive", "lidar_to_base", "scan", "approval",
    }
    missing_sections = sorted(required_sections.difference(calibration))
    if missing_sections:
        raise RuntimeError(
            "HW1 calibration record is incomplete: " + ", ".join(missing_sections)
        )
    _validate_lidar_calibration(calibration)

    config_dir = os.path.join(bridge_share, "config")
    footprint = calibration["footprint"]
    polygon_xy = footprint.get("polygon_xy")
    if polygon_xy:
        footprint_points = [coordinate for point in polygon_xy for coordinate in point]
    else:
        # The approved HW1 record contains measured length and width. This
        # conservative rectangle is replaced automatically once polygon_xy is
        # surveyed and added to the calibration record.
        half_length = float(footprint["length_m"]) / 2.0
        half_width = float(footprint["width_m"]) / 2.0
        footprint_points = [
            half_length, half_width, half_length, -half_width,
            -half_length, -half_width, -half_length, half_width,
        ]
    # With no arm state stream, keep a measured-reach envelope rather than
    # dropping the arms from collision geometry. This keeps navigation alive
    # while conservatively rejecting spaces the unknown arm pose could occupy.
    arm_data_fallback_points = [
        1.05, 1.30, 1.05, -1.30,
        -1.05, -1.30, -1.05, 1.30,
    ]
    runtime_prefix = shlex.join([
        python_executable,
        "-m",
        "r1_neupan_integration.torch_runtime_bootstrap",
        "--torch-num-threads", "8",
        "--torch-num-interop-threads", "24",
        "--thread-env-policy", "inherit",
        "--runtime-report-file",
        LaunchConfiguration("runtime_report_file").perform(context),
    ])

    livox_scan_topic = "/scan_livox" if scan_source == "dual" else "/scan_neupan"
    # The localization node withholds /deskew_cloud until its initial pose is
    # valid.  A dual-scan collision guard must still receive the front scan
    # during that bootstrap period, so use Livox's live source directly.
    preprocess_input_cloud = "/lidar/pointcloud" if scan_source == "dual" else input_cloud
    preprocess = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="neupan_pointcloud_to_scan",
        output="screen",
        remappings=[("cloud_in", preprocess_input_cloud), ("scan", livox_scan_topic)],
        parameters=[os.path.join(config_dir, "lidar_r1_hw1.yaml"), {"use_sim_time": False}],
    )
    arm_footprint = Node(
        package="navigation_neupan_bridge",
        executable="r1_arm_footprint",
        name="r1_arm_footprint",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "base_frame": "base_link",
            "base_footprint_points": footprint_points,
            "fallback_footprint_points": arm_data_fallback_points,
            "joint_state_timeout_sec": 0.50,
            "update_frequency": 10.0,
            "link_radius_m": 0.12,
            "tool_radius_m": 0.16,
            "projection_margin_m": 0.03,
        }],
    )
    scan_fusion = None
    if scan_source == "dual":
        scan_fusion = Node(
            package="r1_lidar_preprocess",
            executable="scan_fusion_node",
            name="r1_scan_fusion",
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("r1_lidar_preprocess"),
                    "config", "scan_fusion_hw1.yaml"),
                {"use_sim_time": False},
            ],
        )
    worker = Node(
        package="r1_neupan_integration",
        executable="n1_16_node",
        name="n1_16_production_bridge",
        output="screen",
        emulate_tty=True,
        prefix=runtime_prefix,
        parameters=[
            os.path.join(config_dir, "robot_r1_hw1.yaml"),
            {
                "use_sim_time": False,
                "robot_config_dir": config_dir,
                "planner_config_file": "planner_r1_hw1.yaml",
                "dune_checkpoint_file": os.path.join(
                    integration_share, "config", "models", "dune_model_5000.pth"),
                "scan_topic": "/scan_neupan",
                "plan_input_topic": "/neupan_path",
                "goal_topic": "/neupan_goal",
                "map_frame": "map",
                "base_frame": "base_link",
                "lidar_frame": "base_link",
                "self_filter_enabled": True,
                "allow_reverse": True,
                "control_frequency": 10.0,
                "latest_scan_only": True,
                "navigation_mode": "terminal",
                "planner_result_topic": "/neupan_planner_result",
                "planner_context_topic": "/neupan_planner_context",
                "bridge_status_topic": "/neupan_bridge_status",
                "torch_num_threads": 8,
                "torch_num_interop_threads": 24,
                "shadow_tf_fault_injection_allowed": False,
                "shadow_force_robot_tf_failure": False,
            },
        ],
    )
    collision_guard = Node(
        package="r1_collision_guard",
        executable="r1_collision_guard_node",
        name="r1_collision_guard",
        output="screen",
        parameters=[
            os.path.join(guard_share, "config", "r1_collision_guard_hw1.yaml"),
            {
                "use_sim_time": False,
                "footprint_points": footprint_points,
                "odom_topic": "/odom",
                "scan_topic": "/scan_neupan",
                "cmd_vel_in_topic": (
                    "/neupan_nav2_smoothed_cmd_vel"
                    if command_profile == "smoothed" else "/neupan_nav2_cmd_vel"),
                "source_cmd_vel_topic": (
                    "/neupan_nav2_raw_cmd_vel"
                    if command_profile == "smoothed" else "/neupan_nav2_cmd_vel"),
                "cmd_vel_out_topic": cmd_vel_out_topic,
            },
        ],
    )
    localization_monitor = Node(
        package="navigation_neupan_bridge",
        executable="r1_localization_monitor",
        name="r1_localization_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "map_frame": "map",
            "base_frame": "base_link",
            "odom_topic": "/odom",
            "localization_status_topic": "/indoor_location/status",
            "costmap_service": "/global_costmap/get_costmap",
            "status_topic": "/r1_localization_health",
            "tf_timeout_sec": 1.5,
            "odom_timeout_sec": 1.5,
            "footprint_points": footprint_points,
            "footprint_padding_m": 0.01,
            "require_dynamic_footprint": True,
            "dynamic_footprint_topic": "/r1_arm_footprint",
            "dynamic_footprint_timeout_sec": 0.50,
        }],
    )
    nodes = [preprocess, arm_footprint, localization_monitor, collision_guard, worker]
    if scan_fusion is not None:
        nodes.insert(1, scan_fusion)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_cloud", default_value="/deskew_cloud",
            description="High-sampling localization's deskewed PointCloud2 in base_link."),
        DeclareLaunchArgument(
            "scan_source", default_value="livox_only",
            description="NeuPAN obstacle scan source: livox_only or dual (Livox plus rear BlueSea)."),
        DeclareLaunchArgument(
            "python_executable",
            default_value=os.path.expanduser("~/venvs/neupan_ros_humble/bin/python"),
            description="Dedicated Humble NeuPAN Python environment."),
        DeclareLaunchArgument(
            "calibration_record", default_value="",
            description="Approved JSON record for R1 HW1 footprint, drive, LiDAR, and scan calibration."),
        DeclareLaunchArgument(
            "command_profile", default_value="smoothed", choices=["direct", "smoothed"],
            description="Must match navigation_neupan.launch.py for the entire run."),
        DeclareLaunchArgument(
            "cmd_vel_out_topic", default_value="/cmd_vel",
            description="Collision-guard output; use an isolated topic for no-motion integration tests."),
        DeclareLaunchArgument("runtime_report_file", default_value="neupan_hw1_torch_runtime.json"),
        OpaqueFunction(function=_build),
    ])
