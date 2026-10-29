"""NeuPAN HW0 execution chain for Humble.

This launch intentionally omits the chassis driver. The gate starts with an
e-stop latch and is the only node allowed to publish the final /cmd_vel topic.
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
    """Shadow and active NeuPAN must use the same robot TF calibration."""
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
    python_executable = LaunchConfiguration("python_executable").perform(context)
    input_cloud = LaunchConfiguration("input_cloud").perform(context)
    calibration_record = LaunchConfiguration("calibration_record").perform(context)
    if not os.path.isfile(python_executable):
        raise RuntimeError(
            "NeuPAN Humble venv is unavailable: " + python_executable +
            ". Create it and install the vendored NeuPAN core before HW0."
        )
    if not calibration_record or not os.path.isfile(calibration_record):
        raise RuntimeError(
            "Shadow mode requires the approved HW1 calibration record so its "
            "scan chain exactly matches the active NeuPAN mode."
        )
    try:
        with open(calibration_record, encoding="utf-8") as record_file:
            calibration = json.load(record_file)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Shadow calibration record is unreadable: " + str(error)) from error
    if calibration.get("schema") != "r1_hw1_calibration_record_v1" or not calibration.get("approved_for_hw1"):
        raise RuntimeError("Shadow calibration record is not approved for HW1")
    lidar_to_base = calibration.get("lidar_to_base")
    if not isinstance(lidar_to_base, dict):
        raise RuntimeError("Shadow calibration record is missing lidar_to_base")
    _validate_lidar_calibration(calibration)
    footprint = calibration.get("footprint")
    if not isinstance(footprint, dict):
        raise RuntimeError("Shadow calibration record is missing footprint")
    polygon_xy = footprint.get("polygon_xy")
    if polygon_xy:
        footprint_points = [coordinate for point in polygon_xy for coordinate in point]
    else:
        try:
            half_length = float(footprint["length_m"]) / 2.0
            half_width = float(footprint["width_m"]) / 2.0
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError("Shadow calibration record has invalid footprint dimensions") from error
        footprint_points = [
            half_length, half_width, half_length, -half_width,
            -half_length, -half_width, -half_length, half_width,
        ]

    config_dir = os.path.join(bridge_share, "config")
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

    preprocess = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="neupan_pointcloud_to_scan",
        output="screen",
        remappings=[("cloud_in", input_cloud), ("scan", "/scan_neupan")],
        parameters=[os.path.join(config_dir, "lidar_r1_hw1.yaml"), {"use_sim_time": False}],
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
                "control_frequency": 10.0,
                "latest_scan_only": True,
                "navigation_mode": "terminal",
                "planner_result_topic": "/neupan_planner_result",
                "require_execution_context": False,
                "planner_context_topic": "/neupan_planner_context",
                "bridge_status_topic": "/neupan_bridge_status",
                "torch_num_threads": 8,
                "torch_num_interop_threads": 24,
                # The fault is still default-off and must be explicitly set
                # at runtime for a HW0 test. This launch is the sole place
                # that authorizes the test hook to exist.
                "shadow_tf_fault_injection_allowed": True,
                "shadow_force_robot_tf_failure": False,
            },
        ],
    )
    relay = Node(
        package="r1_neupan_integration",
        executable="n2_3_command_relay",
        name="r1_n2_3_command_relay",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "source_topic": "/neupan_planner_result",
            "relay_topic": "/neupan_cmd_envelope",
            "publish_hz": 50.0,
        }],
    )
    gate = Node(
        package="r1_neupan_integration",
        executable="r1_safety_gate",
        name="r1_hw0_safety_gate",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "input_topic": "/neupan_cmd_envelope",
            "output_topic": "/cmd_vel",
            "command_mode": "freshness_envelope",
            "planner_context_topic": "/neupan_planner_context",
            "emergency_stop": True,
            "max_linear": 0.15,
            "max_angular": 0.4,
            "max_linear_acceleration": 0.30,
            "max_angular_acceleration": 0.8,
            "command_timeout": 0.30,
            "relay_timeout": 0.30,
            "planner_freshness_timeout": 0.30,
            "scan_timeout": 0.30,
            "odom_timeout": 0.30,
            "tf_timeout": 0.30,
            "timing_timeout": 0.50,
            "require_timing": True,
        }],
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
        }],
    )
    return [preprocess, localization_monitor, worker, relay, gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_cloud", default_value="/deskew_cloud",
            description="High-sampling localization's deskewed PointCloud2 in base_link, shared with active NeuPAN mode."),
        DeclareLaunchArgument(
            "calibration_record", default_value="",
            description="Approved R1 calibration record required to match the active NeuPAN scan chain."),
        DeclareLaunchArgument(
            "python_executable",
            default_value=os.path.expanduser("~/venvs/neupan_ros_humble/bin/python"),
            description="Dedicated Humble NeuPAN Python environment."),
        DeclareLaunchArgument("runtime_report_file", default_value="neupan_hw0_torch_runtime.json"),
        OpaqueFunction(function=_build),
    ])
