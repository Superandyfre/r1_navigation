"""N2-PRE admission harness with a default-locked R1 safety relay."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build(context):
    integration_share = get_package_share_directory("r1_neupan_integration")
    scenario = LaunchConfiguration("scenario").perform(context)
    # Keep legacy reverse scenarios available for historical reproduction.
    # N2-PRE.3R1 admission uses m2_no_reverse_a/b instead.
    valid = {"forward", "reverse", "m1_forward", "m2_reverse", "m3_left_rotate", "m4_right_rotate", "left", "right", "arrive",
             "r0", "r1", "r2", "r3", "a0", "a1", "a2", "a3", "a4", "a5",
             "front_obstacle", "left_obstacle", "right_obstacle"}
    if scenario not in valid:
        raise RuntimeError(f"scenario must be one of {sorted(valid)}")
    output = LaunchConfiguration("output_file").perform(context)
    n116_dir = LaunchConfiguration("n116_output_dir").perform(context)
    if not n116_dir:
        n116_dir = os.path.splitext(output)[0] + "_bridge"
    n1_scenario = "obstacle" if scenario.endswith("_obstacle") else "free_space"
    sensor_fault = LaunchConfiguration("sensor_fault").perform(context)
    if sensor_fault not in {"", "scan", "odom_tf"}:
        raise RuntimeError("sensor_fault must be empty, scan, or odom_tf")
    relay_parameters = {
        "use_sim_time": True,
        "drop_after_sec": float(LaunchConfiguration("sensor_fault_drop_after_sec").perform(context)),
        "relay_scan": sensor_fault == "scan",
        "relay_odom_tf": sensor_fault == "odom_tf",
    }
    relay = Node(
        package="r1_neupan_integration",
        executable="n2_pre3_sensor_relay",
        name="n2_pre3_sensor_relay",
        output="screen",
        parameters=[relay_parameters],
    ) if sensor_fault else None
    n1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(integration_share, "launch", "n1_validation.launch.py")
        ),
        launch_arguments={
            "scenario": n1_scenario,
            "duration_sec": "3600",
            "warmup_sec": "0",
            "lidar_profile": "lidar_navigation",
            "fixed_path_topic": "/n2_pre_unused_path",
            "navigation_mode": LaunchConfiguration("navigation_mode"),
            "n116_output_dir": n116_dir,
            "n116_fault_mode": LaunchConfiguration("n116_fault_mode"),
            "n116_fault_request_id": LaunchConfiguration("n116_fault_request_id"),
            "n116_fault_sleep_sec": LaunchConfiguration("n116_fault_sleep_sec"),
            "metrics_file": os.path.splitext(output)[0] + "_n1_metrics.json",
            "metrics_csv_file": os.path.splitext(output)[0] + "_n1_timing.csv",
            "latest_scan_only": "true",
            "enable_visualization": "false",
            "n116_timeout_sec": "0.25",
        }.items(),
    )
    path = Node(
        package="r1_neupan_integration",
        executable="n2_pre_scenarios",
        name="r1_n2_pre_scenario",
        output="screen",
        parameters=[{"use_sim_time": True, "scenario": scenario}],
    )
    gate = Node(
        package="r1_neupan_integration",
        executable="r1_safety_gate",
        name="r1_n2_pre_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"emergency_stop": True, "max_linear": 0.15, "max_angular": 0.4,
             "max_linear_acceleration": 0.30, "max_angular_acceleration": 0.8,
             "command_timeout": 0.30, "scan_timeout": 0.30,
             "odom_timeout": 0.30, "tf_timeout": 0.30,
             "timing_timeout": 0.50, "require_timing": True,
             "scan_topic": LaunchConfiguration("gate_scan_topic"),
             "odom_topic": LaunchConfiguration("gate_odom_topic"),
             "tf_topic": LaunchConfiguration("gate_tf_topic")},
        ],
    )
    audit = Node(
        package="r1_neupan_integration",
        executable="n2_pre3_audit",
        name="r1_n2_pre_audit",
        output="screen",
        parameters=[{"use_sim_time": True, "scenario": scenario,
                     "duration_sec": float(LaunchConfiguration("duration_sec").perform(context)),
                     "output_file": output}],
        on_exit=Shutdown(),
    )
    actions = [n1_launch]
    if relay is not None:
        actions.append(relay)
    actions.extend([path, gate, audit])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="forward"),
        DeclareLaunchArgument("duration_sec", default_value="20.0"),
        DeclareLaunchArgument("output_file", default_value="n2_pre_results/forward.json"),
        DeclareLaunchArgument("n116_output_dir", default_value=""),
        DeclareLaunchArgument("n116_fault_mode", default_value=""),
        DeclareLaunchArgument("n116_fault_request_id", default_value="-1"),
        DeclareLaunchArgument("n116_fault_sleep_sec", default_value="1.0"),
        DeclareLaunchArgument("navigation_mode", default_value="terminal"),
        DeclareLaunchArgument("sensor_fault", default_value=""),
        DeclareLaunchArgument("sensor_fault_drop_after_sec", default_value="18.0"),
        DeclareLaunchArgument("gate_scan_topic", default_value="/scan_neupan"),
        DeclareLaunchArgument("gate_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("gate_tf_topic", default_value="/tf"),
        OpaqueFunction(function=_build),
    ])
