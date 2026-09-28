"""N2.3 B2 evaluation with separate relay and planner-result freshness."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build(context):
    scenario = LaunchConfiguration("scenario").perform(context)
    output = LaunchConfiguration("output_file").perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)
    integration_share = get_package_share_directory("r1_neupan_integration")
    if not world_file:
        world_file = os.path.join(integration_share, "worlds", "n2_b2_wide_channel.sdf")
    n116_dir = LaunchConfiguration("n116_output_dir").perform(context)
    if not n116_dir:
        n116_dir = os.path.splitext(output)[0] + "_bridge"

    n1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(integration_share, "launch", "n1_validation.launch.py")
        ),
        launch_arguments={
            "scenario": "free_space",
            "world_file": world_file,
            "duration_sec": "3600.0",
            "warmup_sec": "0.0",
            "lidar_profile": "lidar_navigation",
            "fixed_path_topic": "/n2_3_unused_path",
            "metrics_file": os.path.splitext(output)[0] + "_n1_metrics.json",
            "metrics_csv_file": os.path.splitext(output)[0] + "_n1_timing.csv",
            "navigation_mode": "terminal",
            "n116_output_dir": n116_dir,
            "n116_timeout_sec": "0.25",
            "latest_scan_only": "true",
            "enable_visualization": "false",
        }.items(),
    )
    relay = Node(
        package="r1_neupan_integration",
        executable="n2_3_command_relay",
        name="r1_n2_3_command_relay",
        output="screen",
        parameters=[{
            "source_topic": "/neupan_planner_result",
            "relay_topic": "/neupan_cmd_envelope",
            "publish_hz": 50.0,
        }],
    )
    scenario_node = Node(
        package="r1_neupan_integration",
        executable="n2_pre_scenarios",
        name="r1_n2_3_scenario",
        output="screen",
        parameters=[{"use_sim_time": True, "scenario": scenario}],
    )
    gate = Node(
        package="r1_neupan_integration",
        executable="r1_safety_gate",
        name="r1_n2_pre_safety_gate",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/neupan_cmd_envelope",
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
    audit = Node(
        package="r1_neupan_integration",
        executable="n2_pre3_audit",
        name="r1_n2_3_audit",
        output="screen",
        on_exit=Shutdown(),
        parameters=[{
            "use_sim_time": True,
            "scenario": scenario,
            "duration_sec": float(LaunchConfiguration("duration_sec").perform(context)),
            "output_file": output,
            "stop_on_arrival": False,
            "capture_detail": LaunchConfiguration("capture_detail").perform(context).lower() == "true",
        }],
    )
    return [n1_launch, relay, scenario_node, gate, audit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="b2_wide_channel"),
        DeclareLaunchArgument("duration_sec", default_value="120.0"),
        DeclareLaunchArgument("capture_detail", default_value="true"),
        DeclareLaunchArgument("world_file", default_value=""),
        DeclareLaunchArgument("output_file", default_value="n2_3_results/p1_b2_run.json"),
        DeclareLaunchArgument("n116_output_dir", default_value=""),
        OpaqueFunction(function=_build),
    ])
