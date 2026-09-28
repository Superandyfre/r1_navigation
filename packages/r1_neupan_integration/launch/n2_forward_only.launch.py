"""Run one independent forward-only N2 navigation evaluation scenario."""

from __future__ import annotations

import os
import json

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
    lidar_mount = LaunchConfiguration("lidar_mount").perform(context)
    if not world_file:
        world_file = os.path.join(
            get_package_share_directory("r1_neupan_integration"),
            "worlds", "free_space.sdf",
        )
    integration_share = get_package_share_directory("r1_neupan_integration")
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
            # N2 audit owns the finite scenario window.  Keep the nested N1
            # metrics process alive so it cannot shut down the launch first.
            "duration_sec": "3600.0",
            "warmup_sec": "0.0",
            "lidar_profile": "lidar_navigation",
            "lidar_mount": lidar_mount,
            # Reproduce the historical N2 failure-input chain exactly:
            # PointCloudPacked -> pointcloud_to_laserscan -> /scan_neupan.
            "direct_scan": "false",
            "fixed_path_topic": "/n2_forward_only_unused_path",
            "metrics_file": os.path.splitext(output)[0] + "_n1_metrics.json",
            "metrics_csv_file": os.path.splitext(output)[0] + "_n1_timing.csv",
            "navigation_mode": "terminal",
            "n116_output_dir": n116_dir,
            "n115_exact_capture_dir": LaunchConfiguration("n116_exact_capture_dir").perform(context),
            "n115_exact_capture_max_records": LaunchConfiguration("n116_exact_capture_max_records").perform(context),
            "n115_capture_scene": scenario,
            "n115_capture_metadata": json.dumps({
                "stage": "N2.4B",
                "mode": "exact_state_capture",
                "scene": scenario,
                "world_file": os.path.abspath(world_file),
                "lidar_mount": lidar_mount,
            }, sort_keys=True),
            "n116_timeout_sec": "0.25",
            "latest_scan_only": "true",
            "enable_visualization": "false",
        }.items(),
    )
    scenario_node = Node(
        package="r1_neupan_integration",
        executable="n2_pre_scenarios",
        name="r1_n2_forward_only_scenario",
        output="screen",
        parameters=[{"use_sim_time": True, "scenario": scenario}],
    )
    gate = Node(
        package="r1_neupan_integration",
        executable="r1_safety_gate",
        # The audit service path is part of the already verified N2-PRE
        # contract; keep the node name stable across N2 stages.
        name="r1_n2_pre_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True, "emergency_stop": True,
             "max_linear": 0.15, "max_angular": 0.4,
             "max_linear_acceleration": 0.30,
             "max_angular_acceleration": 0.8,
             "command_timeout": 0.30, "scan_timeout": 0.30,
             "odom_timeout": 0.30, "tf_timeout": 0.30,
             "timing_timeout": 0.50, "require_timing": True},
        ],
    )
    audit = Node(
        package="r1_neupan_integration",
        executable="n2_pre3_audit",
        name="r1_n2_pre_audit",
        output="screen",
        on_exit=Shutdown(),
        parameters=[{"use_sim_time": True, "scenario": scenario,
                     "duration_sec": float(LaunchConfiguration("duration_sec").perform(context)),
                     "output_file": output}],
    )
    return [n1_launch, scenario_node, gate, audit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="a1_straight"),
        DeclareLaunchArgument("duration_sec", default_value="32.0"),
        DeclareLaunchArgument("world_file", default_value=""),
        DeclareLaunchArgument("lidar_mount", default_value="base_link"),
        DeclareLaunchArgument("output_file", default_value="n2_results/a1_straight.json"),
        DeclareLaunchArgument("n116_output_dir", default_value=""),
        DeclareLaunchArgument("n116_exact_capture_dir", default_value=""),
        DeclareLaunchArgument("n116_exact_capture_max_records", default_value="512"),
        OpaqueFunction(function=_build),
    ])
