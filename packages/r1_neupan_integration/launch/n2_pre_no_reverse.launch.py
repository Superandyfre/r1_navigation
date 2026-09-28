"""Standalone Gazebo no-reverse invariant test without a planner publisher."""

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
    gazebo_share = get_package_share_directory("r1_gazebo")
    preprocess_share = get_package_share_directory("r1_lidar_preprocess")
    output = LaunchConfiguration("output_file").perform(context)
    scenario = LaunchConfiguration("scenario").perform(context)
    world = os.path.join(integration_share, "worlds", "free_space.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "r1_nav_sim.launch.py")),
        launch_arguments={"headless": "true", "world": world, "lidar_profile": "lidar_navigation", "rviz": "false"}.items(),
    )
    preprocess = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(preprocess_share, "launch", "pointcloud_to_scan.launch.py")),
        launch_arguments={
            "profile": "lidar_navigation",
            "input_topic": "/lidar/points",
            "output_topic": "/scan_neupan",
            "use_sim_time": "true",
        }.items(),
    )
    gate = Node(
        package="r1_neupan_integration",
        executable="r1_safety_gate",
        name="r1_n2_pre_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "emergency_stop": True,
                "max_linear": 0.15,
                "max_angular": 0.4,
                "max_linear_acceleration": 0.30,
                "max_angular_acceleration": 0.8,
                "command_timeout": 0.30,
                "scan_timeout": 0.30,
                "odom_timeout": 0.30,
                "tf_timeout": 0.30,
                "timing_timeout": 0.50,
                "require_timing": True,
            },
        ],
    )
    probe = Node(
        package="r1_neupan_integration",
        executable="n2_pre_no_reverse_probe",
        name="r1_n2_pre_no_reverse_probe",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "probe_mode": LaunchConfiguration("probe_mode"),
                "probe_angular_z": LaunchConfiguration("probe_angular_z"),
            }
        ],
    )
    audit = Node(
        package="r1_neupan_integration",
        executable="n2_pre3_audit",
        name="r1_n2_pre3_audit",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "scenario": scenario,
                "duration_sec": float(LaunchConfiguration("duration_sec").perform(context)),
                "output_file": output,
            }
        ],
        on_exit=Shutdown(),
    )
    return [gazebo, preprocess, gate, probe, audit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("duration_sec", default_value="12.0"),
        DeclareLaunchArgument("output_file", default_value="n2_pre_results/n2p3r1_m2a.json"),
        DeclareLaunchArgument("scenario", default_value="m2_no_reverse_a"),
        DeclareLaunchArgument("probe_mode", default_value="no_reverse"),
        DeclareLaunchArgument("probe_angular_z", default_value="0.2"),
        OpaqueFunction(function=_build),
    ])
