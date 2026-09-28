"""UI high-sampling navigation chain with NeuPAN substituted for DWB.

This mirrors ``local_and_nav_high_sampling_test.launch.py``.  The only
navigation difference is the NeuPAN shadow controller and its e-stop-latched
safety chain; the chassis driver remains externally owned by robot bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    navigation_share = get_package_share_directory("navigation_2d")
    package_share = FindPackageShare("navigation_2d")
    bridge_share = FindPackageShare("navigation_neupan_bridge")

    def navigation_launch(filename):
        return PythonLaunchDescriptionSource(
            [PathJoinSubstitution([package_share, "launch", filename])])

    default_location_config = os.path.join(
        navigation_share,
        "config",
        "localication_r1_mid360_high_sampling_localization.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "location_config",
            default_value=default_location_config,
            description="Location node YAML configuration",
        ),
        DeclareLaunchArgument(
            "start_arm_drivers",
            default_value="false",
            description="Start real AR5 state broadcasters for dynamic self filtering",
        ),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.11.60"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.11.61"),
        DeclareLaunchArgument("local_ip", default_value="192.168.11.11"),
        DeclareLaunchArgument(
            "calibration_record", default_value="",
            description="Approved R1 LiDAR-to-base calibration used by the shared NeuPAN scan chain.",
        ),
        IncludeLaunchDescription(navigation_launch("2d_map_server_high_sampling.launch.py")),
        IncludeLaunchDescription(
            navigation_launch("localization_high_sampling_test.launch.py"),
            launch_arguments={
                "location_config": LaunchConfiguration("location_config"),
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            PathJoinSubstitution([bridge_share, "launch", "navigation_neupan.launch.py"]),
        ]), launch_arguments={"mode": "shadow"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            PathJoinSubstitution([bridge_share, "launch", "r1_neupan_hw0_shadow.launch.py"]),
        ]), launch_arguments={
            "calibration_record": LaunchConfiguration("calibration_record"),
        }.items()),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(navigation_share, "rviz", "nav_neupan.rviz")],
            output="screen",
        ),
    ])
