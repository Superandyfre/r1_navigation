"""HW1 high-sampling navigation chain with NeuPAN substituted for DWB.

This starts only the static-map, localization, and NeuPAN sensor/control
chains. The chassis driver remains externally owned by robot bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    navigation_share = get_package_share_directory("navigation_2d")
    package_share = FindPackageShare("navigation_2d")
    bridge_share = FindPackageShare("navigation_neupan_bridge")
    arm_bringup_share = FindPackageShare("rokae_dual_arm_bringup")

    def navigation_launch(filename):
        return PythonLaunchDescriptionSource(
            [PathJoinSubstitution([package_share, "launch", filename])])

    default_location_config = os.path.join(
        navigation_share,
        "config",
        "localication_r1_mid360_high_sampling_raw_test.yaml",
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
            description="Approved JSON record required before the HW1 NeuPAN worker can be launched.",
        ),
        DeclareLaunchArgument(
            "scan_source", default_value="livox_only",
            description="NeuPAN obstacle scan source: livox_only or dual.",
        ),
        DeclareLaunchArgument(
            "command_profile", default_value="smoothed",
            description="NeuPAN HW1 command path: smoothed or direct.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic", default_value="/cmd_vel",
            description="Collision-guard output; use an isolated topic for no-motion integration tests.",
        ),
        IncludeLaunchDescription(navigation_launch("2d_map_server_high_sampling.launch.py")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    arm_bringup_share,
                    "launch",
                    "dual_ar5_real_state_navigation.launch.py",
                ])
            ]),
            condition=IfCondition(LaunchConfiguration("start_arm_drivers")),
            launch_arguments={
                "left_robot_ip": LaunchConfiguration("left_robot_ip"),
                "right_robot_ip": LaunchConfiguration("right_robot_ip"),
                "local_ip": LaunchConfiguration("local_ip"),
            }.items(),
        ),
        IncludeLaunchDescription(
            navigation_launch("localization_high_sampling_test.launch.py"),
            launch_arguments={
                "location_config": LaunchConfiguration("location_config"),
            }.items(),
        ),
        # Read-only adapters expose the already-running arm state broadcasters
        # under the names consumed by the dynamic footprint node.  They do not
        # start arm hardware or send arm commands.
        Node(
            package="navigation_neupan_bridge",
            executable="joint_state_prefixer.py",
            name="left_arm_joint_state_prefixer",
            parameters=[{
                "input_topic": "/left/joint_states",
                "output_topic": "/left_arm/joint_states",
                "prefix": "left_",
            }],
            output="screen",
        ),
        Node(
            package="navigation_neupan_bridge",
            executable="joint_state_prefixer.py",
            name="right_arm_joint_state_prefixer",
            parameters=[{
                "input_topic": "/right/joint_states",
                "output_topic": "/right_arm/joint_states",
                "prefix": "right_",
            }],
            output="screen",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            PathJoinSubstitution([bridge_share, "launch", "navigation_neupan.launch.py"]),
        ]), launch_arguments={
            "mode": "hw1",
            "command_profile": LaunchConfiguration("command_profile"),
        }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            PathJoinSubstitution([bridge_share, "launch", "r1_neupan_hw1_motion.launch.py"]),
        ]), launch_arguments={
            "calibration_record": LaunchConfiguration("calibration_record"),
            "scan_source": LaunchConfiguration("scan_source"),
            "command_profile": LaunchConfiguration("command_profile"),
            "cmd_vel_out_topic": LaunchConfiguration("cmd_vel_out_topic"),
        }.items()),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(navigation_share, "rviz", "nav_neupan.rviz")],
            output="screen",
        ),
    ])
