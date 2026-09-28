"""Reusable PointCloud2 -> LaserScan preprocessing for R1.

The node is the Jazzy pointcloud_to_laserscan implementation.  This package
has no Gazebo dependency: simulation and hardware differ only by the input
topic, profile parameters, and use_sim_time setting.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = 'r1_lidar_preprocess'
PROFILES = ('lidar_smoke', 'lidar_navigation')


def _launch_node(context):
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in PROFILES:
        raise RuntimeError(
            f'Unsupported profile {profile!r}; choose one of {", ".join(PROFILES)}'
        )

    config_file = os.path.join(
        get_package_share_directory(PACKAGE), 'config', f'{profile}.yaml'
    )

    return [Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', LaunchConfiguration('input_topic')),
            ('scan', LaunchConfiguration('output_topic')),
        ],
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='lidar_navigation',
            description='Preprocessing profile: lidar_smoke or lidar_navigation.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/lidar/points',
            description='PointCloud2 input topic; remap this for the real robot.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/scan_neupan',
            description='LaserScan output topic for NeuPAN.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation; keep false for hardware.',
        ),
        OpaqueFunction(function=_launch_node),
    ])
