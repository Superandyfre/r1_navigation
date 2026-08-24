import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare("navigation_2d")
    navigation_share = get_package_share_directory("navigation_2d")

    def launch_file(filename):
        return PythonLaunchDescriptionSource(
            [PathJoinSubstitution([package_share, "launch", filename])]
        )

    default_location_config = os.path.join(
        navigation_share,
        "config",
        "localication_r1_mid360_high_sampling_localization.yaml",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            launch_file("2d_map_server_high_sampling.launch.py")
        ),
        IncludeLaunchDescription(launch_file("livox_undistortion.launch.py")),
        IncludeLaunchDescription(launch_file("laser_undistortion.launch.py")),
        IncludeLaunchDescription(
            launch_file("localization_high_sampling_test.launch.py"),
            launch_arguments={
                "location_config": default_location_config,
            }.items(),
        ),
        IncludeLaunchDescription(launch_file("navigation.launch.py")),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join(navigation_share, "rviz", "nav.rviz"),
            ],
            output="screen",
        ),
    ])
