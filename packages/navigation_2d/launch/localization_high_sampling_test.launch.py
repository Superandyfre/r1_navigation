import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("navigation_2d")
    default_config = os.path.join(
        package_share,
        "config",
        "localication_r1_mid360_high_sampling_localization.yaml",
    )
    eskf_cfg = os.path.join(package_share, "config", "eskf_cfg.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "location_config",
            default_value=default_config,
            description="Location node YAML configuration",
        ),
        Node(
            package="indoor_location",
            executable="location_node",
            name="location_node",
            output="screen",
            respawn=True,
            parameters=[
                {
                    "config_file": LaunchConfiguration("location_config"),
                    "eskf_cfg_file": eskf_cfg,
                }
            ],
            arguments=["--ros-args", "--log-level", "WARN"],
        ),
    ])
