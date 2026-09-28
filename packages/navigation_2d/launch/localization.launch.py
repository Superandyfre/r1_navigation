from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    location_config_path = os.path.join(get_package_share_directory("navigation_2d"), "config", "localication_r1_mid360.yaml")
    eskf_cfg_path = os.path.join(get_package_share_directory("navigation_2d"), "config", "eskf_cfg.yaml")

    return LaunchDescription(
        [
            Node(
                package="indoor_location",
                executable="location_node",
                name="location_node",
                output="screen",
                respawn=True,
                parameters=[
                    {
                        "config_file": location_config_path,
                        "eskf_cfg_file": eskf_cfg_path,
                    }
                ],
                arguments=["--ros-args", "--log-level", "WARN"],  # 终端仅仅输出WARN调试日志
            )
        ]
    )
