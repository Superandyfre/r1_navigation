import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config_file = os.path.join(get_package_share_directory("navigation_2d"), "rviz", "nav.rviz")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("robot_bringup"),
                                "launch",
                                "robot_driver.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("navigation_2d"), "launch", "2d_map_server.launch.py"])])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("navigation_2d"), "launch", "livox_undistortion.launch.py"])])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("navigation_2d"), "launch", "localization.launch.py"])])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("navigation_2d"), "launch", "navigation.launch.py"])])
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            ),
        ]
    )
