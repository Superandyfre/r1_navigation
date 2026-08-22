from sympy import false
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    laser_undistortion_log_level = LaunchConfiguration("laser_undistortion_log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument("laser_undistortion_log_level", default_value="warn", description="laser undistortion log level"),
            Node(
                package="lidar_undistortion_2d",
                executable="lidar_undistortion_2d_node",
                name="lidar_undistortion_2d_node",
                output="screen",
                arguments=["--ros-args", "--log-level", laser_undistortion_log_level],
                parameters=[
                    {
                        "laser_frame": "laser_link",
                        "imu_frame": "imu_link",
                    }
                ],
                remappings=[
                    ("/scan", "/scan"),
                    ("/imu", "/imu_data"),
                    ("/scan_undistort", "/laser/scan_undistort"),
                ],
            ),
        ]
    )
