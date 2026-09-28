from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="livox_undistortion",
                executable="livox_undistortion_node",
                name="livox_undistortion_node",
                output="screen",
                parameters=[{"ExtIL": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}],  # Default identity transform
                remappings=[
                    ("/livox/lidar", "/lidar/pointcloud"),
                    ("/livox/imu", "/imu/imu_data"),
                    ("/livox/lidar_undistort", "/lidar/pointcloud_undistort"),
                ],
                arguments=["--ros-args", "--log-level", "WARN"],  # 终端仅仅输出WARN调试日志
            ),
        ]
    )
