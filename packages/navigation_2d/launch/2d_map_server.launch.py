from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_server",
                parameters=[
                    {"yaml_filename": "/home/rpp/rpp_data/map/filtered/map.yaml"},
                    {"topic_name": "map"},
                    {"frame_id": "map"},
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_2d_map_server",
                output="screen",
                parameters=[{"autostart": True}, {"node_names": ["map_server"]}],
            ),
        ]
    )
