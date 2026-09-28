from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {
                "yaml_filename": "/home/rpp/rpp_data/map_high_sampling/variants/conservative/map.yaml",
                "topic_name": "map",
                "frame_id": "map",
            }
        ],
        output="screen",
    )

    # Let DDS discover map_server's lifecycle services before the manager sends
    # its first Configure request. On this R1, map_server can take several
    # seconds to register while the real sensor graph is also starting; a short
    # delay can lose the one-shot Configure request and leave /map unpublished.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_2d_map_server_high_sampling",
        output="screen",
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # The lifecycle manager occasionally loses its first Configure response on
    # this machine while the sensor graph is starting. This one-shot checker
    # only completes the map server lifecycle when it is still not active.
    lifecycle_ensure = Node(
        package="navigation_2d",
        executable="map_server_lifecycle_ensure",
        output="screen",
    )

    return LaunchDescription(
        [
            map_server,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=map_server,
                    on_start=[
                        TimerAction(period=10.0, actions=[lifecycle_manager]),
                        TimerAction(period=15.0, actions=[lifecycle_ensure]),
                    ],
                )
            ),
        ]
    )
