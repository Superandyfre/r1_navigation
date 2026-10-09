"""Start the minimal Nav2 service set required by the NeuPAN controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _build(context):
    bridge_share = get_package_share_directory("navigation_neupan_bridge")
    mode = LaunchConfiguration("mode").perform(context)
    command_profile = LaunchConfiguration("command_profile").perform(context)
    if mode not in {"shadow", "hw1"}:
        raise RuntimeError("NeuPAN mode must be shadow or hw1")
    if command_profile not in {"direct", "smoothed"}:
        raise RuntimeError("NeuPAN command_profile must be direct or smoothed")
    params_name = (
        "nav2_params_r1_neupan_shadow.yaml"
        if mode == "shadow" else "nav2_params_r1_neupan_hw1.yaml"
    )
    use_sim_time = False
    controller_remappings = [("cmd_vel", "/cmd_vel")]
    use_velocity_smoother = mode == "hw1" and command_profile == "smoothed"
    if mode == "hw1":
        # The HW1 collision guard is the only publisher allowed to reach the
        # chassis topic. Keep shadow behavior unchanged.
        controller_remappings = [(
            "cmd_vel",
            "/neupan_nav2_raw_cmd_vel" if use_velocity_smoother else "/neupan_nav2_cmd_vel")]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(bridge_share, "config", params_name),
            # RewrittenYaml expects launch substitutions, not Python bools.
            param_rewrites={"use_sim_time": "false", "autostart": "true"},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = ["controller_server", "planner_server", "bt_navigator"]
    if use_velocity_smoother:
        lifecycle_nodes.insert(0, "neupan_velocity_smoother")
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": lifecycle_nodes,
        }],
    )
    lifecycle_ensure = Node(
        package="navigation_neupan_bridge",
        executable="nav_lifecycle_ensure",
        output="screen",
        parameters=[{"node_names": lifecycle_nodes, "max_attempts": 0}],
    )
    nodes = [
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="neupan_velocity_smoother",
            output="screen",
            parameters=[configured_params],
            remappings=[
                ("cmd_vel", "/neupan_nav2_raw_cmd_vel"),
                ("cmd_vel_smoothed", "/neupan_nav2_smoothed_cmd_vel"),
            ],
        ) if use_velocity_smoother else None,
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=controller_remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
            remappings=remappings,
        ),
        # ControllerServer constructs its Costmap2DROS before exposing its
        # lifecycle services.  Delay the manager past that startup window.
        TimerAction(period=10.0, actions=[lifecycle_manager]),
        # On this machine a lifecycle service response can be lost during
        # initial plugin loading. Reconcile the three required Nav2 nodes
        # after the manager has had its first chance to bring them up.
        TimerAction(period=20.0, actions=[lifecycle_ensure]),
    ]
    return [node for node in nodes if node is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="shadow", choices=["shadow", "hw1"]),
        DeclareLaunchArgument(
            "command_profile", default_value="smoothed", choices=["direct", "smoothed"],
            description="HW1 command path: direct guard input or closed-loop smoothed input."),
        OpaqueFunction(function=_build),
    ])
