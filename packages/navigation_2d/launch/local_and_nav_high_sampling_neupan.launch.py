"""HW1 high-sampling navigation chain with NeuPAN substituted for DWB.

This starts only the static-map, localization, and NeuPAN sensor/control
chains. The chassis driver remains externally owned by robot bringup.
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _has_live_joint_state(topic):
    """Return true only after receiving a complete, freshly published AR5 state."""
    try:
        result = subprocess.run(
            [
                "/opt/ros/humble/bin/ros2", "topic", "echo", topic, "--once",
                "--qos-reliability", "best_effort",
                "--qos-durability", "volatile",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except subprocess.TimeoutExpired:
        return False

    # A publisher count is unsafe: a stale adapter can remain discoverable
    # while publishing nothing.  Require all seven raw arm joints and values.
    joints = set(re.findall(r"^\s*-\s*joint([1-7])\s*$", result.stdout, re.MULTILINE))
    position_match = re.search(
        r"^position:\s*$(.*?)(?=^velocity:)", result.stdout,
        re.MULTILINE | re.DOTALL,
    )
    positions = re.findall(
        r"^\s*-\s*[-+0-9.eE]+\s*$",
        position_match.group(1) if position_match else "",
        re.MULTILINE,
    )
    return result.returncode == 0 and joints == set("1234567") and len(positions) == 7


def _arm_streams_running():
    """Inspect live direct hardware state topics, excluding navigation adapters."""
    return (
        _has_live_joint_state("/left/joint_states"),
        _has_live_joint_state("/right/joint_states"),
    )


def generate_launch_description():
    navigation_share = get_package_share_directory("navigation_2d")
    package_share = FindPackageShare("navigation_2d")
    bridge_share = FindPackageShare("navigation_neupan_bridge")
    arm_bringup_share = FindPackageShare("rokae_dual_arm_bringup")

    def navigation_launch(filename):
        return PythonLaunchDescriptionSource(
            [PathJoinSubstitution([package_share, "launch", filename])])

    default_location_config = os.path.join(
        navigation_share,
        "config",
        "localication_r1_mid360_high_sampling_raw_test.yaml",
    )

    def navigation_actions():
        return [
            IncludeLaunchDescription(navigation_launch("2d_map_server_high_sampling.launch.py")),
            IncludeLaunchDescription(
                navigation_launch("localization_high_sampling_test.launch.py"),
                launch_arguments={
                    "location_config": LaunchConfiguration("location_config"),
                }.items(),
            ),
            # dual_ar5_real_state_navigation.launch.py owns these adapters.
            # Starting another pair here duplicates each physical joint-state
            # stream and can starve the live arm footprint projection.
            IncludeLaunchDescription(PythonLaunchDescriptionSource([
                PathJoinSubstitution([bridge_share, "launch", "navigation_neupan.launch.py"]),
            ]), launch_arguments={
                "mode": "hw1",
                "command_profile": LaunchConfiguration("command_profile"),
            }.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource([
                PathJoinSubstitution([bridge_share, "launch", "r1_neupan_hw1_motion.launch.py"]),
            ]), launch_arguments={
                "calibration_record": LaunchConfiguration("calibration_record"),
                "scan_source": LaunchConfiguration("scan_source"),
                "use_mesh_self_filter": LaunchConfiguration("use_mesh_self_filter"),
                "mesh_projection_margin_m": LaunchConfiguration("mesh_projection_margin_m"),
                "command_profile": LaunchConfiguration("command_profile"),
                "cmd_vel_out_topic": LaunchConfiguration("cmd_vel_out_topic"),
                "collision_check_enabled": LaunchConfiguration("collision_check_enabled"),
            }.items()),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(navigation_share, "rviz", "nav_neupan.rviz")],
                output="screen",
            ),
        ]

    def after_arm_gate(event, context):
        if event.returncode == 0:
            return [LogInfo(msg="Real dual-arm state flow ready; starting NeuPAN navigation"), *navigation_actions()]
        return [
            LogInfo(msg="ERROR: NeuPAN navigation was not started: real dual-arm state flow is unavailable"),
            EmitEvent(event=Shutdown(reason="dual-arm state gate failed")),
        ]

    def admit_after_arm_state(context):
        left_running, right_running = _arm_streams_running()
        if left_running != right_running:
            return [
                LogInfo(msg="ERROR: Only one real arm state stream is running; refusing navigation to avoid a duplicate hardware connection"),
                EmitEvent(event=Shutdown(reason="partial dual-arm state flow")),
            ]

        gate = Node(
            package="navigation_2d",
            executable="arm_state_gate.py",
            arguments=["--timeout-seconds", "45"],
            output="screen",
        )
        actions = []
        if not left_running:
            actions.extend([
                LogInfo(msg="Real dual-arm state flow absent; starting it before navigation"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            arm_bringup_share,
                            "launch",
                            "dual_ar5_real_state_navigation.launch.py",
                        ])
                    ]),
                    launch_arguments={
                        "left_robot_ip": LaunchConfiguration("left_robot_ip"),
                        "right_robot_ip": LaunchConfiguration("right_robot_ip"),
                        "local_ip": LaunchConfiguration("local_ip"),
                    }.items(),
                ),
            ])
        else:
            actions.append(LogInfo(msg="Using already-running real dual-arm state flow"))
        actions.extend([
            gate,
            RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_arm_gate)),
        ])
        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            "location_config",
            default_value=default_location_config,
            description="Location node YAML configuration",
        ),
        DeclareLaunchArgument(
            "start_arm_drivers",
            default_value="auto",
            description="Deprecated compatibility argument; real dual-arm state is automatically gated.",
        ),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.11.60"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.11.61"),
        DeclareLaunchArgument("local_ip", default_value="192.168.11.11"),
        DeclareLaunchArgument(
            "calibration_record", default_value="",
            description="Approved JSON record required before the HW1 NeuPAN worker can be launched.",
        ),
        DeclareLaunchArgument(
            "scan_source", default_value="livox_only",
            description="NeuPAN obstacle scan source: livox_only or dual.",
        ),
        DeclareLaunchArgument(
            "use_mesh_self_filter", default_value="true", choices=["false", "true"],
            description="Use the dual-arm mesh self-return filter for NeuPAN obstacle scanning.",
        ),
        DeclareLaunchArgument(
            "collision_check_enabled", default_value="false", choices=["false"],
            description="HW1 uses NeuPAN collision avoidance; the final node enforces runtime authorization only."),
        DeclareLaunchArgument(
            "mesh_projection_margin_m", default_value="0.06",
            description="Local mesh-only LiDAR self-return edge tolerance in metres.",
        ),
        DeclareLaunchArgument(
            "command_profile", default_value="smoothed",
            description="NeuPAN HW1 command path: smoothed or direct.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic", default_value="/cmd_vel",
            description="Collision-guard output; use an isolated topic for no-motion integration tests.",
        ),
        OpaqueFunction(function=admit_after_arm_state),
    ])
