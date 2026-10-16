"""Static controller selection for high-sampling navigation.

``neupan`` is intentionally rejected: this repository only admits NeuPAN to
HW0 shadow operation. A future HW1 approval must add a separately reviewed
motion launch instead of changing the shadow launch in place.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _select(context):
    mode = LaunchConfiguration("local_controller_mode").perform(context)
    package_share = FindPackageShare("navigation_2d")
    launches = {
        "dwb": "local_and_nav_high_sampling.launch.py",
        "neupan_shadow": "local_and_nav_high_sampling_neupan_shadow.launch.py",
        "neupan": "local_and_nav_high_sampling_neupan.launch.py",
    }
    if mode not in launches:
        raise RuntimeError("local_controller_mode must be dwb, neupan_shadow, or neupan")
    launch_arguments = {}
    if mode in {"neupan_shadow", "neupan"}:
        launch_arguments = {
            "location_config": LaunchConfiguration("location_config"),
            "start_arm_drivers": LaunchConfiguration("start_arm_drivers"),
            "left_robot_ip": LaunchConfiguration("left_robot_ip"),
            "right_robot_ip": LaunchConfiguration("right_robot_ip"),
            "local_ip": LaunchConfiguration("local_ip"),
        }
        if mode in {"neupan_shadow", "neupan"}:
            launch_arguments["calibration_record"] = LaunchConfiguration("calibration_record")
        if mode == "neupan":
            launch_arguments["scan_source"] = LaunchConfiguration("scan_source")
            launch_arguments["command_profile"] = LaunchConfiguration("command_profile")
            launch_arguments["cmd_vel_out_topic"] = LaunchConfiguration("cmd_vel_out_topic")
            launch_arguments["collision_check_enabled"] = LaunchConfiguration("collision_check_enabled")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_share, "launch", launches[mode]]),
        ]),
        launch_arguments=launch_arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "local_controller_mode",
            default_value="dwb",
            description="Static controller selection: dwb, neupan_shadow, or neupan (HW1 blocked)."),
        DeclareLaunchArgument(
            "location_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("navigation_2d"),
                "config",
                "localication_r1_mid360_high_sampling_raw_test.yaml",
            ]),
            description="High-sampling localization YAML shared with the production UI.",
        ),
        DeclareLaunchArgument(
            "start_arm_drivers",
            default_value="false",
            description="Start AR5 state broadcasters for the shadow self-filter.",
        ),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.11.60"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.11.61"),
        DeclareLaunchArgument("local_ip", default_value="192.168.11.11"),
        DeclareLaunchArgument(
            "calibration_record", default_value="",
            description="Approved JSON calibration record required for local_controller_mode:=neupan.",
        ),
        DeclareLaunchArgument(
            "scan_source", default_value="livox_only",
            description="NeuPAN obstacle scan source: livox_only or dual.",
        ),
        DeclareLaunchArgument(
            "command_profile", default_value="smoothed",
            description="NeuPAN HW1 command path: smoothed or direct.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic", default_value="/cmd_vel",
            description="Collision-guard output; use an isolated topic for no-motion integration tests.",
        ),
        DeclareLaunchArgument(
            "collision_check_enabled", default_value="true", choices=["false", "true"],
            description="Enable final obstacle collision checks."),
        OpaqueFunction(function=_select),
    ])
