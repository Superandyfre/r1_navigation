"""N2.4C reproducibility audit with the frozen N2.3 relay contract."""

from __future__ import annotations

import json
import hashlib
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build(context):
    integration_share = get_package_share_directory("r1_neupan_integration")
    scenario = LaunchConfiguration("scenario").perform(context)
    output = LaunchConfiguration("output_file").perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)
    if not world_file:
        world_file = os.path.join(integration_share, "worlds", f"n2_{scenario}.sdf")
    world_file = os.path.abspath(world_file)
    n116_dir = LaunchConfiguration("n116_output_dir").perform(context)
    exact_dir = LaunchConfiguration("exact_capture_dir").perform(context)
    stack_manifest = LaunchConfiguration("current_stack_manifest").perform(context)
    stack_manifest_sha256 = LaunchConfiguration(
        "current_stack_manifest_sha256"
    ).perform(context)
    if stack_manifest:
        stack_manifest = os.path.abspath(stack_manifest)
        if not os.path.isfile(stack_manifest):
            raise RuntimeError(f"current stack manifest does not exist: {stack_manifest}")
        digest = hashlib.sha256()
        with open(stack_manifest, "rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        actual_manifest_sha256 = digest.hexdigest()
        if stack_manifest_sha256 != actual_manifest_sha256:
            raise RuntimeError(
                "current stack manifest hash mismatch: "
                f"expected {stack_manifest_sha256}, got {actual_manifest_sha256}"
            )
    elif stack_manifest_sha256:
        raise RuntimeError(
            "current_stack_manifest_sha256 requires current_stack_manifest"
        )
    if not n116_dir:
        n116_dir = os.path.splitext(output)[0] + "_bridge"

    n1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(integration_share, "launch", "n1_validation.launch.py")
        ),
        launch_arguments={
            "scenario": "free_space",
            "world_file": world_file,
            "duration_sec": "3600.0",
            "warmup_sec": "0.0",
            "lidar_profile": "lidar_navigation",
            "lidar_mount": "base_link",
            "direct_scan": "false",
            "fixed_path_topic": "/n2_4c_unused_path",
            "metrics_file": os.path.splitext(output)[0] + "_n1_metrics.json",
            "metrics_csv_file": os.path.splitext(output)[0] + "_n1_timing.csv",
            "runtime_report_file": os.path.splitext(output)[0] + "_torch_runtime.json",
            "navigation_mode": "terminal",
            "n116_output_dir": n116_dir,
            "n115_exact_capture_dir": exact_dir,
            "n115_exact_capture_max_records": LaunchConfiguration(
                "exact_capture_max_records"
            ).perform(context),
            "n115_capture_scene": scenario,
            "n115_capture_metadata": json.dumps({
                "stage": "N2.4C2",
                "mode": "current_stack_matrix_completion",
                "scene": scenario,
                "world_file": world_file,
                "relay_contract": "n2_3_command_relay+freshness_envelope",
                "recovery_commands_allowed": False,
                "current_stack_manifest": stack_manifest,
                "current_stack_manifest_sha256": stack_manifest_sha256,
            }, sort_keys=True),
            "n116_timeout_sec": "0.25",
            "latest_scan_only": "true",
            "enable_visualization": "false",
        }.items(),
    )

    relay = Node(
        package="r1_neupan_integration",
        executable="n2_3_command_relay",
        name="r1_n2_3_command_relay",
        output="screen",
        parameters=[{
            "source_topic": "/neupan_planner_result",
            "relay_topic": "/neupan_cmd_envelope",
            "publish_hz": 50.0,
        }],
    )
    scenario_node = Node(
        package="r1_neupan_integration",
        executable="n2_pre_scenarios",
        name="r1_n2_4c_scenario",
        output="screen",
        parameters=[{"use_sim_time": True, "scenario": scenario}],
    )
    gate = Node(
        package="r1_neupan_integration",
        executable="r1_safety_gate",
        name="r1_n2_pre_safety_gate",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/neupan_cmd_envelope",
            "command_mode": "freshness_envelope",
            "planner_context_topic": "/neupan_planner_context",
            "emergency_stop": True,
            "max_linear": 0.15,
            "max_angular": 0.4,
            "max_linear_acceleration": 0.30,
            "max_angular_acceleration": 0.8,
            "command_timeout": 0.30,
            "relay_timeout": 0.30,
            "planner_freshness_timeout": 0.30,
            "scan_timeout": 0.30,
            "odom_timeout": 0.30,
            "tf_timeout": 0.30,
            "timing_timeout": 0.50,
            "require_timing": True,
        }],
    )
    audit = Node(
        package="r1_neupan_integration",
        executable="n2_pre3_audit",
        name="r1_n2_4c_audit",
        output="screen",
        on_exit=Shutdown(),
        parameters=[{
            "use_sim_time": True,
            "scenario": scenario,
            "duration_sec": float(LaunchConfiguration("duration_sec").perform(context)),
            "output_file": output,
            "stop_on_arrival": False,
            "capture_detail": True,
            "runtime_attribution_file": LaunchConfiguration(
                "runtime_attribution_file"
            ).perform(context),
            "runtime_run_label": LaunchConfiguration(
                "runtime_run_label"
            ).perform(context),
        }],
    )
    return [n1_launch, relay, scenario_node, gate, audit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="b4_center_obstacle"),
        DeclareLaunchArgument("duration_sec", default_value="32.0"),
        DeclareLaunchArgument("world_file", default_value=""),
        DeclareLaunchArgument("output_file", default_value="n2_4c_results/run.json"),
        DeclareLaunchArgument("n116_output_dir", default_value=""),
        DeclareLaunchArgument("exact_capture_dir", default_value=""),
        DeclareLaunchArgument("exact_capture_max_records", default_value="512"),
        DeclareLaunchArgument("current_stack_manifest", default_value=""),
        DeclareLaunchArgument("current_stack_manifest_sha256", default_value=""),
        DeclareLaunchArgument("runtime_attribution_file", default_value=""),
        DeclareLaunchArgument("runtime_run_label", default_value="unknown"),
        OpaqueFunction(function=_build),
    ])
