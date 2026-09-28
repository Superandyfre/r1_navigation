"""N2.4C-R runtime attribution around the unchanged N2.4C launch."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _build(context):
    share = get_package_share_directory("r1_neupan_integration")
    scenario = LaunchConfiguration("scenario").perform(context)
    output = os.path.abspath(LaunchConfiguration("output_file").perform(context))
    world = LaunchConfiguration("world_file").perform(context)
    if not world:
        world = os.path.join(share, "worlds", f"n2_{scenario}.sdf")
    runtime_output = os.path.abspath(LaunchConfiguration("runtime_file").perform(context))
    run_label = LaunchConfiguration("run_label").perform(context)
    stack_manifest = LaunchConfiguration("current_stack_manifest").perform(context)
    stack_manifest_sha256 = LaunchConfiguration(
        "current_stack_manifest_sha256"
    ).perform(context)
    n116_output = os.path.splitext(runtime_output)[0] + "_bridge"
    exact_output = os.path.splitext(runtime_output)[0] + "_exact"
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "n2_4c_reproducibility.launch.py")
        ),
        launch_arguments={
            "scenario": scenario,
            "duration_sec": LaunchConfiguration("duration_sec").perform(context),
            "world_file": os.path.abspath(world),
            "output_file": output,
            "n116_output_dir": n116_output,
            "exact_capture_dir": exact_output,
            "exact_capture_max_records": "2048",
            "runtime_attribution_file": runtime_output,
            "runtime_run_label": run_label,
            "current_stack_manifest": stack_manifest,
            "current_stack_manifest_sha256": stack_manifest_sha256,
        }.items(),
    )
    return [base]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="a1_straight"),
        DeclareLaunchArgument("duration_sec", default_value="35.0"),
        DeclareLaunchArgument("world_file", default_value=""),
        DeclareLaunchArgument("output_file", default_value="n2_4c_runtime/base.json"),
        DeclareLaunchArgument("runtime_file", default_value="n2_4c_runtime/runtime.json"),
        DeclareLaunchArgument("run_label", default_value="unknown"),
        DeclareLaunchArgument("current_stack_manifest", default_value=""),
        DeclareLaunchArgument("current_stack_manifest_sha256", default_value=""),
        OpaqueFunction(function=_build),
    ])
