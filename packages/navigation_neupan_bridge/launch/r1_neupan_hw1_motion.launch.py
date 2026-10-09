"""NeuPAN HW1 motion chain for Humble.

The chassis driver remains externally owned. NeuPAN publishes through the
Nav2 controller, velocity smoother, and final scan-based collision guard.
"""

import json
import os
import shlex

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_lidar_calibration(calibration):
    """Reject an HW1 record that disagrees with the robot TF calibration."""
    calibration_path = os.path.join(
        get_package_share_directory("r1_description"),
        "config", "r1_lidar_extrinsics.yaml")
    try:
        with open(calibration_path, encoding="utf-8") as calibration_file:
            canonical = yaml.safe_load(calibration_file)["base_link_to_lidar_link"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError("canonical R1 lidar calibration is unreadable: " + str(error)) from error

    record = calibration["lidar_to_base"]
    fields = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
    mismatches = [
        field for field in fields
        if field not in record or abs(float(record[field]) - float(canonical[field])) > 1e-4
    ]
    if mismatches:
        raise RuntimeError(
            "HW1 lidar calibration differs from r1_description for: " + ", ".join(mismatches))


def _build(context):
    bridge_share = get_package_share_directory("navigation_neupan_bridge")
    integration_share = get_package_share_directory("r1_neupan_integration")
    guard_share = get_package_share_directory("r1_collision_guard")
    python_executable = LaunchConfiguration("python_executable").perform(context)
    input_cloud = LaunchConfiguration("input_cloud").perform(context)
    scan_source = LaunchConfiguration("scan_source").perform(context)
    use_mesh_self_filter = LaunchConfiguration("use_mesh_self_filter").perform(context)
    mesh_projection_margin_m = float(
        LaunchConfiguration("mesh_projection_margin_m").perform(context))
    planner_footprint_margin_m = float(
        LaunchConfiguration("planner_footprint_margin_m").perform(context))
    command_profile = LaunchConfiguration("command_profile").perform(context)
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic").perform(context)
    calibration_record = LaunchConfiguration("calibration_record").perform(context)
    if not os.path.isfile(python_executable):
        raise RuntimeError(
            "NeuPAN Humble venv is unavailable: " + python_executable +
            ". Create it and install the vendored NeuPAN core before HW1."
        )
    if not calibration_record or not os.path.isfile(calibration_record):
        raise RuntimeError(
            "HW1 calibration record is required. Supply calibration_record:=/path/to/r1_hw1_calibration.json"
        )
    try:
        with open(calibration_record, encoding="utf-8") as record_file:
            calibration = json.load(record_file)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("HW1 calibration record is unreadable: " + str(error)) from error
    if calibration.get("schema") != "r1_hw1_calibration_record_v1" or not calibration.get("approved_for_hw1"):
        raise RuntimeError("HW1 calibration record is not approved for HW1")
    if scan_source not in {"livox_only", "dual"}:
        raise RuntimeError("scan_source must be livox_only or dual")
    if use_mesh_self_filter not in {"false", "true"}:
        raise RuntimeError("use_mesh_self_filter must be true or false")
    if not 0.0 <= mesh_projection_margin_m <= 0.10:
        raise RuntimeError("mesh_projection_margin_m must be within [0.0, 0.10]")
    if not 0.0 <= planner_footprint_margin_m <= 0.30:
        raise RuntimeError("planner_footprint_margin_m must be within [0.0, 0.30]")
    if command_profile not in {"direct", "smoothed"}:
        raise RuntimeError("command_profile must be direct or smoothed")
    if not cmd_vel_out_topic.startswith("/"):
        raise RuntimeError("cmd_vel_out_topic must be an absolute ROS topic")
    required_sections = {
        "footprint", "drive", "lidar_to_base", "scan", "approval",
    }
    missing_sections = sorted(required_sections.difference(calibration))
    if missing_sections:
        raise RuntimeError(
            "HW1 calibration record is incomplete: " + ", ".join(missing_sections)
        )
    _validate_lidar_calibration(calibration)

    config_dir = os.path.join(bridge_share, "config")
    footprint = calibration["footprint"]
    polygon_xy = footprint.get("polygon_xy")
    if polygon_xy:
        footprint_points = [coordinate for point in polygon_xy for coordinate in point]
    else:
        # The approved HW1 record contains measured length and width. This
        # conservative rectangle is replaced automatically once polygon_xy is
        # surveyed and added to the calibration record.
        half_length = float(footprint["length_m"]) / 2.0
        half_width = float(footprint["width_m"]) / 2.0
        footprint_points = [
            half_length, half_width, half_length, -half_width,
            -half_length, -half_width, -half_length, half_width,
        ]
    # With no arm state stream, keep a measured-reach envelope rather than
    # dropping the arms from collision geometry. This keeps navigation alive
    # while conservatively rejecting spaces the unknown arm pose could occupy.
    arm_data_fallback_points = [
        1.05, 1.30, 1.05, -1.30,
        -1.05, -1.30, -1.05, 1.30,
    ]
    runtime_prefix = shlex.join([
        python_executable,
        "-m",
        "r1_neupan_integration.torch_runtime_bootstrap",
        # The planner itself is sub-30 ms.  Leaving BLAS/OpenMP at their host
        # defaults creates 28-thread pools in both bridge and worker, starving
        # the 10 Hz command publisher behind localization and mesh projection.
        "--torch-num-threads", "4",
        "--torch-num-interop-threads", "2",
        "--thread-env-policy", "match_intra",
        "--runtime-report-file",
        LaunchConfiguration("runtime_report_file").perform(context),
    ])

    livox_scan_topic = "/scan_livox" if scan_source == "dual" else "/scan_neupan"
    navigation_scan_topic = (
        "/scan_neupan_mesh_shadow" if use_mesh_self_filter == "true" else "/scan_neupan")
    # The localization node withholds /deskew_cloud until its initial pose is
    # valid.  A dual-scan collision guard must still receive the front scan
    # during that bootstrap period, so use Livox's live source directly.
    preprocess_input_cloud = "/lidar/pointcloud" if scan_source == "dual" else input_cloud
    preprocess = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="neupan_pointcloud_to_scan",
        output="screen",
        remappings=[("cloud_in", preprocess_input_cloud), ("scan", livox_scan_topic)],
        parameters=[os.path.join(config_dir, "lidar_r1_hw1.yaml"), {"use_sim_time": False}],
    )
    arm_footprint = Node(
        package="navigation_neupan_bridge",
        executable="r1_arm_footprint",
        name="r1_arm_footprint",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "base_frame": "base_link",
            "base_footprint_points": footprint_points,
            "fallback_footprint_points": arm_data_fallback_points,
            "joint_state_timeout_sec": 0.50,
            "update_frequency": 10.0,
            # Exact mesh projection is recomputed only after a material joint
            # change, then republished at 4 Hz for downstream freshness.
            "mesh_projection_frequency": 4.0,
            "mesh_reprojection_joint_delta_rad": 0.01,
            "link_radius_m": 0.12,
            "tool_radius_m": 0.16,
            "projection_margin_m": 0.03,
            # Mesh-only tolerance for surveyed LiDAR-to-arm edge mismatch.
            # It does not change the navigation or collision footprint.
            "mesh_projection_margin_m": mesh_projection_margin_m,
            # Costmaps and the global planner receive the arm hull padded by
            # this margin; SMAC then rejects corridors the live arm posture
            # cannot clear, matching NeuPAN's arm-aware local feasibility.
            "planner_footprint_margin_m": planner_footprint_margin_m,
        }],
    )
    footprint_sync = Node(
        package="navigation_neupan_bridge",
        executable="r1_dynamic_footprint_sync",
        name="r1_dynamic_footprint_sync",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            # Feed the costmap footprint parameter from the planner-padded
            # hull so the SMAC lattice's per-plan footprint check and the
            # footprint topics always agree with the published envelope.
            "input_topic": "/r1_arm_footprint_planner",
            "base_frame": "base_link",
            "global_costmap_node": "/global_costmap/global_costmap",
            "local_costmap_node": "/local_costmap/local_costmap",
            "update_threshold_m": 0.02,
        }],
    )
    passage_clearance = Node(
        package="navigation_neupan_bridge",
        executable="r1_passage_clearance_diagnostics",
        name="r1_passage_clearance_diagnostics",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "base_frame": "base_link",
            "scan_topic": navigation_scan_topic,
            "path_topic": "/neupan_path",
            "footprint_topic": "/r1_arm_footprint",
            "planning_clearance_m": 0.15,
            "probe_distance_m": 0.45,
            "probe_half_width_m": 0.12,
        }],
    )
    scan_fusion = None
    if scan_source == "dual":
        scan_fusion = Node(
            package="r1_lidar_preprocess",
            executable="scan_fusion_node",
            name="r1_scan_fusion",
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("r1_lidar_preprocess"),
                    "config", "scan_fusion_hw1.yaml"),
                {"use_sim_time": False},
            ],
        )
    worker = Node(
        package="r1_neupan_integration",
        executable="n1_16_node",
        name="n1_16_production_bridge",
        output="screen",
        emulate_tty=True,
        prefix=runtime_prefix,
        parameters=[
            os.path.join(config_dir, "robot_r1_hw1.yaml"),
            {
                "use_sim_time": False,
                "robot_config_dir": config_dir,
                "planner_config_file": "planner_r1_hw1.yaml",
                "dune_checkpoint_file": os.path.join(
                    integration_share, "config", "models", "dune_model_5000.pth"),
                "scan_topic": navigation_scan_topic,
                "plan_input_topic": "/neupan_path",
                "goal_topic": "/neupan_goal",
                "map_frame": "map",
                "base_frame": "base_link",
                "lidar_frame": "base_link",
                "self_filter_enabled": True,
                "dynamic_footprint_enabled": True,
                "dynamic_footprint_topic": "/r1_arm_footprint",
                "dynamic_footprint_timeout_sec": 0.50,
                # The convex live footprint arrives at 10 Hz and remains the
                # immediate motion gate. The exact non-convex mesh projection
                # is costlier and arrives near 4 Hz, so it has its own bounded
                # freshness window for NeuPAN obstacle shaping.
                "dynamic_mesh_components_timeout_sec": 1.50,
                "dynamic_footprint_clearance_m": 0.15,
                "dynamic_mesh_components_enabled": True,
                "dynamic_mesh_components_topic": "/r1_arm_mesh_components",
                "planner_base_front_m": 0.341,
                "planner_base_rear_m": 0.341,
                "planner_base_left_m": 0.266,
                "planner_base_right_m": 0.266,
                "allow_reverse": True,
                "control_frequency": 10.0,
                "latest_scan_only": True,
                "navigation_mode": "terminal",
                "planner_result_topic": "/neupan_planner_result",
                "planner_context_topic": "/neupan_planner_context",
                "bridge_status_topic": "/neupan_bridge_status",
                "torch_num_threads": 4,
                "torch_num_interop_threads": 2,
                "shadow_tf_fault_injection_allowed": False,
                "shadow_force_robot_tf_failure": False,
            },
        ],
    )
    collision_guard = Node(
        package="r1_collision_guard",
        executable="r1_collision_guard_node",
        name="r1_collision_guard",
        output="screen",
        parameters=[
            os.path.join(guard_share, "config", "r1_collision_guard_hw1.yaml"),
            {
                "use_sim_time": False,
                "footprint_points": footprint_points,
                "odom_topic": "/odom",
                "scan_topic": navigation_scan_topic,
                "mesh_self_filter_preapplied": use_mesh_self_filter == "true",
                "cmd_vel_in_topic": (
                    "/neupan_nav2_smoothed_cmd_vel"
                    if command_profile == "smoothed" else "/neupan_nav2_cmd_vel"),
                "source_cmd_vel_topic": (
                    "/neupan_nav2_raw_cmd_vel"
                    if command_profile == "smoothed" else "/neupan_nav2_cmd_vel"),
                "cmd_vel_out_topic": cmd_vel_out_topic,
            },
        ],
    )
    localization_monitor = Node(
        package="navigation_neupan_bridge",
        executable="r1_localization_monitor",
        name="r1_localization_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "map_frame": "map",
            "base_frame": "base_link",
            "odom_topic": "/odom",
            "localization_status_topic": "/indoor_location/status",
            "costmap_service": "/global_costmap/get_costmap",
            "status_topic": "/r1_localization_health",
            "tf_timeout_sec": 1.5,
            "odom_timeout_sec": 1.5,
            "localization_timeout_sec": 1.5,
            "localization_fault_timeout_sec": 3.0,
            "footprint_points": footprint_points,
            "footprint_padding_m": 0.01,
            "require_dynamic_footprint": True,
            # Compare the planner-padded envelope against the costmap's
            # published footprint. Both carry planner_footprint_margin_m, so
            # the sync handshake validates like against like; the raw heartbeat
            # on /r1_arm_footprint stays reserved for the collision guard.
            "dynamic_footprint_topic": "/r1_arm_footprint_planner",
            # This monitor only gates localization readiness. Keep enough
            # margin for the 10 Hz mesh projection during scheduler pauses;
            # the final collision guard retains its independent 0.50 s limit.
            "dynamic_footprint_timeout_sec": 1.50,
        }],
    )
    nodes = [
        preprocess, arm_footprint, footprint_sync, passage_clearance, localization_monitor, collision_guard, worker,
    ]
    if scan_fusion is not None:
        nodes.insert(1, scan_fusion)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_cloud", default_value="/deskew_cloud",
            description="High-sampling localization's deskewed PointCloud2 in base_link."),
        DeclareLaunchArgument(
            "scan_source", default_value="livox_only",
            description="NeuPAN obstacle scan source: livox_only or dual (Livox plus rear BlueSea)."),
        DeclareLaunchArgument(
            "use_mesh_self_filter", default_value="false", choices=["false", "true"],
            description="Use the validated dual-arm mesh self-return filter; disabled by default."),
        DeclareLaunchArgument(
            "mesh_projection_margin_m", default_value="0.06",
            description="Local mesh-only LiDAR self-return edge tolerance in metres."),
        DeclareLaunchArgument(
            "planner_footprint_margin_m", default_value="0.15",
            description=(
                "Lateral margin added to the arm footprint handed to the costmaps and "
                "global planner so SMAC's feasibility model matches NeuPAN's live "
                "arm-aware clearance. The collision-guard heartbeat stays unpadded.")),
        DeclareLaunchArgument(
            "python_executable",
            default_value=os.path.expanduser("~/venvs/neupan_ros_humble/bin/python"),
            description="Dedicated Humble NeuPAN Python environment."),
        DeclareLaunchArgument(
            "calibration_record", default_value="",
            description="Approved JSON record for R1 HW1 footprint, drive, LiDAR, and scan calibration."),
        DeclareLaunchArgument(
            "command_profile", default_value="smoothed", choices=["direct", "smoothed"],
            description="Must match navigation_neupan.launch.py for the entire run."),
        DeclareLaunchArgument(
            "cmd_vel_out_topic", default_value="/cmd_vel",
            description="Collision-guard output; use an isolated topic for no-motion integration tests."),
        DeclareLaunchArgument("runtime_report_file", default_value="neupan_hw1_torch_runtime.json"),
        OpaqueFunction(function=_build),
    ])
