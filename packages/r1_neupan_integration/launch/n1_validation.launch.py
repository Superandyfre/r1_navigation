"""Run the R1 ROS2 + NeuPAN N1 interface validation without motion control."""

import os
import shlex

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


PACKAGE = 'r1_neupan_integration'


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _build(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario not in ('free_space', 'obstacle'):
        raise RuntimeError('scenario must be free_space or obstacle')
    lidar_profile = LaunchConfiguration('lidar_profile').perform(context)
    if lidar_profile not in ('lidar_navigation', 'lidar_smoke'):
        raise RuntimeError('lidar_profile must be lidar_navigation or lidar_smoke')
    direct_scan = _as_bool(LaunchConfiguration('direct_scan').perform(context))
    lidar_mount = LaunchConfiguration('lidar_mount').perform(context)
    if lidar_mount not in ('base_link', 'lidar_link'):
        raise RuntimeError('lidar_mount must be base_link or lidar_link')
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    fixed_path_topic = LaunchConfiguration('fixed_path_topic').perform(context)
    control_frequency = float(LaunchConfiguration('control_frequency').perform(context))
    latest_scan_only = _as_bool(LaunchConfiguration('latest_scan_only').perform(context))
    enable_visualization = _as_bool(
        LaunchConfiguration('enable_visualization').perform(context)
    )
    scan_downsample = int(LaunchConfiguration('scan_downsample').perform(context))
    if scan_downsample <= 0:
        raise RuntimeError('scan_downsample must be a positive integer')
    planner_config_file = LaunchConfiguration('planner_config_file').perform(context)
    planner_receding = int(LaunchConfiguration('planner_receding').perform(context))
    planner_iter_num = int(LaunchConfiguration('planner_iter_num').perform(context))
    planner_dune_max_num = int(LaunchConfiguration('planner_dune_max_num').perform(context))
    planner_nrmp_max_num = int(LaunchConfiguration('planner_nrmp_max_num').perform(context))
    planner_iter_threshold = float(
        LaunchConfiguration('planner_iter_threshold').perform(context)
    )
    navigation_mode = LaunchConfiguration('navigation_mode').perform(context)
    if navigation_mode not in ('cyclic', 'terminal'):
        raise RuntimeError('navigation_mode must be cyclic or terminal')
    metrics_file = LaunchConfiguration('metrics_file').perform(context)
    metrics_csv_file = LaunchConfiguration('metrics_csv_file').perform(context)
    input_capture_file = LaunchConfiguration('input_capture_file').perform(context)
    n110_snapshot_dir = LaunchConfiguration('n110_snapshot_dir').perform(context)
    n110_normal_max_ms = float(LaunchConfiguration('n110_normal_max_ms').perform(context))
    n110_medium_min_ms = float(LaunchConfiguration('n110_medium_min_ms').perform(context))
    n110_medium_max_ms = float(LaunchConfiguration('n110_medium_max_ms').perform(context))
    n110_spike_min_ms = float(LaunchConfiguration('n110_spike_min_ms').perform(context))
    n111_snapshot_dir = LaunchConfiguration('n111_snapshot_dir').perform(context)
    n113_output_dir = LaunchConfiguration('n113_output_dir').perform(context)
    n113_shadow_enabled = _as_bool(
        LaunchConfiguration('n113_shadow_enabled').perform(context)
    )
    n114_output_dir = LaunchConfiguration('n114_output_dir').perform(context)
    n114_warmup_count = int(LaunchConfiguration('n114_warmup_count').perform(context))
    n114_timeout_sec = float(LaunchConfiguration('n114_timeout_sec').perform(context))
    n115_output_dir = LaunchConfiguration('n115_output_dir').perform(context)
    n115_exact_capture_dir = LaunchConfiguration('n115_exact_capture_dir').perform(context)
    n115_exact_capture_max_records = int(
        LaunchConfiguration('n115_exact_capture_max_records').perform(context)
    )
    n115_capture_scene = LaunchConfiguration('n115_capture_scene').perform(context)
    n115_capture_metadata = LaunchConfiguration('n115_capture_metadata').perform(context)
    n115_warmup_count = int(LaunchConfiguration('n115_warmup_count').perform(context))
    n115_timeout_sec = float(LaunchConfiguration('n115_timeout_sec').perform(context))
    n116_output_dir = LaunchConfiguration('n116_output_dir').perform(context)
    n116_warmup_count = int(LaunchConfiguration('n116_warmup_count').perform(context))
    n116_timeout_sec = float(LaunchConfiguration('n116_timeout_sec').perform(context))
    n116_fault_mode = LaunchConfiguration('n116_fault_mode').perform(context)
    n116_fault_request_id = int(LaunchConfiguration('n116_fault_request_id').perform(context))
    n116_fault_sleep_sec = float(LaunchConfiguration('n116_fault_sleep_sec').perform(context))
    if n116_output_dir:
        n115_output_dir = n116_output_dir
        n115_warmup_count = n116_warmup_count
        n115_timeout_sec = n116_timeout_sec
    torch_num_threads = int(LaunchConfiguration('torch_num_threads').perform(context))
    torch_num_interop_threads = int(
        LaunchConfiguration('torch_num_interop_threads').perform(context)
    )
    thread_env_policy = LaunchConfiguration('thread_env_policy').perform(context)
    runtime_report_file = LaunchConfiguration('runtime_report_file').perform(context)
    if torch_num_threads < 0 or torch_num_interop_threads < 0:
        raise RuntimeError('Torch thread values must be >= 0; zero means unset')
    if thread_env_policy not in ('inherit', 'match_intra'):
        raise RuntimeError('thread_env_policy must be inherit or match_intra')
    if thread_env_policy == 'match_intra' and torch_num_threads == 0:
        raise RuntimeError('thread_env_policy=match_intra requires torch_num_threads > 0')

    integration_share = get_package_share_directory(PACKAGE)
    gazebo_share = get_package_share_directory('r1_gazebo')
    preprocess_share = get_package_share_directory('r1_lidar_preprocess')
    requested_world = LaunchConfiguration('world_file').perform(context)
    if requested_world:
        world_file = os.path.abspath(requested_world)
    elif scenario == 'free_space':
        world_file = os.path.join(integration_share, 'worlds', 'free_space.sdf')
    else:
        world_file = os.path.join(gazebo_share, 'worlds', 'navigation_test.sdf')
    if not os.path.isfile(world_file):
        raise RuntimeError(f'Gazebo world does not exist: {world_file}')
    config_dir = os.path.join(integration_share, 'config')
    python_executable = LaunchConfiguration('python_executable').perform(context)
    if not os.path.isfile(python_executable):
        raise RuntimeError(
            f'N1 Python executable does not exist: {python_executable}. '
            'Set python_executable:=/path/to/neupan_ros_jazzy/bin/python.'
        )
    cleanup_executable = os.path.join(
        get_package_prefix(PACKAGE), 'lib', PACKAGE, 'cleanup_n1_world'
    )
    runtime_prefix = [
        python_executable,
        '-m',
        'r1_neupan_integration.torch_runtime_bootstrap',
        '--torch-num-threads', str(torch_num_threads),
        '--torch-num-interop-threads', str(torch_num_interop_threads),
        '--thread-env-policy', thread_env_policy,
        '--runtime-report-file', runtime_report_file,
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'r1_nav_sim.launch.py')
        ),
        launch_arguments={
            'headless': 'true',
            'world': world_file,
            'lidar_profile': lidar_profile,
            'lidar_source': 'scan' if direct_scan else 'points',
            'lidar_mount': lidar_mount,
            'rviz': 'false',
        }.items(),
    )
    preprocess = [] if direct_scan else [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(preprocess_share, 'launch', 'pointcloud_to_scan.launch.py')
        ),
        launch_arguments={
            'profile': 'lidar_navigation',
            'input_topic': '/lidar/points',
            'output_topic': '/scan_neupan',
            'use_sim_time': 'true',
        }.items(),
    )]
    neupan = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        # ROS console entry points are generated with /usr/bin/python3.  The
        # wrapper must instead run in the isolated N1 venv so it can import
        # the frozen NeuPAN core without changing system ROS Python.
        # Apply the opt-in R1 runtime policy in this interpreter before the
        # generic node imports NeuPAN.  Zero values preserve the node's
        # ordinary default behaviour.
        prefix=shlex.join(runtime_prefix),
        parameters=[
            os.path.join(config_dir, 'robot.yaml'),
            {
                'robot_config_dir': config_dir,
                'scan_topic': scan_topic,
                'control_frequency': control_frequency,
                'latest_scan_only': latest_scan_only,
                'enable_visualization': enable_visualization,
                'enable_dune_markers': enable_visualization,
                'enable_nrmp_markers': enable_visualization,
                'enable_robot_marker': enable_visualization,
                'scan_downsample': scan_downsample,
                'navigation_mode': navigation_mode,
                'planner_config_file': planner_config_file,
                'planner_receding_override': planner_receding,
                'planner_iter_num_override': planner_iter_num,
                'planner_dune_max_num_override': planner_dune_max_num,
                'planner_nrmp_max_num_override': planner_nrmp_max_num,
                'planner_iter_threshold_override': planner_iter_threshold,
                'input_capture_file': input_capture_file,
                'n110_snapshot_dir': n110_snapshot_dir,
                'n110_normal_max_ms': n110_normal_max_ms,
                'n110_medium_min_ms': n110_medium_min_ms,
                'n110_medium_max_ms': n110_medium_max_ms,
                'n110_spike_min_ms': n110_spike_min_ms,
                'n111_snapshot_dir': n111_snapshot_dir,
                'n113_output_dir': n113_output_dir,
                'n113_shadow_enabled': n113_shadow_enabled,
                'n114_output_dir': n114_output_dir,
                'n114_warmup_count': n114_warmup_count,
                'n114_timeout_sec': n114_timeout_sec,
            },
        ],
    )
    if n116_output_dir:
        neupan = Node(
            package=PACKAGE,
            executable='n1_16_node',
            name='n1_16_production_bridge',
            output='screen',
            emulate_tty=True,
            prefix=shlex.join(runtime_prefix),
            parameters=[
                os.path.join(config_dir, 'robot.yaml'),
                {
                    'robot_config_dir': config_dir,
                    'scan_topic': scan_topic,
                    'control_frequency': control_frequency,
                    'latest_scan_only': latest_scan_only,
                    'scan_downsample': scan_downsample,
                    'navigation_mode': navigation_mode,
                    'planner_config_file': planner_config_file,
                    'planner_receding_override': planner_receding,
                    'planner_iter_num_override': planner_iter_num,
                    'planner_dune_max_num_override': planner_dune_max_num,
                    'planner_nrmp_max_num_override': planner_nrmp_max_num,
                    'planner_iter_threshold_override': planner_iter_threshold,
                    'n115_output_dir': n115_output_dir,
                    'n115_exact_capture_dir': n115_exact_capture_dir,
                    'n115_exact_capture_max_records': n115_exact_capture_max_records,
                    'n115_capture_scene': n115_capture_scene,
                    'n115_capture_metadata': n115_capture_metadata,
                    'n115_warmup_count': n115_warmup_count,
                    'n115_timeout_sec': n115_timeout_sec,
                    'n116_fault_mode': n116_fault_mode,
                    'n116_fault_request_id': n116_fault_request_id,
                    'n116_fault_sleep_sec': n116_fault_sleep_sec,
                    'bridge_status_topic': '/neupan_bridge_status',
                    'torch_num_threads': torch_num_threads or 1,
                    'torch_num_interop_threads': torch_num_interop_threads or 24,
                },
            ],
        )
    elif n115_output_dir:
        # N1.15 has a separate ROS I/O executable.  It must not import or
        # construct the planner in this process; the complete planner is
        # created by its long-lived spawn worker.
        neupan = Node(
            package=PACKAGE,
            executable='n1_15_node',
            name='n1_15_whole_core',
            output='screen',
            emulate_tty=True,
            prefix=shlex.join(runtime_prefix),
            parameters=[
                os.path.join(config_dir, 'robot.yaml'),
                {
                    'robot_config_dir': config_dir,
                    'scan_topic': scan_topic,
                    'control_frequency': control_frequency,
                    'latest_scan_only': latest_scan_only,
                    'scan_downsample': scan_downsample,
                    'navigation_mode': navigation_mode,
                    'planner_config_file': planner_config_file,
                    'planner_receding_override': planner_receding,
                    'planner_iter_num_override': planner_iter_num,
                    'planner_dune_max_num_override': planner_dune_max_num,
                    'planner_nrmp_max_num_override': planner_nrmp_max_num,
                    'planner_iter_threshold_override': planner_iter_threshold,
                    'n115_output_dir': n115_output_dir,
                    'n115_exact_capture_dir': n115_exact_capture_dir,
                    'n115_exact_capture_max_records': n115_exact_capture_max_records,
                    'n115_capture_scene': n115_capture_scene,
                    'n115_capture_metadata': n115_capture_metadata,
                    'n115_warmup_count': n115_warmup_count,
                    'n115_timeout_sec': n115_timeout_sec,
                    'torch_num_threads': torch_num_threads or 1,
                    'torch_num_interop_threads': torch_num_interop_threads or 24,
                },
            ],
        )
    path = Node(
        package=PACKAGE,
        executable='fixed_path_publisher',
        name='r1_n1_fixed_path',
        output='screen',
        # N2-PRE supplies its own scenario path.  The legacy publisher stays
        # available for N1 but can be moved to an inert topic by the caller.
        parameters=[{'use_sim_time': True, 'path_topic': fixed_path_topic}],
    )
    metrics = Node(
        package=PACKAGE,
        executable='n1_metrics',
        name='r1_n1_metrics',
        output='screen',
        parameters=[
            {'duration_sec': float(LaunchConfiguration('duration_sec').perform(context))},
            {'warmup_sec': float(LaunchConfiguration('warmup_sec').perform(context))},
            {'warmup_timing_messages': int(LaunchConfiguration('warmup_timing_messages').perform(context))},
            {
                'output_file': metrics_file,
                'timing_csv_file': metrics_csv_file,
                'scenario': scenario,
                'control_frequency': control_frequency,
                'latest_scan_only': latest_scan_only,
                'enable_visualization': enable_visualization,
                'scan_downsample': scan_downsample,
                'planner_config_file': planner_config_file,
                'planner_receding': planner_receding,
                'planner_iter_num': planner_iter_num,
                'planner_dune_max_num': planner_dune_max_num,
                'planner_nrmp_max_num': planner_nrmp_max_num,
                'planner_iter_threshold': planner_iter_threshold,
                'torch_num_threads': torch_num_threads,
                'torch_num_interop_threads': torch_num_interop_threads,
                'thread_env_policy': thread_env_policy,
                'runtime_report_file': runtime_report_file,
                'bridge_status_topic': '/neupan_bridge_status',
            },
            {'scan_topic': scan_topic},
        ],
        on_exit=Shutdown(),
    )
    # The nested ros_gz_sim launch normally owns its server process.  Keep a
    # world-scoped fallback for interrupted runs where a gz child can outlive
    # the parent launch process.  The helper matches only this exact world.
    cleanup_started = [False]

    def cleanup_once(_context):
        if cleanup_started[0]:
            return []
        cleanup_started[0] = True
        return [
            ExecuteProcess(
                cmd=[cleanup_executable, world_file],
                output='screen',
            ),
        ]

    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=cleanup_once)],
        )
    )
    return [gazebo, *preprocess, neupan, path, metrics, cleanup]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='free_space',
            description='N1 case: free_space or obstacle',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Optional explicit Gazebo Harmonic SDF world path',
        ),
        DeclareLaunchArgument(
            'duration_sec',
            default_value='30.0',
            description='Wall-clock capture duration after launch',
        ),
        DeclareLaunchArgument(
            'warmup_sec',
            default_value='0.0',
            description='Startup warmup excluded from N1 metrics',
        ),
        DeclareLaunchArgument(
            'warmup_timing_messages',
            default_value='0',
            description='Require this many post-time-warmup planner timing cycles before measurement',
        ),
        DeclareLaunchArgument(
            'lidar_profile',
            default_value='lidar_navigation',
            description='R1 lidar profile passed to the existing Gazebo stack',
        ),
        DeclareLaunchArgument(
            'direct_scan',
            default_value='false',
            description='Use the Gazebo ray LaserScan directly without PointCloud preprocessing',
        ),
        DeclareLaunchArgument(
            'lidar_mount',
            default_value='base_link',
            description='Gazebo LiDAR mount link passed to r1_gazebo',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_neupan',
            description='LaserScan topic consumed by wrapper and metrics',
        ),
        DeclareLaunchArgument(
            'fixed_path_topic',
            default_value='/r1_test_path',
            description='Topic used by the legacy N1 fixed-path publisher',
        ),
        DeclareLaunchArgument(
            'metrics_file',
            default_value='n1_results/free_space.json',
            description='JSON metrics output path',
        ),
        DeclareLaunchArgument(
            'metrics_csv_file',
            default_value='n1_results/free_space.timing.csv',
            description='Per-cycle timing CSV output path',
        ),
        DeclareLaunchArgument(
            'input_capture_file',
            default_value='',
            description='Optional one-shot N1 input snapshot (.npz) for core A/B',
        ),
        DeclareLaunchArgument(
            'n110_snapshot_dir',
            default_value='',
            description='N1.10 directory for selected online replay snapshots',
        ),
        DeclareLaunchArgument('n110_normal_max_ms', default_value='20.0'),
        DeclareLaunchArgument('n110_medium_min_ms', default_value='50.0'),
        DeclareLaunchArgument('n110_medium_max_ms', default_value='200.0'),
        DeclareLaunchArgument('n110_spike_min_ms', default_value='300.0'),
        DeclareLaunchArgument(
            'n111_snapshot_dir',
            default_value='',
            description='Optional N1.11 canonical solver evidence directory',
        ),
        DeclareLaunchArgument(
            'n113_output_dir',
            default_value='',
            description='N1.13 in-process online CvxpyLayer shadow evidence directory',
        ),
        DeclareLaunchArgument(
            'n113_shadow_enabled',
            default_value='true',
            description='Run post-primary N1.13 shadows; false records Level-0 only',
        ),
        DeclareLaunchArgument(
            'n114_output_dir',
            default_value='',
            description='N1.14 dedicated solver-process evidence directory; empty keeps in-process solver',
        ),
        DeclareLaunchArgument(
            'n114_warmup_count',
            default_value='30',
            description='N1.14 fixed-parameter worker warmup count before formal timing',
        ),
        DeclareLaunchArgument(
            'n114_timeout_sec',
            default_value='30.0',
            description='N1.14 per-request worker timeout',
        ),
        DeclareLaunchArgument(
            'n115_output_dir',
            default_value='',
            description='N1.15 whole-core worker evidence directory; empty keeps the ordinary N1 node',
        ),
        DeclareLaunchArgument(
            'n115_exact_capture_dir',
            default_value='',
            description='N2.4B exact worker-state capture directory; empty disables capture',
        ),
        DeclareLaunchArgument(
            'n115_exact_capture_max_records',
            default_value='512',
            description='Maximum formal solve records retained by the exact capture ring',
        ),
        DeclareLaunchArgument(
            'n115_capture_scene',
            default_value='',
            description='Scene label stored in exact capture metadata',
        ),
        DeclareLaunchArgument(
            'n115_capture_metadata',
            default_value='{}',
            description='JSON object stored in exact capture metadata',
        ),
        DeclareLaunchArgument(
            'n115_warmup_count',
            default_value='30',
            description='N1.15 full-planner warmup count before READY',
        ),
        DeclareLaunchArgument(
            'n115_timeout_sec',
            default_value='30.0',
            description='N1.15 per-request worker timeout',
        ),
        DeclareLaunchArgument(
            'n116_output_dir',
            default_value='',
            description='N1.16 production bridge evidence directory; enables n1_16_node',
        ),
        DeclareLaunchArgument(
            'n116_warmup_count',
            default_value='30',
            description='N1.16 full-planner warmup count before READY',
        ),
        DeclareLaunchArgument(
            'n116_timeout_sec',
            default_value='0.25',
            description='N1.16 bounded per-request timeout in seconds',
        ),
        DeclareLaunchArgument(
            'n116_fault_mode',
            default_value='',
            description='N1.16 isolated fault injection mode for one request',
        ),
        DeclareLaunchArgument(
            'n116_fault_request_id',
            default_value='-1',
            description='N1.16 request id receiving the selected fault',
        ),
        DeclareLaunchArgument(
            'n116_fault_sleep_sec',
            default_value='1.0',
            description='N1.16 injected worker sleep duration',
        ),
        DeclareLaunchArgument(
            'torch_num_threads',
            default_value='0',
            description='R1-only Torch intra-op threads; zero leaves the generic default unchanged',
        ),
        DeclareLaunchArgument(
            'torch_num_interop_threads',
            default_value='0',
            description='R1-only Torch inter-op threads; zero leaves the generic default unchanged',
        ),
        DeclareLaunchArgument(
            'thread_env_policy',
            default_value='inherit',
            description='inherit records OMP/MKL variables; match_intra sets this process only',
        ),
        DeclareLaunchArgument(
            'runtime_report_file',
            default_value='',
            description='JSON written by the R1 Torch runtime bootstrap',
        ),
        DeclareLaunchArgument(
            'control_frequency',
            default_value='50.0',
            description='NeuPAN planner timer frequency for this experiment',
        ),
        DeclareLaunchArgument(
            'latest_scan_only',
            default_value='true',
            description='Only plan when a newer scan has arrived (latest-data single-flight)',
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable all wrapper visualization markers',
        ),
        DeclareLaunchArgument(
            'scan_downsample',
            default_value='1',
            description='Wrapper-side LaserScan decimation factor',
        ),
        DeclareLaunchArgument(
            'navigation_mode',
            default_value='cyclic',
            description='R1 path semantics: cyclic preserves loop behavior; terminal stops at the goal',
        ),
        DeclareLaunchArgument(
            'planner_config_file',
            default_value='planner.yaml',
            description='Planner YAML relative to the integration config directory',
        ),
        DeclareLaunchArgument(
            'planner_receding',
            default_value='0',
            description='Optional receding override; zero keeps YAML value',
        ),
        DeclareLaunchArgument(
            'planner_iter_num',
            default_value='0',
            description='Optional PAN iter_num override; zero keeps YAML value',
        ),
        DeclareLaunchArgument(
            'planner_dune_max_num',
            default_value='0',
            description='Optional PAN dune_max_num override; zero keeps YAML value',
        ),
        DeclareLaunchArgument(
            'planner_nrmp_max_num',
            default_value='0',
            description='Optional PAN nrmp_max_num override; zero keeps YAML value',
        ),
        DeclareLaunchArgument(
            'planner_iter_threshold',
            default_value='-1.0',
            description='Optional PAN iter_threshold override; negative keeps YAML value',
        ),
        DeclareLaunchArgument(
            'python_executable',
            default_value=os.environ.get(
                'NEUPAN_ROS_PYTHON',
                '/home/pinggu/venvs/neupan_ros_jazzy/bin/python',
            ),
            description='Python interpreter for the NeuPAN ROS wrapper',
        ),
        OpaqueFunction(function=_build),
    ])
