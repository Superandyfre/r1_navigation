"""Navigation-only recovery boundary: no localization, map, arms or drivers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share=FindPackageShare('navigation_neupan_bridge')
    declarations=[DeclareLaunchArgument(name,default_value=value) for name,value in
                  [('calibration_record',''),('scan_source','dual'),('use_mesh_self_filter','true'),
                   ('mesh_projection_margin_m','0.06'),('command_profile','smoothed'),
                   ('cmd_vel_out_topic','/cmd_vel'),('runtime_auto_arm','true')]]
    def include(file,names,extra=None):
        arguments={name:LaunchConfiguration(name) for name in names}
        arguments.update(extra or {})
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share,'launch',file])),launch_arguments=arguments.items())
    return LaunchDescription(declarations+[
        include('navigation_neupan.launch.py',['command_profile','runtime_auto_arm'],{'mode':'hw1'}),
        include('r1_neupan_hw1_motion.launch.py',['calibration_record','scan_source','use_mesh_self_filter',
            'mesh_projection_margin_m','command_profile','cmd_vel_out_topic'],{'collision_check_enabled':'false'}),
    ])
