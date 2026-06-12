import os
import shlex

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context)


def _workspace_root():
    package_prefix = get_package_prefix('fast_lio_sam_sc_qn2')
    install_root = os.path.dirname(package_prefix)
    if os.path.basename(install_root) == 'install':
        return os.path.dirname(install_root)
    return os.getcwd()


def _resolve_workspace_path(context, launch_name):
    path = _launch_value(context, launch_name)
    if os.path.isabs(path):
        return path
    return os.path.join(_workspace_root(), path)


def _map_storage_paths(context):
    map_history_root = os.path.normpath(_resolve_workspace_path(context, 'map_history_root'))
    save_root = os.path.dirname(map_history_root)
    maps_directory_name = os.path.basename(map_history_root)
    return map_history_root, save_root, maps_directory_name


def _web_mapping_launch(context, *args, **kwargs):
    if not _as_bool(LaunchConfiguration('start_web_ui').perform(context)):
        return []

    map_history_root, _, _ = _map_storage_paths(context)
    web_mapping_share = get_package_share_directory('web_mapping')
    web_mapping_launch = os.path.join(web_mapping_share, 'launch', 'web_mapping.launch.py')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_mapping_launch),
            launch_arguments={
                'host': LaunchConfiguration('web_host'),
                'port': LaunchConfiguration('web_port'),
                'livox_custom_topic': LaunchConfiguration('livox_lidar_topic'),
                'optimized_cloud_topic': '/web_mapping/current_frame',
                'max_raw_points_per_cloud': LaunchConfiguration('web_raw_points'),
                'max_optimized_points_per_cloud': LaunchConfiguration('web_current_points'),
                'max_map_points_per_cloud': LaunchConfiguration('web_map_points'),
                'min_cloud_interval_sec': LaunchConfiguration('web_cloud_interval'),
                'min_map_interval_sec': LaunchConfiguration('web_map_interval'),
                'map_history_root': map_history_root,
                'map_history_limit': LaunchConfiguration('map_history_limit'),
            }.items(),
        )
    ]


def _broker_launch(context, *args, **kwargs):
    workspace_root = _workspace_root()
    map_history_root, save_root, maps_directory_name = _map_storage_paths(context)
    mapping_launch = [
        'ros2',
        'launch',
        'fast_lio_sam_sc_qn2',
        'mid360_mapping.launch.py',
        'start_web_broker:=false',
        f"start_livox_driver:={LaunchConfiguration('start_livox_driver').perform(context)}",
        f"use_mid360s:={_launch_value(context, 'use_mid360s')}",
        f"start_fast_lio_frontend:={_launch_value(context, 'start_fast_lio_frontend')}",
        f"backend_config_file:={_launch_value(context, 'backend_config_file')}",
        f"use_sim_time:={_launch_value(context, 'use_sim_time')}",
        f"save_trigger_topic:={_launch_value(context, 'save_trigger_topic')}",
        f"map_history_root:={map_history_root}",
        f"save_root:={save_root}",
        f"maps_directory_name:={maps_directory_name}",
        "fast_lio_current_frame_topic:=/cloud_registered_1",
        "fast_lio_global_map_topic:=corrected_map",
        "{session_arg}",
    ]
    start_command = ' '.join(shlex.quote(part) for part in mapping_launch)
    return [
        Node(
            package='fast_lio_sam_sc_qn2',
            executable='fast_lio_web_broker.py',
            name='fast_lio_web_broker',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_history_root': map_history_root,
                'save_root': save_root,
                'save_trigger_topic': LaunchConfiguration('save_trigger_topic'),
                'start_command': start_command,
                'process_cwd': workspace_root,
                'fast_lio_current_frame_topic': LaunchConfiguration('fast_lio_current_frame_topic'),
                'fast_lio_global_map_topic': LaunchConfiguration('fast_lio_global_map_topic'),
                'fast_lio_map_save_service': LaunchConfiguration('fast_lio_map_save_service'),
                'fast_lio_pose_topic': LaunchConfiguration('fast_lio_pose_topic'),
                'fast_lio_raw_path_topic': LaunchConfiguration('fast_lio_raw_path_topic'),
                'fast_lio_optimized_path_topic': LaunchConfiguration('fast_lio_optimized_path_topic'),
                'fast_lio_imu_topic': LaunchConfiguration('fast_lio_imu_topic'),
                'process_exit_timeout_sec': LaunchConfiguration('process_exit_timeout_sec'),
                'output_quiet_timeout_sec': LaunchConfiguration('output_quiet_timeout_sec'),
                'stop_wait_topics': LaunchConfiguration('stop_wait_topics'),
            }],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio_sam_sc_qn2')
    default_config = os.path.join(pkg_share, 'config', 'mid360.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend_config_file',
            default_value=default_config,
            description='MID360-only back-end parameter file used by Web start.',
        ),
        DeclareLaunchArgument(
            'start_livox_driver',
            default_value='true',
            description='Start Livox driver when Web sends start.',
        ),
        DeclareLaunchArgument(
            'use_mid360s',
            default_value='true',
            description='Use the MID360s driver launch when Web sends start.',
        ),
        DeclareLaunchArgument(
            'start_fast_lio_frontend',
            default_value='true',
            description='Start FAST-LIO front-end when Web sends start.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation time.',
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='ROS domain used by the Web Mapping stack to avoid topic collisions.',
        ),
        DeclareLaunchArgument(
            'ros_localhost_only',
            default_value='1',
            description='Limit the Web Mapping ROS graph to localhost by default.',
        ),
        DeclareLaunchArgument(
            'map_history_root',
            default_value='maps',
            description='Map history directory exposed to web_mapping.',
        ),
        DeclareLaunchArgument(
            'map_history_limit',
            default_value='20',
            description='Maximum saved map sessions shown in web_mapping.',
        ),
        DeclareLaunchArgument(
            'save_trigger_topic',
            default_value='save_dir',
            description='Topic used by the back-end to trigger map saving.',
        ),
        DeclareLaunchArgument('livox_lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('fast_lio_current_frame_topic', default_value='/cloud_registered_1'),
        DeclareLaunchArgument('fast_lio_global_map_topic', default_value='corrected_map'),
        DeclareLaunchArgument('fast_lio_map_save_service', default_value='map_save'),
        DeclareLaunchArgument('fast_lio_pose_topic', default_value='pose_stamped'),
        DeclareLaunchArgument('fast_lio_raw_path_topic', default_value='ori_path'),
        DeclareLaunchArgument('fast_lio_optimized_path_topic', default_value='corrected_path'),
        DeclareLaunchArgument('fast_lio_imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'process_exit_timeout_sec',
            default_value='3.0',
            description='Time to wait for mapping child processes to exit before forcing shutdown.',
        ),
        DeclareLaunchArgument(
            'output_quiet_timeout_sec',
            default_value='2.5',
            description='Time to wait for mapping output topics to have no publishers before reporting stopped.',
        ),
        DeclareLaunchArgument(
            'stop_wait_topics',
            default_value='/livox/lidar,/livox/imu,/cloud_registered_1,/Odometry_loc,corrected_map',
            description='Comma-separated output topics that must have no publishers before stop completes.',
        ),
        DeclareLaunchArgument(
            'start_web_ui',
            default_value='true',
            description='Start web_mapping bridge together with this broker.',
        ),
        DeclareLaunchArgument(
            'web_host',
            default_value='0.0.0.0',
            description='Web Mapping host.',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8765',
            description='Web Mapping port.',
        ),
        DeclareLaunchArgument(
            'web_raw_points',
            default_value='5000',
            description='Maximum Livox raw points sent to the browser per frame.',
        ),
        DeclareLaunchArgument(
            'web_current_points',
            default_value='50000',
            description='Maximum current-frame points sent to the browser per frame.',
        ),
        DeclareLaunchArgument(
            'web_map_points',
            default_value='50000',
            description='Maximum global-map points sent to the browser per frame.',
        ),
        DeclareLaunchArgument(
            'web_cloud_interval',
            default_value='0.12',
            description='Minimum browser cloud send interval for raw/current streams.',
        ),
        DeclareLaunchArgument(
            'web_map_interval',
            default_value='0.5',
            description='Minimum browser cloud send interval for global map snapshots.',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('ros_localhost_only')),
        OpaqueFunction(function=_broker_launch),
        OpaqueFunction(function=_web_mapping_launch),
    ])
