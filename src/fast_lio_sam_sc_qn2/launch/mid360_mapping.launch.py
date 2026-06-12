import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _livox_driver_launch(context, *args, **kwargs):
    if not _as_bool(LaunchConfiguration('start_livox_driver').perform(context)):
        return []

    livox_share = get_package_share_directory('livox_ros_driver2')
    launch_file = 'msg_MID360s_launch.py' if _as_bool(LaunchConfiguration('use_mid360s').perform(context)) else 'msg_MID360_launch.py'
    driver_launch = os.path.join(livox_share, 'launch_ROS2', launch_file)

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch))
    ]


def _fast_lio_frontend_launch(context, *args, **kwargs):
    if not _as_bool(LaunchConfiguration('start_fast_lio_frontend').perform(context)):
        return []

    fast_lio_share = get_package_share_directory('fast_lio')
    fast_lio_launch = os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
    fast_lio_config_path = os.path.join(fast_lio_share, 'config')
    save_root = LaunchConfiguration('save_root').perform(context) or os.getcwd()
    maps_directory_name = LaunchConfiguration('maps_directory_name').perform(context)
    session_name = LaunchConfiguration('session_name').perform(context)
    frontend_map_dir = os.path.join(save_root, maps_directory_name, session_name) if session_name else os.path.join(save_root, maps_directory_name)
    frontend_map_filename = f'{session_name}_map.pcd' if session_name else 'result.pcd'
    frontend_map_path = os.path.join(frontend_map_dir, frontend_map_filename)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                'config_path': fast_lio_config_path,
                'config_file': 'mid360.yaml',
                'rviz': 'false',
                'start_livox_driver': 'false',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_path': frontend_map_path,
                'publish_map': 'true',
            }.items()
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio_sam_sc_qn2')

    default_config = os.path.join(pkg_share, 'config', 'mid360.yaml')

    backend_config_file = LaunchConfiguration('backend_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    node_name = LaunchConfiguration('node_name')
    start_web_broker = LaunchConfiguration('start_web_broker')
    map_history_root = LaunchConfiguration('map_history_root')
    save_root = LaunchConfiguration('save_root')
    maps_directory_name = LaunchConfiguration('maps_directory_name')
    session_name = LaunchConfiguration('session_name')
    save_trigger_topic = LaunchConfiguration('save_trigger_topic')
    fast_lio_current_frame_topic = LaunchConfiguration('fast_lio_current_frame_topic')
    fast_lio_global_map_topic = LaunchConfiguration('fast_lio_global_map_topic')
    fast_lio_pose_topic = LaunchConfiguration('fast_lio_pose_topic')
    fast_lio_raw_path_topic = LaunchConfiguration('fast_lio_raw_path_topic')
    fast_lio_optimized_path_topic = LaunchConfiguration('fast_lio_optimized_path_topic')
    fast_lio_imu_topic = LaunchConfiguration('fast_lio_imu_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend_config_file',
            default_value=default_config,
            description='MID360-only back-end parameter file.'
        ),
        DeclareLaunchArgument(
            'start_livox_driver',
            default_value='false',
            description='Start the official livox_ros_driver2 MID360 launch file.'
        ),
        DeclareLaunchArgument(
            'use_mid360s',
            default_value='false',
            description='Use the official MID360s driver launch instead of MID360.'
        ),
        DeclareLaunchArgument(
            'start_fast_lio_frontend',
            default_value='false',
            description='Start FAST-LIO front-end to publish /Odometry_loc and /cloud_registered_1.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation time.'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='fast_lio_sam_sc_qn2_node',
            description='Back-end node name.'
        ),
        DeclareLaunchArgument(
            'start_web_broker',
            default_value='true',
            description='Start the web_mapping broker adapter for this SLAM back-end.'
        ),
        DeclareLaunchArgument(
            'map_history_root',
            default_value='maps',
            description='Map history directory exposed to web_mapping.'
        ),
        DeclareLaunchArgument(
            'save_root',
            default_value='',
            description='Root directory used by the back-end when saving maps. Empty means current working directory.'
        ),
        DeclareLaunchArgument(
            'maps_directory_name',
            default_value='maps',
            description='Directory name created under save_root for saved map sessions.'
        ),
        DeclareLaunchArgument(
            'session_name',
            default_value='',
            description='Optional fixed map session name for saved results.'
        ),
        DeclareLaunchArgument(
            'save_trigger_topic',
            default_value='save_dir',
            description='Topic used by the back-end to trigger map saving.'
        ),
        DeclareLaunchArgument('fast_lio_current_frame_topic', default_value='/cloud_registered_1'),
        DeclareLaunchArgument('fast_lio_global_map_topic', default_value='corrected_map'),
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
        OpaqueFunction(function=_livox_driver_launch),
        OpaqueFunction(function=_fast_lio_frontend_launch),
        Node(
            package='fast_lio_sam_sc_qn2',
            executable='fast_lio_sam_sc_qn2_node',
            name=node_name,
            output='screen',
            parameters=[backend_config_file, {
                'use_sim_time': use_sim_time,
                'result.save_directory': save_root,
                'result.maps_directory_name': maps_directory_name,
                'result.session_name': session_name,
            }]
        ),
        Node(
            package='fast_lio_sam_sc_qn2',
            executable='fast_lio_web_broker.py',
            name='fast_lio_web_broker',
            output='screen',
            condition=IfCondition(start_web_broker),
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_history_root': map_history_root,
                'save_root': save_root,
                'save_trigger_topic': save_trigger_topic,
                'fast_lio_current_frame_topic': fast_lio_current_frame_topic,
                'fast_lio_global_map_topic': fast_lio_global_map_topic,
                'fast_lio_map_save_service': 'map_save',
                'fast_lio_pose_topic': fast_lio_pose_topic,
                'fast_lio_raw_path_topic': fast_lio_raw_path_topic,
                'fast_lio_optimized_path_topic': fast_lio_optimized_path_topic,
                'fast_lio_imu_topic': fast_lio_imu_topic,
                'process_exit_timeout_sec': LaunchConfiguration('process_exit_timeout_sec'),
                'output_quiet_timeout_sec': LaunchConfiguration('output_quiet_timeout_sec'),
                'stop_wait_topics': LaunchConfiguration('stop_wait_topics'),
            }]
        )
    ])
