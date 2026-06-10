import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio_sam_sc_qn2')

    default_config = os.path.join(pkg_share, 'config', 'mid360.yaml')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    node_name = LaunchConfiguration('node_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
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
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation time.'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='fast_lio_sam_sc_qn2_node',
            description='Back-end node name.'
        ),
        OpaqueFunction(function=_livox_driver_launch),
        Node(
            package='fast_lio_sam_sc_qn2',
            executable='fast_lio_sam_sc_qn2_node',
            name=node_name,
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        )
    ])
