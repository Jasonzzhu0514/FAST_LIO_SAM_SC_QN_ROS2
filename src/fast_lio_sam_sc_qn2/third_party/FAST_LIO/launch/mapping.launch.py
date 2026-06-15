import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    livox_package_path = get_package_share_directory('livox_ros_driver2')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')
    livox_launch_path = os.path.join(
        livox_package_path, 'launch_ROS2', 'msg_MID360s_launch.py'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    start_livox_driver = LaunchConfiguration('start_livox_driver')
    map_file_path = LaunchConfiguration('map_file_path')
    publish_map = LaunchConfiguration('publish_map')
    cloud_registered_topic = LaunchConfiguration('cloud_registered_topic')
    cloud_registered_body_topic = LaunchConfiguration('cloud_registered_body_topic')
    cloud_effected_topic = LaunchConfiguration('cloud_effected_topic')
    laser_map_topic = LaunchConfiguration('laser_map_topic')
    odometry_topic = LaunchConfiguration('odometry_topic')
    path_topic = LaunchConfiguration('path_topic')
    map_save_service = LaunchConfiguration('map_save_service')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_start_livox_driver_cmd = DeclareLaunchArgument(
        'start_livox_driver', default_value='true',
        description='Start livox_ros_driver2 for MID360 input'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='Output PCD path for the accumulated FAST-LIO frontend map'
    )
    declare_publish_map_cmd = DeclareLaunchArgument(
        'publish_map', default_value='false',
        description='Publish and accumulate the FAST-LIO frontend map'
    )
    declare_cloud_registered_topic_cmd = DeclareLaunchArgument(
        'cloud_registered_topic', default_value='/cloud_registered_1',
        description='FAST-LIO world-frame cloud output topic'
    )
    declare_cloud_registered_body_topic_cmd = DeclareLaunchArgument(
        'cloud_registered_body_topic', default_value='/cloud_registered_body_1',
        description='FAST-LIO body-frame cloud output topic'
    )
    declare_cloud_effected_topic_cmd = DeclareLaunchArgument(
        'cloud_effected_topic', default_value='/cloud_effected_1',
        description='FAST-LIO effect cloud output topic'
    )
    declare_laser_map_topic_cmd = DeclareLaunchArgument(
        'laser_map_topic', default_value='/Laser_map_1',
        description='FAST-LIO accumulated map output topic'
    )
    declare_odometry_topic_cmd = DeclareLaunchArgument(
        'odometry_topic', default_value='/Odometry_loc',
        description='FAST-LIO odometry output topic'
    )
    declare_path_topic_cmd = DeclareLaunchArgument(
        'path_topic', default_value='/path_1',
        description='FAST-LIO path output topic'
    )
    declare_map_save_service_cmd = DeclareLaunchArgument(
        'map_save_service', default_value='map_save',
        description='FAST-LIO accumulated map save service'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path),
        condition=IfCondition(start_livox_driver)
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {
                        'use_sim_time': use_sim_time,
                        'map_file_path': map_file_path,
                        'publish.map_en': ParameterValue(publish_map, value_type=bool),
                    }],
        remappings=[
            ('/cloud_registered_1', cloud_registered_topic),
            ('/cloud_registered_body_1', cloud_registered_body_topic),
            ('/cloud_effected_1', cloud_effected_topic),
            ('/Laser_map_1', laser_map_topic),
            ('/Odometry_loc', odometry_topic),
            ('/path_1', path_topic),
            ('map_save', map_save_service),
        ],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_start_livox_driver_cmd)
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_publish_map_cmd)
    ld.add_action(declare_cloud_registered_topic_cmd)
    ld.add_action(declare_cloud_registered_body_topic_cmd)
    ld.add_action(declare_cloud_effected_topic_cmd)
    ld.add_action(declare_laser_map_topic_cmd)
    ld.add_action(declare_odometry_topic_cmd)
    ld.add_action(declare_path_topic_cmd)
    ld.add_action(declare_map_save_service_cmd)

    ld.add_action(livox_launch)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
