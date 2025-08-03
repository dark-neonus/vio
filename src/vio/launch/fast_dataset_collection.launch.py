import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_vio = get_package_share_directory('vio')

    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='forest1.sdf',
        description='World file to load'
    )
    
    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='aggressive',
        description='Flight pattern to execute'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='20.0',
        description='Maximum speed for aggressive pattern'
    )
    
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/tmp/vio_dataset',
        description='Path to save dataset'
    )
    
    session_name_arg = DeclareLaunchArgument(
        'session_name',
        default_value='',
        description='Session name (auto-generated if empty)'
    )
    
    record_rate_arg = DeclareLaunchArgument(
        'record_rate',
        default_value='10.0',
        description='Recording rate in Hz'
    )

    # Include robot_state_publisher launch
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'robot_state_publisher.launch.py'))
    )

    # Include headless gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'gazebo_headless.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    # Include spawn_entity launch
    spawn_entity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'spawn_entity.launch.py'))
    )

    # Pattern flight controller with specified pattern
    pattern_controller = Node(
        package='vio',
        executable='pattern_flight_controller',
        name='pattern_flight_controller',
        output='screen',
        parameters=[{
            'initial_pattern': LaunchConfiguration('pattern'),
            'max_speed': LaunchConfiguration('max_speed')
        }]
    )

    # Dataset recorder
    dataset_recorder = Node(
        package='vio',
        executable='dataset_recorder',
        name='dataset_recorder',
        output='screen',
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'session_name': LaunchConfiguration('session_name'),
            'record_rate': LaunchConfiguration('record_rate')
        }]
    )

    # Pattern commander to send initial command
    pattern_commander = Node(
        package='vio',
        executable='pattern_commander',
        name='pattern_commander',
        output='screen',
        arguments=[LaunchConfiguration('pattern'), '--max_speed', LaunchConfiguration('max_speed')]
    )

    return LaunchDescription([
        world_arg,
        pattern_arg,
        max_speed_arg,
        dataset_path_arg,
        session_name_arg,
        record_rate_arg,
        rsp_launch,
        gazebo_launch,
        spawn_entity_launch,
        pattern_controller,
        dataset_recorder,
        pattern_commander,
    ])