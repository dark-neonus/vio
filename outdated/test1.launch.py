import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    package_name='vio'

    # Add your own gazebo library path here
    gazebo_models_path = "/home/dark-neonus/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    world_arg = DeclareLaunchArgument(
        'world', default_value='forest1.sdf',
        description='Name of the Gazebo world file to load'
    )

    world_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'worlds',
        LaunchConfiguration('world')
    ])

    # Pass the list of substitutions as a list, let the launch system join them for you
    gz_args = [TextSubstitution(text='-r -v -v1 '), world_path]
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Fixed: Use correct filename
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Launch them all!
    return LaunchDescription([
        world_arg,
        gazebo,
        ros_gz_bridge,
    ])