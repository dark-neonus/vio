import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'vio'

    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='forest1.sdf',
        description='Name of the SDF world file to load'
    )

    # Compose full path to the world file (SDF)
    world_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'worlds',
        LaunchConfiguration('world')
    ])

    # Set Gazebo models path
    gazebo_models_path = "/home/dark-neonus/gazebo_models"

    # Set GZ_SIM_RESOURCE_PATH environment variable
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.environ.get('GZ_SIM_RESOURCE_PATH', '') + os.pathsep + gazebo_models_path
    )

    # Launch Gazebo in headless mode (no GUI)
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -s -v1 '), world_path],  # -s for headless
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Bridge config
    bridge_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ros_gz_bridge.yaml'
    )

    # ros_gz_bridge node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params_file}'
        ],
    )

    return LaunchDescription([
        world_arg,
        set_gz_resource_path,
        gz_sim_launch,
        ros_gz_bridge_node,
    ])