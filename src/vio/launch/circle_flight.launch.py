import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_vio = get_package_share_directory('vio')

    # Include all basic drone components
    all_in_one_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'all_in_one.launch.py'))
    )

    # Circle flight controller
    circle_controller = Node(
        package='vio',
        executable='circle_flight_controller',
        name='circle_flight_controller',
        output='screen'
    )

    return LaunchDescription([
        all_in_one_launch,
        circle_controller,
    ])