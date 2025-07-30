import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_vio = get_package_share_directory('vio')
    rviz_config_file = os.path.join(pkg_vio, 'rviz', 'robot_visualization.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([rviz_node])
