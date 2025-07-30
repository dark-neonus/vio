import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the robot description from the parameter 'robot_description'
    # Note: This assumes that robot_state_publisher is launched beforehand,
    # or if you want you can call xacro here directly (but it's cleaner with rsp separate).
    
    # To spawn from a parameter, ros_gz_sim's `create` accepts '-string' with XML.
    # But launch substitution cannot directly pass a parameter, so we spawn based on topic.

    # Spawn entity node using 'ros_gz_sim create' executable:
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-name', 'test_rb_1',        # Entity name in Gazebo
            '-topic', 'robot_description' # Specify that the robot XML comes from this topic
        ]
    )

    return LaunchDescription([
        spawn_entity,
    ])
