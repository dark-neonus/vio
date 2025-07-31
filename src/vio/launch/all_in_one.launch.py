import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_vio = get_package_share_directory('vio')

    # Include robot_state_publisher launch
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'robot_state_publisher.launch.py'))
    )

    # Include gazebo launch (which should launch gazebo and ros_gz_bridge)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'gazebo.launch.py'))
    )

    # Include spawn_entity launch (spawns your robot in Gazebo using robot_description)
    spawn_entity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'spawn_entity.launch.py'))
    )

    # Include RViz launch (loads your robot model visualization)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'rviz.launch.py'))
    )

    # Include camera view launch (launches camera viewer node subscribing to camera topic)
    camera_view_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'camera_view.launch.py'))
    )

    # ADD: Circle flight controller for full simulation
    circle_controller = Node(
        package='vio',
        executable='circle_flight_controller',
        name='circle_flight_controller',
        output='screen'
    )

    # ADD: IMU monitor for terminal output
    imu_monitor = Node(
        package='vio',
        executable='imu_monitor',
        name='imu_monitor',
        output='screen'
    )

    # Aggregate all launches into one
    return LaunchDescription([
        rsp_launch,
        gazebo_launch,
        spawn_entity_launch,
        rviz_launch,
        camera_view_launch,
        circle_controller,
        imu_monitor,
    ])
