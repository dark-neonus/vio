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

    # Include gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'gazebo.launch.py'))
    )

    # Include spawn_entity launch
    spawn_entity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'spawn_entity.launch.py'))
    )

    # Include RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'rviz.launch.py'))
    )

    # Include camera view launch
    camera_view_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vio, 'launch', 'camera_view.launch.py'))
    )

    # Pattern-based flight controller for VIO dataset collection
    pattern_controller = Node(
        package='vio',
        executable='pattern_flight_controller',
        name='pattern_flight_controller',
        output='screen'
    )

    # IMU monitor for data verification
    imu_monitor = Node(
        package='vio',
        executable='imu_monitor',
        name='imu_monitor',
        output='screen'
    )

    # Pattern commander for interactive control
    pattern_commander = Node(
        package='vio',
        executable='pattern_commander',
        name='pattern_commander',
        output='screen'
    )

    # Data recorder node (optional - for later implementation)
    # data_recorder = Node(
    #     package='vio',
    #     executable='data_recorder',
    #     name='data_recorder',
    #     output='screen'
    # )

    return LaunchDescription([
        rsp_launch,
        gazebo_launch,
        spawn_entity_launch,
        rviz_launch,
        camera_view_launch,
        pattern_controller,
        imu_monitor,
        pattern_commander,
        # data_recorder,  # Uncomment when implemented
    ])