# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():

#     image_view_node = Node(
#         package='image_tools',                     # or use 'image_view' if installed
#         executable='cam2image',
#         name='camera_view',
#         output='screen',
#         arguments=['--ros-args', '--remap', '/camera/image_raw:=/your_camera_topic']
#     )

#     return LaunchDescription([
#         image_view_node
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    image_view_node = Node(
        package='image_tools',   # or use 'image_view' if you have it
        executable='showimage',  # or suitable viewer exe for your package
        name='camera_view',
        output='screen',
        arguments=[
            '--ros-args',
            '--remap', '/image:=/camera/image_raw'  # Make sure this matches your bridge output!
        ]
    )

    return LaunchDescription([image_view_node])
