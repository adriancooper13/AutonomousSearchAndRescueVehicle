from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='camera_driver'
        ),
        Node(
            package='vision',
            executable='image_processing'
        ),
        Node(
            package='motion',
            executable='manual_control'
        ),
        Node(
            package='motion',
            executable='joy_linux_node'
        ),
        Node(
            package='motion',
            executable='navigation'
        )
    ])
