from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='image_processing'
        ),
        Node(
            package='manual_control',
            executable='manual_control'
        ),
        Node(
            package='manual_control',
            executable='joy_linux_node'
        ),
        Node(
            package='motion',
            executable='navigation'
        )
    ])