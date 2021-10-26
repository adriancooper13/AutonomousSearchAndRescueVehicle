from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='image_processing'
        ),
        Node(
            package='motion',
            executable='manual_control'
        ),
        Node(
            package='hardware',
            executable='controller'
        ),
        Node(
            package='manual_control',
            executable='joystick'
        ),
        Node(
            package='motion',
            executable='navigation'
        )
    ])