from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion',
            executable='navigation'
        ),
        Node(
            package='hardware',
            executable='controller'
        ),
        Node(
            package='manual_control',
            executable='joystick'
        )
    ])