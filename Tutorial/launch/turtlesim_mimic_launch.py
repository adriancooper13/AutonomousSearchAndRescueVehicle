from launch import LaunchDescription
from launch_ros.actions import Node

# Example of creating a launch file to avoid having to open so many terminal windows.
# This launch file opens two turtlesim windows, and has the second turtle mimic the movements
# of the first turtle.
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            namespace="turtlesim1",
            executable="turtlesim_node",
            name="sim"
        ),
        Node(
            package="turtlesim",
            namespace="turtlesim2",
            executable="turtlesim_node",
            name="sim"
        ),
        Node(
            package="turtlesim",
            executable="mimic",
            name="mimic",
            remappings=[
                ("/input/pose", "/turtlesim1/turtle1/pose"),
                ("/output/cmd_vel", "/turtlesim2/turtle1/cmd_vel")
            ]
        )
    ])