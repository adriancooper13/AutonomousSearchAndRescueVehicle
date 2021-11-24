from simulation.filepaths import directories
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from os.path import join
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'golfball_field.world'
 
    os.environ["GAZEBO_MODEL_PATH"] = directories['models']
    world = join(directories['worlds'], world_file_name)
 
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world,
            '-s',
            'libgazebo_ros_init.so', 
            '-s',
            'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='simulation',
        executable='spawn_demo',
        output='screen'
    )
    delete_entity = Node(
        package='simulation',
        executable='remove_golfballs',
        output='screen'
    )
 
    return LaunchDescription([
        gazebo,
        spawn_entity,
        delete_entity
    ])
