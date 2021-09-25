import os
from helpers.filepaths import directories
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'empty.world'
    pkg_dir = get_package_share_directory('bringup')
 
    os.environ["GAZEBO_MODEL_PATH"] = directories['models']
    world = os.path.join(directories['worlds'], world_file_name)
 
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    spawn_entity = Node(
        package='bringup',
        executable='spawn_demo',
        output='screen'
    )
 
    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])