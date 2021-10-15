from os.path import join
from ament_index_python.packages import get_package_share_directory

GAZEBO = '../../../../../gazebo'
packages = {
    'launch': get_package_share_directory('bringup')
} 

directories = {
    'models': join(packages['launch'], GAZEBO, 'models'),
    'worlds': join(packages['launch'], GAZEBO, 'worlds')
}
models = {
    'grass': 'grass_plane',
    'golfball': 'golfball',
    'tape': 'dropoff_tape',
    'robot': 'turtlebot3_waffle',
    'border': 'border'
}
filenames = {
    'model': 'model.sdf'
}
model_paths = {
    'grass': join(
        directories['models'],
        models['grass'],
        filenames['model']
    ),
    'golfball': join(
        directories['models'],
        models['golfball'],
        filenames['model']
    ),
    'tape': join(
        directories['models'],
        models['tape'],
        filenames['model']
    ),
    'robot': join(
        directories['models'],
        models['robot'],
        filenames['model']
    ),
    'border': join(
        directories['models'],
        models['border'],
        filenames['model']
    )
}
