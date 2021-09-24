from glob import glob
import os
from setuptools import setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Path to launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Path to world file
        (os.path.join('share', package_name, 'gazebo/worlds/'), glob('./gazebo/worlds/*')),
        # Path to the grass sdf file
        (os.path.join('share', package_name, 'gazebo/models/grass_plane/'), glob('./gazebo/models/grass_plane/*')),
        # Path to the golfball sdf file
        (os.path.join('share', package_name, 'gazebo/models/GolfBall/'), glob('./gazebo/models/GolfBall/*')),
        # Path to the tape sdf file
        (os.path.join('share', package_name, 'gazebo/models/DropOffTape/'), glob('./gazebo/models/DropOffTape/*')),

        # Path to the world file (global enviornments?) TODO: figure out what this does.
        (os.path.join('share', package_name, 'gazebo/models/'), glob('./gazebo/worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adriancooper',
    maintainer_email='adriancooperwrx13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_demo = bringup.spawn_demo:main'
        ],
    },
)
