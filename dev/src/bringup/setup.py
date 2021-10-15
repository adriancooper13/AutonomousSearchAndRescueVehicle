from glob import glob
from os.path import join
from setuptools import setup

package_name = 'bringup'
gazebo = '../../../gazebo'
directories = {
    'worlds': join(gazebo, 'worlds'),
    'models': join(gazebo, 'models')
}

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Path to launch file
        (join('share', package_name, 'launch'), glob('launch/*.launch.py'))
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
