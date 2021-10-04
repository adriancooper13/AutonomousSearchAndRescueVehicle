from setuptools import setup

package_name = 'manual_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'inputs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'joystick = manual_control.joystick_handler:main'
        ],
    },
)
