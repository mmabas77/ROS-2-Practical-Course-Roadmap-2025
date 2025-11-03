import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_lab05'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmabas77',
    maintainer_email='mmabas77@gmail.com',
    description='TurtleBot3 Lab05 - Camera, LiDAR and Odometry Monitoring with RViz',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_monitor = turtlebot3_lab05.odometry_monitor:main',
            'sensor_logger = turtlebot3_lab05.sensor_logger:main',
        ],
    },
)
