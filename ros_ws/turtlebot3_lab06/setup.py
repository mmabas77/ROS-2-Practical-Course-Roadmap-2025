import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_lab06'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmabas77',
    maintainer_email='mmabas77@gmail.com',
    description='TurtleBot3 Lab06 - SLAM (Simultaneous Localization and Mapping) with Real-time Map Creation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_saver = turtlebot3_lab06.map_saver:main',
        ],
    },
)