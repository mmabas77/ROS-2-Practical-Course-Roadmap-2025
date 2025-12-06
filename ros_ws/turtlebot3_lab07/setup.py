import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_lab07'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmabas77',
    maintainer_email='mmabas77@gmail.com',
    description='TurtleBot3 Lab07 - Autonomous Navigation with Nav2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # No custom nodes - Lab07 uses Nav2 system nodes
        ],
    },
)
