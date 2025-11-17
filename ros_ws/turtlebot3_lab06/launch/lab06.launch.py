#!/usr/bin/env python3
"""
TurtleBot3 Lab06 - SLAM Implementation
SLAM launch file using workspace TurtleBot3 packages - Fixed Version
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories - using workspace packages
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    lab06_dir = get_package_share_directory('turtlebot3_lab06')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Set TurtleBot3 model environment variable
    set_turtlebot3_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        'waffle_pi'
    )

    # TurtleBot3 World Launch (from workspace)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox Node - Using sync version for more reliable operation
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([lab06_dir, 'config', 'slam_config.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz with SLAM visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([lab06_dir, 'config', 'turtlebot3_lab06_slam.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        set_turtlebot3_model,
        gazebo_launch,
        slam_toolbox_node,
        rviz_node,
    ])