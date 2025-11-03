#!/usr/bin/env python3
"""
Complete TurtleBot3 Lab05 Launch File
Launches TurtleBot3 simulation, RViz, and monitoring nodes
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    lab05_dir = get_package_share_directory('turtlebot3_lab05')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='turtlebot3_world')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='World to load in Gazebo'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch'),
            '/turtlebot3_world.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # RViz launch with custom config
    rviz_config_file = PathJoinSubstitution([
        lab05_dir, 'config', 'turtlebot3_lab05.rviz'
    ])

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Odometry monitor node
    odometry_monitor_node = Node(
        package='turtlebot3_lab05',
        executable='odometry_monitor',
        name='odometry_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Sensor data logger node
    sensor_logger_node = Node(
        package='turtlebot3_lab05',
        executable='sensor_logger',
        name='sensor_data_logger',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        gazebo_launch,
        rviz_launch,
        odometry_monitor_node,
        sensor_logger_node,
    ])