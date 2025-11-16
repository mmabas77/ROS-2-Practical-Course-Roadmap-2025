#!/usr/bin/env python3
"""
TurtleBot3 Lab06 - SLAM Implementation
Direct SLAM launch file using workspace TurtleBot3 packages with lifecycle management
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
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

    # TurtleBot3 World Launch (from workspace)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
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

    # Lifecycle management - configure SLAM Toolbox after startup
    configure_slam = TimerAction(
        period=5.0,  # Wait 5 seconds for node to start
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen',
                shell=True
            )
        ]
    )

    # Lifecycle management - activate SLAM Toolbox after configuration
    activate_slam = TimerAction(
        period=7.0,  # Wait 7 seconds total
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen',
                shell=True
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo_launch,
        slam_toolbox_node,
        rviz_node,
        configure_slam,
        activate_slam,
    ])