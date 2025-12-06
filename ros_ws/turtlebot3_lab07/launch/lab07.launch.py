#!/usr/bin/env python3
"""
TurtleBot3 Lab07 - Autonomous Navigation with Nav2
Navigation launch file using Nav2 stack for autonomous navigation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    lab07_dir = get_package_share_directory('turtlebot3_lab07')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Map file - default to turtlebot3_navigation2 map, can be overridden
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(turtlebot3_navigation2_dir, 'map', 'map.yaml')
    )

    # Nav2 parameters
    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(turtlebot3_navigation2_dir, 'param', 'waffle_pi.yaml')
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(turtlebot3_navigation2_dir, 'map', 'map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(turtlebot3_navigation2_dir, 'param', 'waffle_pi.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    # Set TurtleBot3 model environment variable
    set_turtlebot3_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        'waffle_pi'
    )

    # TurtleBot3 World Launch (Gazebo simulation)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Nav2 Bringup Launch (includes map_server, amcl, planner, controller, etc.)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )

    # RViz with Navigation visualization
    rviz_config_file = os.path.join(turtlebot3_navigation2_dir, 'rviz', 'tb3_navigation2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_params_file_cmd,
        set_turtlebot3_model,
        gazebo_launch,
        nav2_bringup_launch,
        rviz_node,
    ])
