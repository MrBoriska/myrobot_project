#!/usr/bin/env python3
# Copyright 2024 myrobot_project. Apache-2.0.
"""Bring up the real robot: drivers + description + custom SLAM."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('myrobot_custom_slam')
    default_params = os.path.join(share, 'config', 'slam.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    use_description = LaunchConfiguration('use_description')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_description', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        # Drivers, robot_state_publisher, IMU filter, lidar driver.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('myrobot_description'),
                    'launch', 'description.launch.py'),
            ]),
            condition=IfCondition(use_description),
        ),

        Node(
            package='myrobot_custom_slam',
            executable='slam_node',
            name='myrobot_slam',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(share, 'rviz', 'slam.rviz')],
        ),
    ])
