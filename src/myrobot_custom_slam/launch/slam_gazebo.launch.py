#!/usr/bin/env python3
# Copyright 2024 myrobot_project. Apache-2.0.
"""Simulation bring-up: Gazebo classic + robot_state_publisher + custom SLAM.

This is a real integration test entry point, not just the SLAM node: the world
is started, the URDF (which carries the gazebo_ros diff_drive / ray / imu
plugins) is spawned, TF is provided by robot_state_publisher and only then the
filter consumes /scan and /imu.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    slam_share = get_package_share_directory('myrobot_custom_slam')
    description_share = get_package_share_directory('myrobot_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(description_share, 'urdf', 'myrobot.xacro')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(slam_share, 'config', 'slam.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py'),
            ]),
            launch_arguments={'world': world}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'myrobot'],
        ),

        Node(
            package='myrobot_custom_slam',
            executable='slam_node',
            name='myrobot_slam',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': True}],
            arguments=['-d', os.path.join(slam_share, 'rviz', 'slam.rviz')],
        ),
    ])
