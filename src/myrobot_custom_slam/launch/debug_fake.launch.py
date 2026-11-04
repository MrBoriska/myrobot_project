#!/usr/bin/env python3
# Copyright 2024 myrobot_project. Apache-2.0.
"""Offline debug bring-up: synthetic sensors + SLAM, no Gazebo, no GPU.

Runs anywhere ROS 2 Humble is installed (including inside the dev container)
and needs no robot:

    ros2 launch myrobot_custom_slam debug_fake.launch.py
    ros2 launch myrobot_custom_slam debug_fake.launch.py drive:=static rviz:=true
    ros2 run myrobot_custom_slam check_slam_accuracy.py --duration 60
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('myrobot_custom_slam')
    params_file = LaunchConfiguration('params_file')
    drive = LaunchConfiguration('drive')
    rviz = LaunchConfiguration('rviz')
    room = LaunchConfiguration('room_size')
    speed = LaunchConfiguration('speed')
    odom_scale = LaunchConfiguration('odom_scale')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'slam.yaml')),
        DeclareLaunchArgument('drive', default_value='circle',
                              description='circle | static | cmd_vel'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('room_size', default_value='6.0'),
        DeclareLaunchArgument('speed', default_value='0.3'),
        DeclareLaunchArgument('odom_scale', default_value='1.02',
                              description='wheel odometry scale error to be corrected'),

        Node(
            package='myrobot_custom_slam',
            executable='fake_sensors.py',
            name='fake_sensors',
            output='screen',
            arguments=[
                '--room-size', room,
                '--drive', drive,
                '--speed', speed,
                '--odom-scale', odom_scale,
            ],
        ),

        Node(
            package='myrobot_custom_slam',
            executable='slam_node',
            name='myrobot_slam',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(share, 'rviz', 'slam.rviz')],
        ),
    ])
