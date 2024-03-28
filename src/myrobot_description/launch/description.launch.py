from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
import os
import yaml

def generate_launch_description():

    current_pkg_dir = get_package_share_path('myrobot_description')
    default_rviz_config_path = current_pkg_dir / 'launch/default.rviz'
    ld = LaunchDescription([
        DeclareLaunchArgument(name='use_joy', default_value='true', choices=['true', 'false'],
                              description='Launch joy teleop'),
        DeclareLaunchArgument(name='use_rviz', default_value='false', choices=['true', 'false'],
                              description='Launch RVIZ'),
        DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                              description='RVIZ configuration file')
    ])

    # Description
    robot_description = ParameterValue(Command(['xacro ', str(current_pkg_dir / 'urdf' / 'myrobot.xacro')]), value_type=str)
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    ))

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    ))


    # IMU driver
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('mpu9250driver'),
            '/launch/mpu9250driver_launch.py'])
        )
    )

    # Lidar driver
    ld.add_action(Node(
        package='xv_11_driver',
        executable='xv_11_driver',
        parameters=[{
            "port": "/dev/ttyS5",
            "frame_id": "lidar_link"
        }],
        output='screen'
    ))


    # Controller
    ld.add_action(Node(
        package='myrobot_description',
        executable='myrobot_driver',
        name="myrobot_driver",
        output='screen'
    ))

    # # Wheel odometry
    # ld.add_action(Node(
    #     package='trk211_description',
    #     executable='trk211_odometer',
    #     output='screen'
    # ))

    # # Joy teleop
    # ld.add_action(Node(
    #     package='trk211_description',
    #     executable='trk211_joy_teleop',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_joy'))
    # ))

    # # RVIZ
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))
    
    ld.add_action(IncludeLaunchDescription(
        AnyLaunchDescriptionSource([get_package_share_directory('foxglove_bridge'),
            '/launch/foxglove_bridge_launch.xml'])
        )
    )
    return ld
