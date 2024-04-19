from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
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

    # IMU driver
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('mpu9250driver'),
            '/launch/mpu9250driver_launch.py'])
        )
    )

    ld.add_action(Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{
            "stateless": False,
            "use_mag": False,
            "publish_tf": False,
            "reverse_tf": False,
            "fixed_frame": "odom",
            "constant_dt": 0.0,
            "publish_debug_topics": False,
            "world_frame": "enu",
            "gain": 0.1,
            "zeta": 0.0,
            "mag_bias_x": 0.0,
            "mag_bias_y": 0.0,
            "mag_bias_z": 0.0,
            "orientation_stddev": 0.0,
        }],
        remappings=[
            ('/imu/data', '/imu')
        ],
        output='screen'
    ))

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
    ld.add_action(GroupAction(
        actions=[
            Node(
                package='myrobot_description',
                executable='myrobot_wheel_fb',
                name='myrobot_wheel_fb',
                namespace='left_wheel',
                parameters=[{"fd":"/sys/class/gpio/gpio72/value"}],
                #prefix="nice -n -18",
                output='screen'
            ),
            Node(
                package='myrobot_description',
                executable='myrobot_wheel_fb',
                name='myrobot_wheel_fb',
                namespace='right_wheel',
                parameters=[{"fd":"/sys/class/gpio/gpio74/value"}],
                #prefix="nice -n -18",
                output='screen'
            ),
            Node(
                package='myrobot_description',
                executable='myrobot_driver',
                name='myrobot_driver',
                #prefix="nice -n -18",
                output='screen'
            )
        ]
    ))

    # ld.add_action(ComposableNodeContainer(
    #         name='image_container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package='myrobot_description',
    #                 executable='myrobot_wheel_fb',
    #                 name='left_wheel',
    #                 parameters=[{"fd":"/sys/class/gpio/gpio72/value"}],
    #                 extra_arguments=[{'use_intra_process_comms': True}]),
    #             ComposableNode(
    #                 package='myrobot_description',
    #                 executable='myrobot_wheel_fb',
    #                 name='right_wheel',
    #                 parameters=[{"fd":"/sys/class/gpio/gpio74/value"}],
    #                 extra_arguments=[{'use_intra_process_comms': True}]),
    #             ComposableNode(
    #                 package='myrobot_description',
    #                 executable='myrobot_driver',
    #                 name='myrobot_driver',
    #                 extra_arguments=[{'use_intra_process_comms': True}])
    #         ],
    #         output='both',
    # ))

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

    # Power Monitor
    ld.add_action(Node(
        package='myrobot_description',
        executable='myrobot_pwrmon',
        name="myrobot_pwrmon",
        output='screen'
    ))

    # # RVIZ
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))
    
    # ld.add_action(IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource([get_package_share_directory('foxglove_bridge'),
    #         '/launch/foxglove_bridge_launch.xml'])
    #     )
    # )
    return ld
