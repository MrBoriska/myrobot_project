from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run slam node
        Node(
            package='myrobot_custom_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
        ),

        # Publish dummy transforms or states if needed for tests
        # We assume standard gazebo or robot state publisher is launched separately,
        # but here we just launch the SLAM node for isolated testing.
    ])
