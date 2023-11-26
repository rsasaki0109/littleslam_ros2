from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='littleslam_ros2',
            executable='littleslam_ros2',
            name='littleslam_ros2',
            output='screen',
            parameters=[
                {'use_odom': False}
            ]
        )
    ])
