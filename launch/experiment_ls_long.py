from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():
    file_output = LaunchConfiguration('file_output_prefix')

    file_output_launch_arg = DeclareLaunchArgument(
        'file_output_prefix',
        default_value='/home/ros2/tesi/topic_tester/latency_ls_long'
    )
    
    
    return LaunchDescription([
        file_output_launch_arg,
        Node(
            package='topic_tester',
            executable='latency_ls',
            name = "sink",
            remappings=[
                ('/topic', '/laserscan') 
            ],
            parameters=[{
                "msg_type": 0,
                "msg_sample": 40,
                "rolling_window_size": 20,
                "file_output_prefix": file_output
                }]
        )
    ])
