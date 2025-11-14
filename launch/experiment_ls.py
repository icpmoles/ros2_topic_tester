from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():



    return LaunchDescription([
        Node(
            package='topic_tester',
            executable='latency_ls',
            name = "sink",
            parameters=[PathJoinSubstitution([
                FindPackageShare('topic_tester'), 'config', 'sink_ls.yaml'])
            ],
            remappings=[
                ('/input', '/laserscan') # either /laserscan or /lidar
            ]
        )
    ])
