from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_testing',
            executable='chatter'
        ),
        Node(
            package='topic_testing',
            executable='latency'
        )
    ])