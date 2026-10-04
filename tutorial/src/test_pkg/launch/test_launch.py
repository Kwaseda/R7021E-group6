from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_pkg',
            executable='sender_node',
            name='sender',
            output='screen',
            parameters=[{'message': 'Hello from the launch file!'}]
        ),
        Node(
            package='test_pkg',
            executable='receiver_node',
            name='receiver',
            output='screen'
        )
    ])