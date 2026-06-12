from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol',
            executable='simple_patrol_route_node',
            name='simple_patrol_route_node',
            output='screen',
            emulate_tty=True,
        )
    ])
