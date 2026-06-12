from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol',
            executable='patrol_route_node',
            name='patrol_route_node',
            output='screen',
            emulate_tty=True,
        )
    ])
