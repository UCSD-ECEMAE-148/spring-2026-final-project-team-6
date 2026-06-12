from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol',
            executable='record_checkpoints_node',
            name='record_checkpoints_node',
            output='screen',
            emulate_tty=True,
        )
    ])
