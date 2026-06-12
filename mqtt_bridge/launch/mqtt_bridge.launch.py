from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_bridge',
            executable='ros2_mqtt_bridge',
            name='ros2_mqtt_bridge',
            output='screen',
            emulate_tty=True,
        )
    ])
