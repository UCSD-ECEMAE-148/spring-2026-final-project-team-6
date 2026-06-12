from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol',
            executable='thermal_alarm_test_node',
            name='thermal_alarm_test_node',
            output='screen',
            emulate_tty=True,
        )
    ])
