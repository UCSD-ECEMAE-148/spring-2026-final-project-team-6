from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('patrol'),
        'config',
        'mapper_params_online_sync.yaml'
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.13', '0.0', '0.0', '0.0', '0.0', '3.1416', 'base_link', 'laser']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                '/opt/ros/jazzy/share/slam_toolbox/launch/online_sync_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': params_file
            }.items()
        )
    ])
