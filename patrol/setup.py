from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'patrol'),
            glob('patrol/*.wav')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'thermal_alarm_test_node = patrol.thermal_alarm_test_node:main',
            'simple_patrol_route_node = patrol.simple_patrol_route_node:main',
            'patrol_node = patrol.patrol_node:main',
            'record_checkpoints_node = patrol.record_checkpoints_node:main',
            'patrol_route_node = patrol.patrol_route_node:main',
        ],
    },
)
