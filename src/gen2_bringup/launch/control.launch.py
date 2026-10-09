"""Control stack started at boot by gen2-balance.service: balance_node (disarmed, 0 A) + cmd_mux.
The operator arms it with START (pad_teleop) or g (balance_cli). Stop this service before hip_cli /
motor tests: only one process may command the CAN bus."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # sensor mounts in base_link (= midpoint of the hip axes; desktop CAD 2026-10-08). LiDAR yaw: measured 0
    # (the CAD connector says -180). balance_node publishes odom -> base_link.
    def static(child, xyz, ypr=(0.0, 0.0, 0.0)):
        return Node(package='tf2_ros', executable='static_transform_publisher', name=f'tf_{child}',
                    arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]), '--yaw', str(ypr[0]),
                               '--pitch', str(ypr[1]), '--roll', str(ypr[2]), '--frame-id', 'base_link', '--child-frame-id', child])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gen2_control'), 'launch', 'balance.launch.py'))),
        static('laser', (-0.0774, 0.0, 0.1225)),
        static('camera_link', (0.1080, 0.0, 0.0153)),
        static('imu_link', (0.0500, -0.0050, 0.0700)),
    ])
