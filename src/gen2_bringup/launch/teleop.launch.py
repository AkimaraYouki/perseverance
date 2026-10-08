"""Game pad operator: joy_node + pad_teleop (config/pad_teleop.yaml). Needs gen2-balance running.
Run: ros2 launch gen2_bringup teleop.launch.py   (pad on this machine's USB / Bluetooth)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('gen2_bringup'), 'config', 'pad_teleop.yaml')
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[cfg], output='screen'),
        Node(package='gen2_tools', executable='pad_teleop', name='pad_teleop', parameters=[cfg], output='screen'),
    ])
