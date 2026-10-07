"""balance_node with motors.yaml + leg_table.yaml + balance.yaml (+ balance_tables.yaml when present).
Stop gen2 motor test tools first: only one process may command the CAN bus."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    c = get_package_share_directory('gen2_control')
    files = [os.path.join(get_package_share_directory('gen2_hardware'), 'config', 'motors.yaml'),
             os.path.join(c, 'config', 'leg_table.yaml'), os.path.join(c, 'config', 'balance.yaml')]
    tables = os.path.join(c, 'config', 'balance_tables.yaml')
    if os.path.exists(tables):
        files.append(tables)
        files.append(os.path.join(c, 'config', 'balance_gains_robot.yaml'))   # robot-tuned lqr_k (needs the tables)
    return LaunchDescription([Node(package='gen2_control', executable='balance_node', name='balance',
                                   parameters=files, output='screen')])
