"""Control stack started at boot by gen2-balance.service: balance_node (disarmed, 0 A) + cmd_mux.
The operator arms it with START (pad_teleop) or g (balance_cli). Stop this service before hip_cli /
motor tests: only one process may command the CAN bus."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gen2_control'), 'launch', 'balance.launch.py')))])
