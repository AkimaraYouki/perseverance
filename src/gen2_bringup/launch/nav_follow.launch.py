"""Nav2 planner + MPPI controller for person following with obstacle avoidance (odom frame, local only).
The controller's cmd_vel goes to cmd_vel/auto (cmd_mux). Needs gen2-balance (odom TF, scan_level).
Run: ros2 launch gen2_bringup nav_follow.launch.py   (person_follow uses it when use_nav is true)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('gen2_bringup'), 'config', 'nav2_follow.yaml')
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[cfg], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[cfg],
             remappings=[('cmd_vel', 'cmd_vel/auto')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav',
             parameters=[cfg], output='screen'),
    ])
