from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = LaunchConfiguration('motors_config')
    return LaunchDescription([
        DeclareLaunchArgument('motors_config', default_value=PathJoinSubstitution(
            [FindPackageShare('gen2_hardware'), 'config', 'motors.yaml'])),
        Node(package='gen2_hardware', executable='motor_monitor_node', name='motor_monitor',
             parameters=[cfg], output='screen', respawn=True, respawn_delay=2.0),
    ])
