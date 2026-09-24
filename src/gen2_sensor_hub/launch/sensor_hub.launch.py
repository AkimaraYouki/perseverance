from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [FindPackageShare('gen2_sensor_hub'), 'config', 'sensor_hub.yaml'])),
        Node(package='gen2_sensor_hub', executable='sensor_hub_node', name='sensor_hub',
             parameters=[cfg], output='screen', respawn=True, respawn_delay=2.0),
    ])
