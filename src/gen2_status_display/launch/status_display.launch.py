from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(package='gen2_status_display', executable='status_display', name='status_display',
             parameters=[PathJoinSubstitution(
                 [FindPackageShare('gen2_status_display'), 'config', 'status_display.yaml'])],
             output='screen', respawn=True, respawn_delay=3.0),
    ])
