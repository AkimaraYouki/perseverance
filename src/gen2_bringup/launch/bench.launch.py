"""bench profile: sensors + read-only motor monitor + status LCD. No motor TX is possible."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, file, cond=None):
    return IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(pkg), 'launch', file]),
        condition=IfCondition(cond) if cond is not None else None)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('display', default_value='true'),
        DeclareLaunchArgument('motors', default_value='true'),
        _inc('gen2_sensor_hub', 'sensor_hub.launch.py'),
        _inc('gen2_hardware', 'motor_monitor.launch.py', LaunchConfiguration('motors')),
        _inc('gen2_status_display', 'status_display.launch.py', LaunchConfiguration('display')),
    ])
