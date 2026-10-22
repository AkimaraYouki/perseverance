"""bench profile: sensor hub (hub:=false to skip) + IMU + LiDAR + read-only motor monitor + status LCD + camera + health log
(logs/health: 1 Hz CSV + events, see gen2_tools/health_logger.py). No motor TX is possible."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, file, cond=None):
    return IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(pkg), 'launch', file]),
        condition=IfCondition(cond) if cond is not None else None)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('display', default_value='true'),
        DeclareLaunchArgument('motors', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('health', default_value='true'),
        # ESP32 sensor hub (GPS, magnetometer, PM02 battery V/I): removed 2026-10-14, refitted 2026-10-22 -> on
        DeclareLaunchArgument('hub', default_value='true'),
        _inc('gen2_sensor_hub', 'sensor_hub.launch.py', LaunchConfiguration('hub')),
        _inc('gen2_sensors', 'sensors.launch.py', LaunchConfiguration('sensors')),
        _inc('gen2_hardware', 'motor_monitor.launch.py', LaunchConfiguration('motors')),
        _inc('gen2_status_display', 'status_display.launch.py', LaunchConfiguration('display')),
        _inc('gen2_camera', 'camera.launch.py', LaunchConfiguration('camera')),
        Node(package='gen2_tools', executable='health_logger', name='health_logger', output='log',
             condition=IfCondition(LaunchConfiguration('health'))),
    ])
