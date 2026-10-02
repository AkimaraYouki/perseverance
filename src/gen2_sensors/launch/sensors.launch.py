"""IMU (iAHRS, 500 Hz) + RPLIDAR C1 (/scan) + scan diagnostics. Devices by udev name."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('gen2_sensors'), 'config', 'iahrs.yaml'])
    return LaunchDescription([
        Node(package='gen2_sensors', executable='iahrs_node', name='iahrs', parameters=[cfg],
             respawn=True, respawn_delay=2.0, output='screen'),
        Node(package='rplidar_ros', executable='rplidar_node', name='rplidar',
             parameters=[{'channel_type': 'serial', 'serial_port': '/dev/gen2_lidar',
                          'serial_baudrate': 460800, 'frame_id': 'laser', 'inverted': False,
                          'angle_compensate': True, 'scan_mode': 'Standard'}],
             respawn=True, respawn_delay=3.0, output='screen'),
        Node(package='gen2_sensors', executable='scan_monitor', name='scan_monitor'),
    ])
