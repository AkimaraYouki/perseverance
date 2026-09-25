import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("gen2_camera"), "config", "camera.yaml")
    return LaunchDescription([
        Node(package="gen2_camera", executable="camera_node", name="camera", output="screen", parameters=[cfg],
             respawn=True, respawn_delay=3.0),
    ])
