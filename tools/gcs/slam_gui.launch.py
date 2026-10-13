"""SLAM for the GCS GUI: gen2_bringup/slam.launch.py with a parameter file argument (the GUI writes
~/gen2_desk/slam_gui.yaml = slam.yaml + the values tuned in the GUI).
Run: ros2 launch ~/gen2_desk/slam_gui.launch.py params:=$HOME/gen2_desk/slam_gui.yaml"""
import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    default = os.path.join(get_package_share_directory('gen2_bringup'), 'config', 'slam.yaml')
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                         namespace='', parameters=[LaunchConfiguration('params')], output='screen')
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                              transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default),
        slam, activate, configure,
    ])
