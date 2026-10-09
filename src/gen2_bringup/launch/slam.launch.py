"""2D SLAM while driving: scan_gate (drops tilted scans) + slam_toolbox online async (lifecycle node,
configured and activated here). Needs gen2-balance (odom TF) running.
Run: ros2 launch gen2_bringup slam.launch.py ; view /map in RViz on the desktop (gen2_bringup/rviz/gen2.rviz).
Save: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/parksudo/maps/room'}}" """
import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('gen2_bringup'), 'config', 'slam.yaml')
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                         namespace='', parameters=[cfg], output='screen')
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                              transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        Node(package='gen2_tools', executable='scan_gate', name='scan_gate', output='screen'),
        slam, activate, configure,
    ])
