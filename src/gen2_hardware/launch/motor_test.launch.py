"""motor_test profile: guarded single-motor tests with the GUI.

Needs /motors/state from the bench profile (gen2-bench.service) or pass with_monitor:=true.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = LaunchConfiguration('motors_config')
    gui = Node(package='gen2_tools', executable='motor_test_gui', name='motor_test_gui',
               output='screen', condition=IfCondition(LaunchConfiguration('gui')))
    return LaunchDescription([
        DeclareLaunchArgument('motors_config', default_value=PathJoinSubstitution(
            [FindPackageShare('gen2_hardware'), 'config', 'motors.yaml'])),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('with_monitor', default_value='false'),
        Node(package='gen2_hardware', executable='motor_test_node', name='motor_test',
             parameters=[cfg], output='screen'),
        Node(package='gen2_hardware', executable='motor_monitor_node', name='motor_monitor',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('with_monitor'))),
        gui,
        # closing the GUI window ends the whole profile (motor_test_node sends 0 A on exit)
        RegisterEventHandler(OnProcessExit(target_action=gui,
                                           on_exit=[EmitEvent(event=Shutdown(reason='GUI closed'))])),
    ])
