"""balance_node with motors.yaml + leg_table.yaml + balance.yaml (+ balance_tables.yaml when present).
Optional tune overlay: config/balance_tune_<name>.yaml loaded last, <name> from $GEN2_TUNE or ~/.ros/gen2_tune
(one word, e.g. "opt1"; empty / missing = balance.yaml as is). Switch with gen2_tune.sh <name>|default.
Stop gen2 motor test tools first: only one process may command the CAN bus."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

TUNE_FILE = os.path.expanduser('~/.ros/gen2_tune')


def tune_name():
    name = os.environ.get('GEN2_TUNE', '')
    if not name and os.path.exists(TUNE_FILE):
        name = open(TUNE_FILE).read().strip()
    return name if name and name != 'default' else ''


def generate_launch_description():
    c = get_package_share_directory('gen2_control')
    files = [os.path.join(get_package_share_directory('gen2_hardware'), 'config', 'motors.yaml'),
             os.path.join(c, 'config', 'leg_table.yaml'), os.path.join(c, 'config', 'balance.yaml')]
    tables = os.path.join(c, 'config', 'balance_tables.yaml')
    if os.path.exists(tables):
        files.append(tables)
        files.append(os.path.join(c, 'config', 'balance_gains_robot.yaml'))   # robot-tuned lqr_k (needs the tables)
    name = tune_name()
    overlay = os.path.join(c, 'config', f'balance_tune_{name}.yaml') if name else ''
    if overlay and os.path.exists(overlay):
        files.append(overlay)                                                   # last: overrides balance.yaml
        msg = f'tune overlay: {name.upper()} ({overlay})'
    elif name:
        msg = f'tune overlay {name!r} requested but {overlay} is missing -> balance.yaml only'
    else:
        msg = 'tune overlay: none (balance.yaml)'
    return LaunchDescription([LogInfo(msg=msg),
                              Node(package='gen2_control', executable='balance_node', name='balance',
                                   parameters=files, output='screen'),
                              # cmd_vel/teleop + cmd_vel/auto -> LiDAR safety -> cmd_vel
                              Node(package='gen2_tools', executable='cmd_mux', name='cmd_mux',
                                   parameters=[os.path.join(c, 'config', 'cmd_mux.yaml')], output='screen')])
