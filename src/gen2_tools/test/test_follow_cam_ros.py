"""ROS test of person_follow_cam with fake inputs (desktop, own domain): a person 2 m ahead at +10 deg (LiDAR cluster +
people/target LOCK) -> after follow/enable: 'register' sent, FOLLOW, vx clipped to 0.7, wz toward the person; target
LOST for 1 s -> OCCLUDED (keeps going to the predicted place), back -> FOLLOW."""
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from gen2_msgs.msg import ControllerState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gen2_tools.person_follow_cam import PersonFollowCam  # noqa: E402


class Fake(Node):
    def __init__(self):
        super().__init__('fake')
        self.p_state = self.create_publisher(ControllerState, 'controller/state', 10)
        self.p_odom = self.create_publisher(Odometry, 'odom', 10)
        self.p_scan = self.create_publisher(LaserScan, 'scan_level', 10)
        self.p_tg = self.create_publisher(String, 'people/target', 10)
        self.cmds, self.status, self.tcmd = [], [], []
        self.create_subscription(Twist, 'cmd_vel/auto', lambda m: self.cmds.append((m.linear.x, m.angular.z)), 10)
        self.create_subscription(String, 'follow/status', lambda m: self.status.append(m.data), 10)
        self.create_subscription(String, 'people/target_cmd', lambda m: self.tcmd.append(m.data), 10)
        self.cli = self.create_client(SetBool, 'follow/enable')
        self.target_state = 'LOCK'

    def tick(self):
        s = ControllerState()
        s.mode = 'balance'
        s.h = [0.18, 0.18]
        self.p_state.publish(s)
        o = Odometry()
        o.pose.pose.orientation.w = 1.0
        self.p_odom.publish(o)
        sc = LaserScan()
        sc.angle_min, sc.angle_increment, sc.range_min, sc.range_max = -math.pi, math.radians(0.5), 0.1, 12.0
        a = sc.angle_min + np.arange(720) * sc.angle_increment
        r = np.full(720, np.inf)
        b = math.radians(10)                                # person (two legs) 2.0 m from the camera at +10 deg
        for k in range(720):
            if abs(a[k] - b) < math.radians(2.5):
                r[k] = 2.0 + 0.03 * math.sin(k)
        sc.ranges = [float(x) for x in r]
        self.p_scan.publish(sc)
        j = {'stamp': time.time(), 'state': self.target_state, 'id': 1, 'w': 640, 'h': 480}
        if self.target_state != 'LOST':
            j.update(box=[300, 100, 360, 400], bearing_deg=10.0, span_deg=[12.0, 8.0], feet_row=400.0, score=0.9, sim=None)
        self.p_tg.publish(String(data=json.dumps(j)))


def main():
    os.environ['ROS_DOMAIN_ID'] = '77'
    rclpy.init()
    f, n = Fake(), PersonFollowCam()
    n.set_parameters([rclpy.parameter.Parameter('use_nav', value=False)])
    n.use_nav = False
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(f); ex.add_node(n)  # noqa: E702
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t0 = time.time()
    while time.time() - t0 < 1.0:
        f.tick(); time.sleep(0.033)  # noqa: E702
    assert f.cli.wait_for_service(2.0)
    fut = f.cli.call_async(SetBool.Request(data=True))
    while not fut.done():
        time.sleep(0.01)
    print('enable:', fut.result().message)
    phases = [('LOCK', 2.0), ('LOST', 1.0), ('LOCK', 1.0)]
    out = {}
    for st, dur in phases:
        f.target_state = st
        f.cmds.clear(); f.status.clear()  # noqa: E702
        t0 = time.time()
        while time.time() - t0 < dur:
            f.tick(); time.sleep(0.033)  # noqa: E702
        c = np.array(f.cmds[-10:]) if f.cmds else np.zeros((1, 2))
        out.setdefault(st, []).append((f.status[-1] if f.status else '-', c.mean(0).round(3).tolist(), len(f.cmds) / dur))
        print(f'{st:5s} {dur:.0f} s: status "{f.status[-1] if f.status else "-"}", cmd vx/wz {c.mean(0).round(3).tolist()}, {len(f.cmds) / dur:.0f} cmd/s')
    ex.shutdown()
    rclpy.try_shutdown()
    assert 'register' in f.tcmd, 'no register sent'
    s0, c0, hz0 = out['LOCK'][0]
    assert s0.startswith('FOLLOW') and 'camera+lidar' in s0, s0
    assert abs(c0[0] - 0.7) < 1e-3 and 0.15 < c0[1] < 0.4, c0
    assert hz0 > 25, f'command rate {hz0}'
    assert out['LOST'][0][0].startswith('OCCLUDED'), out['LOST'][0][0]
    assert out['LOCK'][1][0].startswith('FOLLOW'), out['LOCK'][1][0]
    print('ok')


if __name__ == '__main__':
    main()
