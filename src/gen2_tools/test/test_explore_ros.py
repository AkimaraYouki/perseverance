"""ROS test of person_follow_cam occlusion explore with a small 2-D world (GCS, own domain): the person walks ahead,
turns around the corner of a box and is hidden (no camera target, LiDAR sees the box). Expect: FOLLOW -> OCCLUDED ->
EXPLORE (to where the person vanished, then on along their heading) -> the person comes back into view (REACQUIRED)
-> FOLLOW, without Nav2 (use_nav false: direct steering around the box).

  python3 test/test_explore_ros.py [corner|pillar]      (ROS + gen2_msgs sourced)
"""
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

LASER_X, CAM_X, HFOV = -0.0774, 0.108, math.radians(124)
SCENES = {
    # box x 2.2..3.0, y 0.5..2.0; the person walks +x along y 0 to x 3.5, then +y (behind the box) to y 3
    'corner': dict(boxes=[(2.2, 0.5, 3.0, 2.0)], path=[(1.6, 0.0), (3.5, 0.0), (3.5, 3.0)]),
    # a pillar 0.4 m wide straight ahead; the person walks left behind it and stops 1.5 m past it
    'pillar': dict(boxes=[(2.0, -0.2, 2.4, 0.2)], path=[(3.2, -1.2), (3.2, 1.6)]),
}


def seg_box(p, q, b):
    """Segment p-q hits the axis-aligned box b?"""
    x0, y0, x1, y1 = b
    for s in np.linspace(0, 1, 60):
        x, y = p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False


class World(Node):
    def __init__(self, scene):
        super().__init__('world')
        self.sc = SCENES[scene]
        self.p_state = self.create_publisher(ControllerState, 'controller/state', 10)
        self.p_odom = self.create_publisher(Odometry, 'odom', 10)
        self.p_tg = self.create_publisher(String, 'people/target', 10)
        self.p_scan = self.create_publisher(LaserScan, 'scan_level', 10)
        self.cmd = (0.0, 0.0)
        self.create_subscription(Twist, 'cmd_vel/auto', lambda m: setattr(self, 'cmd', (m.linear.x, m.angular.z)), 10)
        self.status = ''
        self.create_subscription(String, 'follow/status', lambda m: setattr(self, 'status', m.data), 10)
        self.rx, self.ry, self.ryaw = 0.0, 0.0, 0.0
        self.s = 0.0                  # person path length walked
        self.seen_before, self.hidden = False, False
        self.hit_box, self.hits = False, []

    def person(self):
        pts, s = self.sc['path'], self.s
        for a, b in zip(pts, pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if s <= L:
                return (a[0] + (b[0] - a[0]) * s / L, a[1] + (b[1] - a[1]) * s / L)
            s -= L
        return pts[-1]

    def tick(self, dt, walk=True):
        vx, wz = self.cmd
        self.ryaw += wz * dt
        nx, ny = self.rx + vx * math.cos(self.ryaw) * dt, self.ry + vx * math.sin(self.ryaw) * dt
        if any(b[0] - 0.12 <= nx <= b[2] + 0.12 and b[1] - 0.12 <= ny <= b[3] + 0.12 for b in self.sc['boxes']):
            self.hit_box = True       # robot body would touch the box: stay
            self.hits.append(self.status.split()[0] if self.status else '-')
        else:
            self.rx, self.ry = nx, ny
        px, py = self.person()
        if walk:
            self.s += 0.5 * dt
        c, s_ = math.cos(self.ryaw), math.sin(self.ryaw)
        st = ControllerState(); st.mode = 'balance'; st.h = [0.18, 0.18]; st.v = vx  # noqa: E702
        self.p_state.publish(st)
        o = Odometry(); o.pose.pose.position.x, o.pose.pose.position.y = self.rx, self.ry  # noqa: E702
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.ryaw / 2), math.cos(self.ryaw / 2)
        self.p_odom.publish(o)
        # LiDAR (laser frame): boxes + the person's body (circle r 0.15), 1 deg
        lx, ly = self.rx + LASER_X * c, self.ry + LASER_X * s_
        n = 360
        ga = self.ryaw + (-math.pi + np.arange(n) * 2 * math.pi / n)[:, None]
        r = np.arange(0.12, 6.0, 0.02)[None, :]
        X, Y = lx + r * np.cos(ga), ly + r * np.sin(ga)
        hit = np.hypot(X - px, Y - py) < 0.15
        for b0, b1, b2, b3 in self.sc['boxes']:
            hit |= (X >= b0) & (X <= b2) & (Y >= b1) & (Y <= b3)
        first = np.where(hit.any(1), hit.argmax(1), -1)
        ranges = [float(r[0, i]) if i >= 0 else 12.0 for i in first]
        m = LaserScan(); m.angle_min, m.angle_increment, m.range_min, m.range_max = -math.pi, 2 * math.pi / n, 0.1, 12.0  # noqa: E702
        m.ranges = ranges
        self.p_scan.publish(m)
        # camera: visible if inside the field of view and the line of sight misses every box
        cx, cy = self.rx + CAM_X * c, self.ry + CAM_X * s_
        dx, dy = px - cx, py - cy
        b = math.atan2(dy, dx) - self.ryaw
        b = (b + math.pi) % (2 * math.pi) - math.pi
        vis = abs(b) < HFOV / 2 and not any(seg_box((cx, cy), (px, py), bx) for bx in self.sc['boxes'])
        if vis:
            d = math.hypot(dx, dy)
            half = math.degrees(math.atan2(0.2, d))
            state = 'REACQUIRED' if self.hidden else 'LOCK'
            self.hidden = False
            self.p_tg.publish(String(data=json.dumps({
                'stamp': time.time(), 'state': state, 'id': 1, 'w': 640, 'h': 480, 'box': [300, 100, 340, 400],
                'bearing_deg': math.degrees(b), 'span_deg': [math.degrees(b) + half, math.degrees(b) - half],
                'feet_row': 330.0, 'score': 0.9})))
        else:
            self.hidden = True
            self.p_tg.publish(String(data=json.dumps({'stamp': time.time(), 'state': 'LOST', 'id': 1, 'w': 640, 'h': 480,
                                                      'box': None})))
        return vis


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 else 'corner'
    rclpy.init()
    w, n = World(scene), PersonFollowCam()
    n.use_nav = False
    n.d_ref = 0.8
    n.occl_s = 1.0                    # preset 1026_1414
    n.ex_on = os.environ.get('EXPLORE', '1') == '1'   # EXPLORE=0: 예전 동작 (비교용)
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(w); ex.add_node(n)  # noqa: E702
    threading.Thread(target=ex.spin, daemon=True).start()
    dt = 0.05
    for _ in range(20):
        w.tick(dt, walk=False); time.sleep(dt)  # noqa: E702
    cli = w.create_client(SetBool, 'follow/enable')
    assert cli.wait_for_service(2.0)
    fut = cli.call_async(SetBool.Request(data=True))
    while not fut.done():
        time.sleep(0.01)
    seq, t0, lost_t, back_t = [], time.time(), None, None
    while time.time() - t0 < 45:
        vis = w.tick(dt); time.sleep(dt)  # noqa: E702
        if os.environ.get('DBG') and n.target is not None and 5.5 < time.time() - t0 < 8.0:
            print('dbg t %.1f person (%.2f, %.2f) est (%.2f, %.2f) src %s vis %s' % ((time.time() - t0,) + w.person() + tuple(n.target['w']) + (n.dist_src, vis)))
        st = w.status.split()[0] if w.status else '-'
        if not seq or seq[-1][0] != st:
            seq.append((st, round(time.time() - t0, 1), round(w.rx, 2), round(w.ry, 2)))
        if not vis and lost_t is None and len(seq) > 1:
            lost_t = time.time()
        if vis and lost_t is not None and back_t is None:
            back_t = time.time()
        if back_t and st == 'FOLLOW' and time.time() - back_t > 2.0:
            break
    ex.shutdown(); rclpy.try_shutdown()  # noqa: E702
    print(scene, 'states', seq)
    print('hidden', None if lost_t is None else round(lost_t - t0, 1), 'seen again',
          None if back_t is None else round(back_t - t0, 1), 'robot touched a box', w.hit_box, sorted(set(w.hits)))
    names = [s[0] for s in seq]
    assert lost_t is not None, 'the person was never hidden'
    assert 'EXPLORE' in names, names
    assert back_t is not None and names[-1] == 'FOLLOW', 'not found again'
    assert 'EXPLORE' not in w.hits, 'explore drove into the box'
    print('ok')


if __name__ == '__main__':
    main()
