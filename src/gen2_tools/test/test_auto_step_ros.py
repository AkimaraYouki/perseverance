"""ROS test of person_follow_cam auto step with fake inputs (desktop, own domain): a person 3 m ahead, a step foot at
x = 2.0 m (rays from 'person_tracker'), odometry integrating the command. Expect: FOLLOW 0.7 -> SLOW (<= 0.3) ->
STEP (balance/step_mode true, wz 0) -> blocked / back / run / fire (fake balance/step_phase 0-1-2-0) -> landing +
1.3 m -> step mode false -> FOLLOW."""
import json
import math
import os
import sys
import threading
import time

import rclpy
from gen2_msgs.msg import ControllerState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8, String
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gen2_tools.person_follow_cam import PersonFollowCam  # noqa: E402

STEP_X = 2.0


class Fake(Node):
    def __init__(self):
        super().__init__('fake')
        self.p_state = self.create_publisher(ControllerState, 'controller/state', 10)
        self.p_odom = self.create_publisher(Odometry, 'odom', 10)
        self.p_tg = self.create_publisher(String, 'people/target', 10)
        self.p_rays = self.create_publisher(String, 'step/rays', 10)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.p_phase = self.create_publisher(Int8, 'balance/step_phase', q)
        self.cmd = (0.0, 0.0)
        self.create_subscription(Twist, 'cmd_vel/auto', lambda m: setattr(self, 'cmd', (m.linear.x, m.angular.z)), 10)
        self.status = ''
        self.create_subscription(String, 'follow/status', lambda m: setattr(self, 'status', m.data), 10)
        self.step_mode, self.calls = False, []
        self.create_service(SetBool, 'balance/step_mode', self._srv)
        self.x, self.v, self.phase, self.t_block = 0.0, 0.0, 0, None
        self.on_top = False

    def _srv(self, req, res):
        self.step_mode = bool(req.data); self.calls.append((round(self.x, 2), self.step_mode))  # noqa: E702
        res.success = True
        return res

    def tick(self, dt):
        # 로봇 흉내: 턱 감지 모드면 0.14 로 접근 -> 발이 턱에 닿으면 막힘 -> 후진 0.3 m -> 재주행 -> 발사 -> 턱 위 (2 단 없음)
        vx = self.cmd[0]
        if self.step_mode and not self.on_top:
            if self.phase == 0:
                self.v = 0.14 if vx > 0.05 else 0.0
                if self.x >= STEP_X - 0.07:
                    self.v, self.phase, self.t_block = 0.0, 1, time.time()
            elif self.phase == 1:
                self.v = -0.3
                if self.x <= STEP_X - 0.37:
                    self.phase = 2
            elif self.phase == 2:
                self.v = 0.3
                if self.x >= STEP_X - 0.27:
                    self.phase, self.on_top, self.v = 0, True, 0.3     # 발사 (2 -> 0), 착지는 턱 위
                    self.x = STEP_X + 0.05
        else:
            self.v = vx if not (self.x >= STEP_X - 0.07 and not self.on_top) else 0.0
        self.x += self.v * dt
        s = ControllerState(); s.mode = 'balance'; s.h = [0.18, 0.18]; s.v = self.v  # noqa: E702
        self.p_state.publish(s)
        o = Odometry(); o.pose.pose.position.x = self.x; o.pose.pose.orientation.w = 1.0  # noqa: E702
        self.p_odom.publish(o)
        self.p_phase.publish(Int8(data=self.phase))
        self.p_tg.publish(String(data=json.dumps({'stamp': time.time(), 'state': 'LOCK', 'id': 1, 'w': 640, 'h': 480,
                                                  'box': [300, 100, 340, 400], 'bearing_deg': 0.0, 'span_deg': [2.0, -2.0],
                                                  'feet_row': 330.0, 'score': 0.9})))
        cam_x = self.x + 0.1118
        d = STEP_X - cam_x
        rays = []
        if not self.on_top and 0.55 < d < 2.5:                   # 0.55 m 안에서는 발치가 화면 아래로 빠짐
            rays = [[math.atan2(0.18 + 0.0863, d), 0.5, -0.5]]
        self.p_rays.publish(String(data=json.dumps({'stamp': time.time(), 'rays': rays})))


def main():
    os.environ['ROS_DOMAIN_ID'] = '77'
    rclpy.init()
    f, n = Fake(), PersonFollowCam()
    n.use_nav = False
    n.auto_on = True
    n.d_ref = 0.8
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(f); ex.add_node(n)  # noqa: E702
    threading.Thread(target=ex.spin, daemon=True).start()
    for _ in range(20):
        f.tick(0.033); time.sleep(0.033)  # noqa: E702
    cli = f.create_client(SetBool, 'follow/enable')
    assert cli.wait_for_service(2.0)
    fut = cli.call_async(SetBool.Request(data=True))
    while not fut.done():
        time.sleep(0.01)
    seen, t0 = [], time.time()
    while time.time() - t0 < 40 and not (f.on_top and not f.step_mode and f.x > STEP_X + 1.4):
        f.tick(0.033); time.sleep(0.033)  # noqa: E702
        a = f.status.split('[AUTO ')[-1].split(']')[0].split()[0] if '[AUTO' in f.status else '-'
        if not seen or seen[-1][0] != a:
            seen.append((a, round(f.x, 2), round(f.cmd[0], 2), round(f.cmd[1], 2)))
    ex.shutdown(); rclpy.try_shutdown()  # noqa: E702
    print('states', seen)
    print('step_mode calls', f.calls)
    assert [s[0] for s in seen][:3] == ['FOLLOW', 'SLOW', 'STEP'], seen
    assert f.calls and f.calls[0][1] is True and f.calls[-1][1] is False, f.calls
    assert f.on_top, 'never jumped'
    slow = [s for s in seen if s[0] == 'SLOW'][0]
    assert slow[2] <= 0.3 + 1e-6, slow
    print('ok')


if __name__ == '__main__':
    main()
