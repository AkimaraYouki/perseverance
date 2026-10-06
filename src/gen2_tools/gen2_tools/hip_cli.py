#!/usr/bin/env python3
"""Hip (leg) setup CLI: joint direction, zero pose and leg height, in leg terms.

Leg = 1-DOF four-bar (docs/lab-meeting/hardware/01_cad.md, sim/model/leg_map.py): the hip motor
turns crank angle theta; CAD zero pose theta0 = 45.0 deg, usable range theta 38.0 .. 97.8 deg.
With the 140 mm wheel (R 70 mm) the motor-axle height is h = 192.5 / 206.5 / 312.5 mm at
theta min / zero / max (leg_map.h_of_theta). Joint value M = theta - theta0, sim convention: **M + = leg extends**
(wheel moves away from the body, body rises), range M -7.0 .. +52.5 deg.

    joint M [rad] = direction * raw_rad - position_offset_rad      (motors.yaml, same as MotorTester)

Commands move the leg through the guarded motor_test_node (position mode: speed limit, heartbeat
dead-man, over-speed / stale / fault aborts, 0 A at the end -> the leg goes limp afterwards).
The CLI starts its own motor_test_node with the source motors.yaml, so saved changes apply at once.

  s                       status (M, crank theta, height h, leg length)
  dir  l|r                direction check: extend +5 deg, you answer if the leg EXTENDED -> fixes sign
  zero l|r                save the current pose as the CAD zero pose (M = 0, theta 45 deg)
  down l|r|b <deg>        extend the leg (wheel goes DOWN / body up)        M += deg
  up   l|r|b <deg>        retract the leg (wheel / link goes UP to the body) M -= deg
  h    l|r|b <mm>         go to motor-axle height h [mm] (192.5 .. 312.5 with R 70 mm)
  home                    both legs to M = 0
  speed <rad/s>           joint speed for moves (default 0.2, 0.05 .. 2.0)
  q                       quit (Ctrl+C during a move = STOP)
Until a leg's direction is verified (dir), moves are limited to |M| <= 10 deg.
"""

import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from gen2_msgs.msg import MotorStateArray, MotorTestStatus
from gen2_msgs.srv import MotorTest
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

WS = os.path.expanduser(os.environ.get('GEN2_WS', '~/gen2_ws'))
CFG = os.path.join(WS, 'src/gen2_hardware/config/motors.yaml')
sys.path.insert(0, os.path.join(WS, 'sim/model'))
import leg_map  # noqa: E402

THETA0 = math.radians(45.002)                       # CAD zero pose (sim cad.py THETA0)
M_MIN, M_MAX = leg_map.THETA_MIN - THETA0, math.radians(52.5)   # -7.0 .. +52.5 deg
UNVERIFIED_LIMIT = math.radians(10.0)
SPEED = 0.2                                         # rad/s joint (slow: assembled legs); 'speed' command
HOLD_S = 1.5
LEGS = {'l': 'leg_l', 'r': 'leg_r'}


def deg(x):
    return math.degrees(x)


# ---------------------------------------------------------------- motors.yaml (text edit, keeps comments)
def load_cfg():
    with open(CFG) as f:
        m = yaml.safe_load(f)['/**']['ros__parameters']['motors']
    return {n: dict(direction=int(m[n].get('direction', 1)),
                    offset=float(m[n].get('position_offset_rad', 0.0)),
                    verified=bool(m[n].get('verified', {}).get('direction', False)))
            for n in LEGS.values() if n in m}


def save_cfg(name, direction=None, offset=None, verified=None, note=''):
    txt = open(CFG).read()
    start = txt.index(f'      {name}:\n')
    nxt = re.search(r'\n      [a-z#][^\n]*\n', txt[start + 1:])
    end = start + 1 + nxt.start() + 1 if nxt else len(txt)
    blk = txt[start:end]
    stamp = time.strftime('%Y-%m-%d')
    if direction is not None:
        blk = re.sub(r'(\n        direction: )[^\n]*',
                     rf'\g<1>{direction:<2d}              # hip_cli {stamp}: + = leg extends (sim M){note}', blk)
    if offset is not None:
        blk = re.sub(r'(\n        position_offset_rad: )[^\n]*',
                     rf'\g<1>{offset:.6f}  # hip_cli {stamp}: CAD zero pose (theta 45 deg)', blk)
    if verified is not None:
        blk = re.sub(r'(\n          direction: )(true|false)', rf'\g<1>{str(verified).lower()}', blk)
    with open(CFG, 'w') as f:
        f.write(txt[:start] + blk + txt[end:])


# ---------------------------------------------------------------- ROS side
class Ros:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('hip_cli')
        self.raw = {}
        self.status = None
        self.node.create_subscription(MotorStateArray, 'motors/state', self._on_state,
                                      qos_profile_sensor_data)
        self.node.create_subscription(MotorTestStatus, 'motor_test/status',
                                      lambda m: setattr(self, 'status', m), 10)
        self.hb = self.node.create_publisher(Empty, 'motor_test/heartbeat', 10)
        self.start_cli = self.node.create_client(MotorTest, 'motor_test/start')
        self.stop_cli = self.node.create_client(Trigger, 'motor_test/stop')
        self.node.create_timer(0.1, lambda: self.hb.publish(Empty()))   # dead-man while the CLI runs
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def _on_state(self, msg):
        for m in msg.motors:
            if m.name in LEGS.values() and not m.stale:
                self.raw[m.name] = math.radians(m.raw_position_deg)

    def call(self, cli, req, timeout=5.0):
        fut = cli.call_async(req)
        t = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < t:
            time.sleep(0.01)
        return fut.result()


class TestNode:
    """Own motor_test_node with the source motors.yaml (restart after saving)."""

    def __init__(self):
        self.p = None

    def start(self):
        self.p = subprocess.Popen(['ros2', 'run', 'gen2_hardware', 'motor_test_node', '--ros-args',
                                   '--params-file', CFG], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, start_new_session=True)

    def stop(self):
        if self.p and self.p.poll() is None:
            os.killpg(self.p.pid, signal.SIGINT)
            try:
                self.p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)
        self.p = None


class Cli:
    def __init__(self):
        self.ros = Ros()
        self.tn = TestNode()
        self.cfg = load_cfg()
        self.speed = SPEED

    # ---- state
    def M(self, name):
        c = self.cfg[name]
        raw = self.ros.raw.get(name)
        return None if raw is None else c['direction'] * raw - c['offset']

    def line(self, name):
        M = self.M(name)
        if M is None:
            return f'{name}: no feedback'
        th = THETA0 + M
        inside = leg_map.THETA_MIN <= th <= leg_map.THETA_MAX
        h = leg_map.h_of_theta(min(max(th, leg_map.THETA_MIN), leg_map.THETA_MAX))
        v = 'verified' if self.cfg[name]['verified'] else 'NOT verified (|M| <= 10 deg)'
        return (f'{name}: M {deg(M):+7.2f} deg  theta {deg(th):6.2f} deg  h {h*1000:6.1f} mm'
                f'  leg {1000*(h - leg_map.R_WHEEL):6.1f} mm{"" if inside else "  (outside 38.0..97.8 deg)"}'
                f'  | dir {self.cfg[name]["direction"]:+d} {v}')

    def status(self):
        for n in LEGS.values():
            print('  ' + self.line(n))

    # ---- motion
    def move_to(self, name, target):
        M = self.M(name)
        if M is None:
            print(f'  {name}: no feedback'); return False
        lo, hi = (M_MIN, M_MAX) if self.cfg[name]['verified'] else (-UNVERIFIED_LIMIT, UNVERIFIED_LIMIT)
        if not lo - 1e-6 <= target <= hi + 1e-6:
            print(f'  {name}: target {deg(target):+.1f} deg outside {deg(lo):+.1f} .. {deg(hi):+.1f} deg'); return False
        d = target - M
        if abs(d) < math.radians(0.2):
            return True
        r = MotorTest.Request()
        r.motor, r.mode, r.value = name, 'position', float(deg(d))
        r.speed_rad_s, r.duration_s, r.confirm_lifted = self.speed, float(abs(d) / self.speed + HOLD_S), True
        res = self.ros.call(self.ros.start_cli, r)
        if res is None or not res.accepted:
            print(f'  {name}: rejected: {None if res is None else res.message}'); return False
        time.sleep(0.3)
        try:
            while self.ros.status is not None and self.ros.status.running:
                print(f'\r  {self.line(name)}   ', end='', flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.ros.call(self.ros.stop_cli, Trigger.Request()); print('\n  STOP'); return False
        time.sleep(0.3)
        print(f'\r  {self.line(name)}   \n  -> {self.ros.status.result}')
        return self.ros.status.result == 'done'

    def legs(self, arg):
        return list(LEGS.values()) if arg == 'b' else [LEGS[arg]]

    # ---- setup
    def restart_node(self):
        self.tn.stop(); self.tn.start()
        self.ros.start_cli.wait_for_service(timeout_sec=15); time.sleep(1.0)

    def apply_saved(self):
        self.cfg = load_cfg()
        self.restart_node()
        b = subprocess.run(['colcon', 'build', '--packages-select', 'gen2_hardware'], cwd=WS,
                           capture_output=True, text=True)
        r = subprocess.run(['sudo', '-n', 'systemctl', 'restart', 'gen2-bench'], capture_output=True)
        print(f'  saved to {CFG}; build {"ok" if b.returncode == 0 else "FAILED"}, '
              f'bench {"restarted" if r.returncode == 0 else "NOT restarted (run: sudo systemctl restart gen2-bench)"}')

    def dir_check(self, name):
        M0 = self.M(name)
        if M0 is None:
            print('  no feedback'); return
        tgt = max(min(M0 + math.radians(5.0), UNVERIFIED_LIMIT), -UNVERIFIED_LIMIT)
        print(f'  {name}: moving +5 deg (M {deg(M0):+.1f} -> {deg(tgt):+.1f}) and holding — watch the leg')
        if not self.move_to(name, tgt):
            return
        ans = input('  Did the leg EXTEND (wheel moved away from the body / down)? [y/n] ').strip().lower()
        self.move_to(name, M0)
        if ans not in ('y', 'n'):
            print('  no change'); return
        c = self.cfg[name]
        if ans == 'y':
            save_cfg(name, direction=c['direction'], verified=True)
        else:   # flip the sign and keep the same physical zero pose: offset -> -offset
            save_cfg(name, direction=-c['direction'], offset=-c['offset'], verified=True,
                     note=' (flipped)')
        print(f'  {name}: direction {"kept" if ans == "y" else "FLIPPED"}, verified')
        self.apply_saved()

    def zero(self, name):
        raw = self.ros.raw.get(name)
        if raw is None:
            print('  no feedback'); return
        if input(f'  Save the CURRENT {name} pose as the CAD zero pose (theta 45 deg)? [yes] ') != 'yes':
            print('  cancelled'); return
        save_cfg(name, offset=self.cfg[name]['direction'] * raw)
        self.apply_saved()

    # ---- loop
    def run(self):
        print(__doc__)
        if input('Robot lifted / legs free / hand on E-stop? [yes] ') != 'yes':
            return
        self.tn.start()
        if not self.ros.start_cli.wait_for_service(timeout_sec=15):
            print('motor_test_node did not start (another commander on can0? close motor_test_gui / motor_cli)')
            return
        time.sleep(1.0)
        self.status()
        while True:
            try:
                parts = input('hip> ').split()
            except (EOFError, KeyboardInterrupt):
                break
            if not parts:
                continue
            c, a = parts[0], parts[1:]
            try:
                if c == 'q':
                    break
                elif c == 's':
                    self.status()
                elif c == 'dir' and a and a[0] in LEGS:
                    self.dir_check(LEGS[a[0]])
                elif c == 'zero' and a and a[0] in LEGS:
                    self.zero(LEGS[a[0]])
                elif c in ('up', 'down') and len(a) == 2 and a[0] in ('l', 'r', 'b'):
                    dd = math.radians(float(a[1])) * (1 if c == 'down' else -1)
                    for n in self.legs(a[0]):
                        M = self.M(n)
                        if M is not None:
                            self.move_to(n, M + dd)
                elif c == 'h' and len(a) == 2 and a[0] in ('l', 'r', 'b'):
                    th = float(leg_map.theta_of_h(float(a[1]) / 1000.0))
                    for n in self.legs(a[0]):
                        self.move_to(n, th - THETA0)
                elif c == 'speed' and len(a) == 1:
                    v = float(a[0])
                    if 0.05 <= v <= 2.0:
                        self.speed = v
                    print(f'  speed {self.speed:.2f} rad/s (0.05 .. 2.0)')
                elif c == 'home':
                    for n in LEGS.values():
                        self.move_to(n, 0.0)
                else:
                    print('  ? (s, dir l|r, zero l|r, up/down l|r|b deg, h l|r|b mm, home, speed rad/s, q)')
            except ValueError as e:
                print(f'  bad number: {e}')
        self.tn.stop()
        rclpy.try_shutdown()


def main():
    Cli().run()


if __name__ == '__main__':
    main()
