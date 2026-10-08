#!/usr/bin/env python3
"""Operator console for gen2_control balance_node.

Sends balance/heartbeat at 20 Hz while running (balance_node stops and sits if this console stops) and
cmd_vel/teleop (cmd_mux forwards it to cmd_vel with the LiDAR safety limit; run balance.launch.py).
Keys (no Enter needed):
  t        stand   (hips hold the IDLE height, wheels off)  — hold the robot!
  b        balance (wheel LQR + leg VMC)
  SPACE/x  DISARM  (0 A everywhere)        q  quit (disarms)
  w / s    forward / backward speed +-0.1 m/s     a / d   turn left / right +-0.3 rad/s
  r / f    body height +-10 mm (balance only, 133..233 mm)
  0        zero the speed / turn command
Unplugging the battery is the hardware E-stop.
"""
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from gen2_msgs.msg import ControllerState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float64
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    n = rclpy.create_node('balance_cli')
    hb = n.create_publisher(Empty, 'balance/heartbeat', 10)
    cmd = n.create_publisher(Twist, 'cmd_vel/teleop', 10)   # via cmd_mux (LiDAR safety) to balance_node
    hp = n.create_publisher(Float64, 'balance/height', 10)
    st = {'m': None}
    n.create_subscription(ControllerState, 'controller/state', lambda m: st.__setitem__('m', m), 10)
    cli = {k: n.create_client(Trigger, f'balance/{k}') for k in ('stand', 'balance', 'disarm')}
    vx, wz, h = 0.0, 0.0, 0.1825
    run = [True]

    def tick():
        while run[0]:
            hb.publish(Empty())
            t = Twist(); t.linear.x, t.angular.z = vx, wz
            cmd.publish(t)
            hp.publish(Float64(data=h))
            time.sleep(0.05)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    threading.Thread(target=tick, daemon=True).start()

    def call(k):
        if not cli[k].wait_for_service(timeout_sec=1.0):
            return 'balance_node not running'
        f = cli[k].call_async(Trigger.Request())
        t = time.time() + 2
        while not f.done() and time.time() < t:
            time.sleep(0.01)
        return f.result().message if f.done() else 'timeout'

    print(__doc__)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    msg = ''
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                c = sys.stdin.read(1)
                if c == 'q':
                    break
                if c in (' ', 'x'):
                    vx = wz = 0.0; msg = call('disarm')
                elif c == 't':
                    msg = call('stand')
                elif c == 'b':
                    msg = call('balance')
                elif c == 'w':
                    vx = round(vx + 0.1, 2)
                elif c == 's':
                    vx = round(vx - 0.1, 2)
                elif c == 'a':
                    wz = round(wz + 0.3, 2)
                elif c == 'd':
                    wz = round(wz - 0.3, 2)
                elif c in ('r', 'f'):
                    h = round(min(0.2325, max(0.1325, h + (0.01 if c == 'r' else -0.01))), 4)
                elif c == '0':
                    vx = wz = 0.0
            m = st['m']
            if m is None:
                line = 'no controller/state'
            else:
                line = (f'{m.mode:8s} pitch {m.pitch*57.3:+5.1f} roll {m.roll*57.3:+5.1f} v {m.v:+.2f} '
                        f'h {m.h[0]*1000:5.1f}/{m.h[1]*1000:5.1f} hipI {m.hip_cur_cmd[0]:+5.2f}/{m.hip_cur_cmd[1]:+5.2f} '
                        f'whlI {m.wheel_cur_cmd[0]:+5.2f}/{m.wheel_cur_cmd[1]:+5.2f} cmd {vx:+.1f} {wz:+.1f} h {h*1000:.0f} '
                        f'{m.fault[:40]}')
            print(f'\r{line}  | {msg[:50]}' + ' ' * 5, end='', flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print('\n' + call('disarm'))
        run[0] = False
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
