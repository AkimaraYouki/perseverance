#!/usr/bin/env python3
"""Game pad operator for C-WANG (runs on the desktop next to `ros2 run joy joy_node`).

sensor_msgs/Joy -> cmd_vel/teleop (cmd_mux), balance/height, balance/heartbeat (20 Hz) and the
balance/* services. Only standard messages, so it also runs on ROS 2 Humble.

  START          start: (home hips) -> stand -> balance by itself once upright
  BACK           sit down slowly, then disarm
  left stick ↕   forward / backward (full = vx_max)
  right stick ↔  turn (full = wz_max, left = +)
  RT / LT        raise / lower the body (rate input, holds when released)
  A              stop (vx = wz = 0 while held)
  B              default height
  LB + RB 0.5 s  emergency DISARM (0 A — the robot drops)
  Y              jump (not on the robot yet)
If /joy stops (pad unplugged, joy_node dead) the heartbeat stops: the robot stops, sits and disarms.

Index defaults = the user's pad (Xbox over Bluetooth, joy_node on Humble, measured 2026-10-09):
axes LX 0, LY 1, RX 2, RT 4, LT 5 (sticks: left / up = +1, triggers: 1 released .. -1 pressed);
buttons A 0, B 1, Y 4, LB 6, RB 7, BACK 10, START 11. Other pads: set the *_axis / *_button parameters
(check with `ros2 topic echo /joy`).
"""

import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, Float64
from std_srvs.srv import Trigger

H_MIN, H_MAX, H_DEFAULT = 0.1325, 0.2325, 0.1825


class PadTeleop(Node):
    def __init__(self):
        super().__init__('pad_teleop')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.vx_max = p('vx_max', 0.7)      # 0.97 (3.5 km/h) overshot to 1.28 m/s on 2026-10-09 -> 0.7 until tuned
        self.wz_max = p('wz_max', 2.0)
        self.h_rate = p('height_rate', 0.05)
        self.dead = p('deadzone', 0.08)
        self.joy_to = p('joy_timeout_s', 0.5)
        # defaults = the user's pad on the desktop (Xbox over Bluetooth, measured 2026-10-09)
        self.ax = {k: p(f'{k}_axis', v) for k, v in dict(vx=1, wz=2, lt=5, rt=4).items()}
        self.bt = {k: p(f'{k}_button', v) for k, v in dict(a=0, b=1, y=4, back=10, start=11, lb=6, rb=7).items()}
        self.joy, self.joy_t, self.prev = None, 0.0, {}
        self.h, self.estop_t = H_DEFAULT, None
        self.hb = self.create_publisher(Empty, 'balance/heartbeat', 10)
        self.cmd = self.create_publisher(Twist, 'cmd_vel/teleop', 10)
        self.hp = self.create_publisher(Float64, 'balance/height', 10)
        self.cli = {k: self.create_client(Trigger, f'balance/{k}') for k in ('start', 'sit', 'disarm')}
        self.create_subscription(Joy, 'joy', self._joy, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('pad_teleop ready: START = start, BACK = sit, LB+RB 0.5 s = disarm')

    def _joy(self, m):
        self.joy, self.joy_t = m, time.monotonic()

    def _axis(self, k):
        i = self.ax[k]
        return self.joy.axes[i] if i < len(self.joy.axes) else 0.0

    def _btn(self, k):
        i = self.bt[k]
        return bool(self.joy.buttons[i]) if i < len(self.joy.buttons) else False

    def _edge(self, k):
        now = self._btn(k)
        was = self.prev.get(k, False)
        self.prev[k] = now
        return now and not was

    def _call(self, k):
        if not self.cli[k].service_is_ready():
            self.get_logger().warn(f'balance/{k}: balance_node not reachable')
            return
        f = self.cli[k].call_async(Trigger.Request())
        f.add_done_callback(lambda fu: self.get_logger().info(f'{k}: {fu.result().message}' if fu.result() else f'{k}: no reply'))

    def _dz(self, v):
        return 0.0 if abs(v) < self.dead else (v - self.dead * (1 if v > 0 else -1)) / (1 - self.dead)

    def _tick(self):
        if self.joy is None or time.monotonic() - self.joy_t > self.joy_to:
            return          # no heartbeat: balance_node stops, sits and disarms on its own
        self.hb.publish(Empty())
        if self._edge('start'):
            self._call('start')
        if self._edge('back'):
            self._call('sit')
        if self._edge('y'):
            self.get_logger().info('jump: not implemented on the robot yet')
        if self._btn('lb') and self._btn('rb'):
            self.estop_t = self.estop_t or time.monotonic()
            if time.monotonic() - self.estop_t > 0.5:
                self._call('disarm')
                self.estop_t = time.monotonic() + 1e6    # once per press
        else:
            self.estop_t = None
        if self._edge('b'):
            self.h = H_DEFAULT
        up = (1.0 - self._axis('rt')) / 2.0 - (1.0 - self._axis('lt')) / 2.0
        self.h = min(H_MAX, max(H_MIN, self.h + self.h_rate * 0.05 * up))
        t = Twist()
        if not self._btn('a'):
            t.linear.x = float(self.vx_max * self._dz(self._axis('vx')))
            t.angular.z = float(self.wz_max * self._dz(self._axis('wz')))
        self.cmd.publish(t)
        self.hp.publish(Float64(data=float(self.h)))


def main():
    rclpy.init()
    try:
        rclpy.spin(PadTeleop())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
