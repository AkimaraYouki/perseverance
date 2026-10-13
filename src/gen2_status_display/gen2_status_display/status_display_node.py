"""ROS 2 node: robot status on the ST7789 LCD. Display only; no control authority.

Subscribes only to low-rate topics (/diagnostics ~1 Hz per updater, /robot/state) so this Python
process stays cheap. Values shown are exactly what the drivers report in diagnostics.
"""
import math
import os
import time

import rclpy
from PIL import ImageChops
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from std_msgs.msg import String

from . import splash
from .render import BAD, DIM, OK, USED, WARN, render
from .sysinfo import SysInfo


def _f(kv, key):
    try:
        v = float(kv[key])
        return v if math.isfinite(v) else None
    except (KeyError, ValueError):
        return None


def _finf(kv, key):
    try:
        v = float(kv[key])
        return None if math.isnan(v) else v
    except (KeyError, ValueError):
        return None


class StatusDisplay(Node):
    def __init__(self):
        super().__init__('status_display')
        p = self.declare_parameter
        self.data_hz = p('data_hz', 1.0).value         # status gathering (diagnostics ~1 Hz)
        self.boot_anim = p('boot_animation', True).value
        self.boot_hold_s = p('boot_hold_s', 0.8).value  # keep the final splash frame this long
        self.frame_hz = p('frame_hz', 12.0).value      # splash frames; blink phases use 4 Hz
        self.timeout = p('stale_timeout_s', 3.0).value
        self.lcd_cfg = dict(
            spi_dev=p('spi_device', '/dev/spidev0.0').value,
            speed_hz=p('spi_speed_hz', 20_000_000).value,
            dc=p('pin_dc', 22).value, rst=p('pin_rst', 15).value, bl=p('pin_bl', 18).value,
            invert=p('invert_colors', True).value, bgr=p('bgr', False).value,
            rotate_180=p('rotate_180', False).value)
        self.snapshot = p('snapshot_path', '').value  # e.g. /tmp/gen2_lcd.png for remote viewing
        self.pc_factor = p('pc_power_factor', 1.08).value   # computer W = factor x VDD_IN + offset
        self.pc_offset = p('pc_power_offset_w', 3.93).value  # (Rigol 2026-10-12; carrier, LiDAR, camera, Wi-Fi, LCD)
        self.motors_expected = p('motors_expected', 4).value
        self.sys = SysInfo(p('wifi_iface', 'auto').value, p('eth_iface', 'auto').value)
        self.diag = {}  # name -> (recv_monotonic, level, message, {key: value})
        self.state, self.state_t = None, 0.0
        self.create_subscription(DiagnosticArray, 'diagnostics', self._on_diag, 20)
        self.create_subscription(String, 'robot/state', self._on_state, 10)
        # balance_node mode (controller/state is 100 Hz: polled on a side node, no per-message callbacks)
        from gen2_msgs.msg import ControllerState
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self._cs_type = ControllerState
        self._side = rclpy.create_node('status_display_poll')
        self._cs_sub = self._side.create_subscription(
            ControllerState, 'controller/state', lambda m: None, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._cs, self._cs_t = None, 0.0
        self.lcd = None
        self._last_status = None
        self._lcd_retry = 0.0
        self._shown = None          # PIL image currently on the panel
        self._ensure_lcd()          # init the panel first so the boot animation starts at t=0
        self.t0 = time.monotonic()
        self._next_data = 0.0
        self._phase = None
        self._used_fast = True
        self._timer = self.create_timer(1.0 / self.frame_hz, self._frame)
        self._slow_timer = False

    def _on_diag(self, msg):
        now = time.monotonic()
        for s in msg.status:
            self.diag[s.name] = (now, s.level, s.message, {kv.key: kv.value for kv in s.values})

    def _on_state(self, msg):
        self.state, self.state_t = msg.data, time.monotonic()

    def _find(self, suffix):
        now = time.monotonic()
        for name, (t, lvl, msg, kv) in self.diag.items():
            if name.endswith(suffix) and now - t < self.timeout:
                return lvl, msg, kv
        return None

    def _status(self):
        now = time.monotonic()
        motors = []
        for name, (t, lvl, msg, kv) in sorted(self.diag.items()):
            if ': motor: ' in name or name.startswith('motor: '):
                if kv.get('gone') == 'True':
                    continue  # unconfigured drive that disappeared
                fresh = now - t < self.timeout
                motors.append(dict(
                    name=name.split('motor: ')[-1], fresh=fresh,
                    configured=kv.get('configured', 'True') != 'False',
                    stale=lvl == DiagnosticStatus.ERROR and 'stale' in msg or 'no feedback' in msg,
                    pos_deg=_f(kv, 'position_deg'), current=_f(kv, 'current_a'),
                    temp=_f(kv, 'temperature_c'), error_code=int(_f(kv, 'error_code') or 0),
                    error_text=msg))
        # configured motors first (config order ~ name), then unconfigured drives by CAN id
        motors.sort(key=lambda m: (not m['configured'], m['name']))
        sy = self.sys.sample()
        jw = sy.get('jetson_w')
        sy['pc_w_est'] = jw * self.pc_factor + self.pc_offset if jw is not None else None
        try:
            uptime = float(open('/proc/uptime').read().split()[0])
        except (OSError, ValueError):
            uptime = None
        modes = []
        for k in ('bench', 'control'):
            try:
                with open(os.path.expanduser(f'~/.ros/gen2_dds_mode.{k}')) as f:
                    modes.append(f.read().strip())
            except OSError:
                pass
        dds = ('local' if 'local' in modes else 'wifi') if modes else None
        can = self._find('can: bus')
        last = None
        with self._cs_sub.handle:
            while True:
                r = self._cs_sub.handle.take_message(self._cs_type, False)
                if r is None:
                    break
                last = r[0] if isinstance(r, tuple) else r
        if last is not None:
            self._cs, self._cs_t = last, now
        state = self.state if self.state and now - self.state_t < 2.0 else None
        if state is None and self._cs is not None and now - self._cs_t < 2.0:
            m = self._cs.mode
            state = f'FAULT {self._cs.fault[:18]}' if m == 'fault' else m.upper()
        return dict(robot_state=state, wall_time=time.time(), sys=sy, uptime_s=uptime, dds=dds,
                    motors=motors,
                    imu=self._device('imu', None, 'not connected'),
                    lidar=self._device('lidar', sy.get('lidar_port'), 'port, no driver'),
                    camera=self._device('camera', sy.get('video0'), '/dev/video0',
                                        missing='no /dev/video0', missing_col=BAD),
                    can_rate=_f(can[2], 'rx_rate_hz') if can else None, can_expected=True,
                    motors_expected=self.motors_expected)

    def _device(self, key, present, present_text, missing='n/a', missing_col=DIM):
        # A driver's own diagnostic ("<key>: ...") wins; otherwise fall back to device presence.
        for name, (t, lvl, msg, kv) in self.diag.items():
            if name.split(': ')[-2:-1] == [key] or f': {key}' in name or name.startswith(key + ':'):
                if time.monotonic() - t < self.timeout:
                    col = OK if lvl == DiagnosticStatus.OK else WARN if lvl == DiagnosticStatus.WARN else BAD
                    return dict(col=col, text=msg[:16])
        if present is None:
            return dict(col=DIM, text=present_text)
        return dict(col=DIM, text=present_text) if present else dict(col=missing_col, text=missing)

    def _ensure_lcd(self):
        if self.lcd is not None:
            return True
        if time.monotonic() < self._lcd_retry:
            return False
        try:
            from .st7789 import ST7789
            self.lcd = ST7789(**self.lcd_cfg)
            self._shown = None
            self.get_logger().info('LCD initialised')
            return True
        except Exception as e:  # keep running without the screen
            self.get_logger().error(f'LCD init failed: {e}; retrying in 5 s')
            self._lcd_retry = time.monotonic() + 5.0
            return False

    def _lcd_do(self, fn, *args):
        try:
            fn(*args)
        except Exception as e:
            self.get_logger().error(f'LCD write failed: {e}')
            self.lcd.close()
            self.lcd = None
            self._lcd_retry = time.monotonic() + 2.0

    def _push(self, img, snapshot=True):
        """Send only the part of the panel that changed since the last frame."""
        if not self._ensure_lcd():
            return
        if self._shown is None:
            self._lcd_do(self.lcd.show, img)
        else:
            box = ImageChops.difference(img, self._shown).getbbox()
            if box is None:
                return
            self._lcd_do(self.lcd.show_region, img, box)
        if self.lcd is not None:
            self._shown = img
        if self.snapshot and snapshot:
            try:
                tmp = self.snapshot + '.tmp.png'
                img.save(tmp)
                os.replace(tmp, self.snapshot)  # atomic: readers never see a partial file
            except OSError:
                pass

    def _frame(self):
        now = time.monotonic()
        t = now - self.t0
        if self.boot_anim and t < splash.DURATION_S + self.boot_hold_s:
            self._push(splash.frame(t), snapshot=False)
            return
        fresh = False
        if not self._slow_timer:   # boot animation done: 4 Hz is enough for the blink phases
            self._slow_timer = True
            self._timer.cancel()
            self._timer = self.create_timer(0.25, self._frame)
        if now >= self._next_data or self._last_status is None:
            self._next_data = now + 1.0 / self.data_hz
            self._last_status = self._status()
            fresh = True
        # 4 Hz blink clock: fast = 2 Hz (errors), slow = 1 Hz (warnings, clock colon).
        # Redraw only when a phase the screen actually uses has changed, or data is new.
        q = int(now * 4)
        fast, slow = q % 2 == 0, (q // 2) % 2 == 0
        key = (fast if self._used_fast else None, slow)
        if key == self._phase and not fresh:
            return
        self._phase = key
        img = render(self._last_status, fast=fast, slow=slow)
        self._used_fast = USED['fast']
        self._push(img, snapshot=fresh)

    def shutdown_screen(self):
        if self.lcd is None or self._last_status is None:
            return
        try:  # reuse the last status: no probes / subprocesses while shutting down
            s = dict(self._last_status)
            s['robot_state'] = 'DISPLAY OFFLINE'
            self.lcd.show(render(s))
        except BaseException:  # a second Ctrl+C / SIGINT must not produce a traceback
            pass


def main():
    rclpy.init()
    node = StatusDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:  # a second SIGINT during teardown must not print a traceback
            node.shutdown_screen()
            node.destroy_node()
            rclpy.try_shutdown()
        except BaseException:
            pass
