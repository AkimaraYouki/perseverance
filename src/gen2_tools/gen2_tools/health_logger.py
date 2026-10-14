#!/usr/bin/env python3
"""Robot health log (runs in bench.launch, restarts with it): one CSV row per second + an events file.

~/gen2_ws/logs/health/<start>_health.csv  — mode/fault, pitch, IMU age, loop overruns; per motor: stale, age,
    temperature, error, current, replies/s; batteries; CAN state + error counters (bus-off, passive, warning);
    topic rates (/scan, /imu/data, camera); Jetson VDD_IN, temperatures, load, memory, Wi-Fi signal, DDS mode.
~/gen2_ws/logs/health/<start>_events.log — operator markers (gcs/marker, std_msgs/String) and only changes: mode / fault, motor error / stale / over-temperature,
    CAN state change or new bus-off / error-passive, battery low, a sensor stops, Wi-Fi lost, clock jump (no RTC:
    boots start in 1970 until NTP).

All ROS inputs are polled once per second on a side node (no 100 Hz callbacks); rates are counted from raw
(not deserialized) messages. ~1 % CPU.
"""

import csv
import json
import math
import os
import subprocess
import time

import rclpy
import rclpy.executors
from diagnostic_msgs.msg import DiagnosticArray  # noqa: F401  (kept for future fields)
from gen2_msgs.msg import ControllerState, MotorStateArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, LaserScan

MOTORS = ('wheel_l', 'wheel_r', 'leg_l', 'leg_r')
# fixed CSV columns (a row only fills what it has: inputs can be missing, e.g. right after start)
COLS = (['time', 'uptime_s', 'mode', 'fault', 'pitch_deg', 'v', 'vx_cmd', 'imu_age_ms', 'loop_dt_max_ms', 'overruns'] +
        [f'{m}_{k}' for m in MOTORS for k in ('stale', 'age_s', 'temp_c', 'err', 'cur_a', 'rx_hz')] +
        [f'{b}_{k}' for b in ('compute', 'motor') for k in ('v', 'a', 'pct', 'present')] +
        ['can_up', 'can_state', 'can_tec', 'can_rec', 'can_bus_off', 'can_passive', 'can_warning', 'can_restarts',
         'can_rx_err', 'can_tx_err', 'can_rx_drop', 'scan_hz', 'imu_hz', 'camera_hz', 'vdd_in_w', 'temp_cpu',
         'temp_gpu', 'temp_tj', 'cpu_pct', 'mem_avail_mb', 'wifi_dbm', 'wifi_ip', 'dds'])
HOME = os.path.expanduser('~')


def _read(path, default=''):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


class Health(Node):
    def __init__(self):
        super().__init__('health_logger')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.dir = os.path.expanduser(p('dir', '~/gen2_ws/logs/health'))
        self.temp_warn = p('motor_temp_warn_c', 70.0)
        self.cell_warn = {'compute': p('compute_cell_warn_v', 3.5), 'motor': p('motor_cell_warn_v', 3.5)}
        os.makedirs(self.dir, exist_ok=True)
        stamp = time.strftime('%Y-%m-%d_%H%M%S')
        self.csv_path = os.path.join(self.dir, f'{stamp}_health.csv')
        self.ev_path = os.path.join(self.dir, f'{stamp}_events.log')
        self.side = rclpy.create_node('health_logger_poll')
        last = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # matches reliable and best-effort publishers
        many = lambda n: QoSProfile(depth=n, history=HistoryPolicy.KEEP_LAST,  # noqa: E731
                                    reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = {
            'state': (self.side.create_subscription(ControllerState, 'controller/state', lambda m: None, last), ControllerState),
            'motors': (self.side.create_subscription(MotorStateArray, 'motors/state', lambda m: None, last), MotorStateArray),
            'p_compute': (self.side.create_subscription(BatteryState, 'power/compute', lambda m: None, last), BatteryState),
            'p_motor': (self.side.create_subscription(BatteryState, 'power/motor', lambda m: None, last), BatteryState),
        }
        self.rate = {
            'scan': (self.side.create_subscription(LaserScan, 'scan', lambda m: None, many(40)), LaserScan),
            'imu': (self.side.create_subscription(Imu, 'imu/data', lambda m: None, many(600)), Imu),
            'camera': (self.side.create_subscription(CompressedImage, 'camera/image_raw/compressed', lambda m: None, many(60)),
                       CompressedImage),
        }
        self.prev = {}
        self.prev_rx = {}
        self.prev_cpu = None
        self.t_wall0, self.t_mono0 = time.time(), time.monotonic()
        self.f = open(self.csv_path, 'w', newline='')
        self.w = None
        self.event(f'health_logger start (boot {_read("/proc/sys/kernel/random/boot_id")[:8]}, '
                   f'uptime {float(_read("/proc/uptime", "0 0").split()[0]):.0f} s, DDS mode {self._dds_mode()})')
        # operator notes from the GCS GUI (e.g. "jump test: step 5.6 cm" / "flat"): written to the events log
        from std_msgs.msg import String
        self.create_subscription(String, 'gcs/marker', lambda m: self.event(f'MARKER: {m.data}'), 10)
        self.create_timer(1.0, self._tick)

    # ------------------------------------------------------------------ helpers
    def event(self, text):
        line = f'{time.strftime("%F %T")}  up {float(_read("/proc/uptime", "0 0").split()[0]):8.1f}  {text}'
        with open(self.ev_path, 'a') as f:
            f.write(line + '\n')
        self.get_logger().info(text)

    def _poll(self, key):
        sub, typ = self.sub[key]
        last = None
        with sub.handle:
            while True:
                r = sub.handle.take_message(typ, False)
                if r is None:
                    break
                last = r[0] if isinstance(r, tuple) else r
        return last

    def _count(self, key):
        sub, typ = self.rate[key]
        n = 0
        with sub.handle:
            while sub.handle.take_message(typ, True) is not None:
                n += 1
        return n

    @staticmethod
    def _dds_mode():
        return '/'.join(f'{k}:{_read(os.path.join(HOME, ".ros", "gen2_dds_mode." + k), "?")}' for k in ('bench', 'control'))

    @staticmethod
    def _can():
        try:
            d = json.loads(subprocess.run(['ip', '-j', '-d', '-s', 'link', 'show', 'can0'], capture_output=True,
                                          text=True, timeout=2).stdout)[0]
            info, xs = d['linkinfo']['info_data'], d['linkinfo'].get('info_xstats', {})
            return {'can_state': info.get('state', '?'), 'can_tec': info.get('berr_counter', {}).get('tx', -1),
                    'can_rec': info.get('berr_counter', {}).get('rx', -1), 'can_bus_off': xs.get('bus_off', -1),
                    'can_passive': xs.get('error_passive', -1), 'can_warning': xs.get('error_warning', -1),
                    'can_restarts': xs.get('restarts', -1), 'can_rx_err': d['stats64']['rx']['errors'],
                    'can_tx_err': d['stats64']['tx']['errors'], 'can_rx_drop': d['stats64']['rx']['dropped'],
                    'can_up': 'UP' in d.get('flags', [])}
        except Exception:
            return {'can_state': 'missing', 'can_up': False}

    def _system(self):
        out = {}
        hw = [h for h in os.popen('ls -d /sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon* 2>/dev/null').read().split()]
        for h in hw:
            for k in range(1, 4):
                if _read(f'{h}/in{k}_label') == 'VDD_IN':
                    v, i = _read(f'{h}/in{k}_input'), _read(f'{h}/curr{k}_input')
                    if v and i:
                        out['vdd_in_w'] = round(float(v) * float(i) * 1e-6, 2)
        temps = {}
        for z in range(12):
            t = _read(f'/sys/class/thermal/thermal_zone{z}/type')
            if t in ('cpu-thermal', 'gpu-thermal', 'tj-thermal'):
                temps[t.split('-')[0]] = round(float(_read(f'/sys/class/thermal/thermal_zone{z}/temp', '0')) / 1000, 1)
        out.update({f'temp_{k}': v for k, v in temps.items()})
        cpu = [float(x) for x in _read('/proc/stat').split('\n')[0].split()[1:]]
        if self.prev_cpu:
            dt = [a - b for a, b in zip(cpu, self.prev_cpu)]
            out['cpu_pct'] = round(100 * (1 - (dt[3] + dt[4]) / max(1e-9, sum(dt))), 1)
        self.prev_cpu = cpu
        mem = dict(line.split(':', 1) for line in _read('/proc/meminfo').split('\n') if ':' in line)
        out['mem_avail_mb'] = int(mem.get('MemAvailable', '0 kB').split()[0]) // 1024
        wl = [line.split() for line in _read('/proc/net/wireless').split('\n')[2:] if line.strip()]
        out['wifi_dbm'] = float(wl[0][3].rstrip('.')) if wl else float('nan')
        out['wifi_ip'] = (os.popen("ip -4 -o addr show dev wlP1p1s0 2>/dev/null | awk '{print $4}'").read().strip()
                          or 'none')
        return out

    # ------------------------------------------------------------------ main
    def _tick(self):
        now_wall, now_mono = time.time(), time.monotonic()
        if abs((now_wall - self.t_wall0) - (now_mono - self.t_mono0)) > 30:
            self.event(f'clock jump: wall {time.strftime("%F %T")} (NTP sync; no RTC)')
            self.t_wall0, self.t_mono0 = now_wall, now_mono
        row = {'time': time.strftime('%F %T'), 'uptime_s': round(float(_read('/proc/uptime', '0 0').split()[0]), 1)}
        st = self._poll('state')
        if st is not None:
            row.update({'mode': st.mode, 'fault': st.fault.replace(',', ';'), 'pitch_deg': round(math.degrees(st.pitch), 1),
                        'v': round(st.v, 2), 'vx_cmd': round(st.vx_cmd, 2), 'imu_age_ms': round(st.imu_age_ms, 1),
                        'loop_dt_max_ms': round(st.loop_dt_max_ms, 1), 'overruns': st.overruns})
        else:
            row['mode'] = 'no_state'
        mo = self._poll('motors')
        seen = {m.name: m for m in mo.motors} if mo is not None else {}
        for name in MOTORS:
            m = seen.get(name)
            if m is None:
                row[f'{name}_stale'] = 'missing'
                continue
            rx = m.rx_count - self.prev_rx.get(name, m.rx_count)
            self.prev_rx[name] = m.rx_count
            row.update({f'{name}_stale': int(m.stale), f'{name}_age_s': round(m.age_s, 3),
                        f'{name}_temp_c': round(m.temperature_c, 1), f'{name}_err': m.error_code,
                        f'{name}_cur_a': round(m.current_a, 2), f'{name}_rx_hz': rx})
        for key, name in (('p_compute', 'compute'), ('p_motor', 'motor')):
            b = self._poll(key)
            if b is not None:
                row.update({f'{name}_v': round(b.voltage, 2), f'{name}_a': round(-b.current, 2),
                            f'{name}_pct': round(100 * b.percentage, 0) if b.percentage == b.percentage else '',
                            f'{name}_present': int(b.present)})
        row.update(self._can())
        for k in self.rate:
            row[f'{k}_hz'] = self._count(k)
        row.update(self._system())
        row['dds'] = self._dds_mode()
        self._events(row)
        if self.w is None:
            self.w = csv.DictWriter(self.f, fieldnames=COLS, extrasaction='ignore')
            self.w.writeheader()
        self.w.writerow(row)
        self.f.flush()

    def _events(self, r):
        p = self.prev

        def changed(key, text, cond=True):
            if key in r and r[key] != p.get(key, r[key]) and cond:
                self.event(text)
        if r.get('mode') != p.get('mode'):
            self.event(f'mode {p.get("mode", "-")} -> {r.get("mode")}' + (f' ({r.get("fault")})' if r.get('fault') else ''))
        elif r.get('fault') and r.get('fault') != p.get('fault'):
            self.event(f'fault/refusal: {r.get("fault")}')
        for name in MOTORS:
            s = r.get(f'{name}_stale')
            if s != p.get(f'{name}_stale', s):
                self.event(f'{name}: {"STALE/missing" if s not in (0,) else "feedback back"} ({s})')
            changed(f'{name}_err', f'{name}: drive error {r.get(f"{name}_err")}')
            t = r.get(f'{name}_temp_c', 0) or 0
            if t > self.temp_warn and (p.get(f'{name}_temp_c', 0) or 0) <= self.temp_warn:
                self.event(f'{name}: temperature {t} C > {self.temp_warn}')
        changed('can_state', f'CAN state {p.get("can_state")} -> {r.get("can_state")}')
        for k in ('can_bus_off', 'can_passive', 'can_restarts', 'can_rx_err', 'can_tx_err', 'can_rx_drop'):
            if k in r and k in p and r[k] > p[k]:
                self.event(f'CAN {k} +{r[k] - p[k]} (total {r[k]}; tec {r.get("can_tec")} rec {r.get("can_rec")})')
        for name, cells in (('compute', 3), ('motor', 6)):
            v = r.get(f'{name}_v')
            if v and v / cells < self.cell_warn[name] and (p.get(f'{name}_v') or 99) / cells >= self.cell_warn[name]:
                self.event(f'{name} battery low: {v} V ({v / cells:.2f} V/cell)')
            changed(f'{name}_present', f'{name} battery present {p.get(f"{name}_present")} -> {r.get(f"{name}_present")}')
        for k in ('scan_hz', 'imu_hz', 'camera_hz'):
            if k in r and k in p and (r[k] == 0) != (p[k] == 0):
                self.event(f'{k[:-3]}: {"STOPPED" if r[k] == 0 else "back"} ({p[k]} -> {r[k]} /s)')
        changed('wifi_ip', f'Wi-Fi IP {p.get("wifi_ip")} -> {r.get("wifi_ip")}')
        changed('dds', f'DDS mode {p.get("dds")} -> {r.get("dds")}')
        if 'overruns' in r and 'overruns' in p and r['overruns'] > p['overruns']:
            self.event(f'control loop overruns +{r["overruns"] - p["overruns"]} (max dt {r.get("loop_dt_max_ms")} ms)')
        self.prev = dict(r)


def main():
    rclpy.init()
    node = Health()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.event('health_logger stop')
        node.f.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
