"""hub_cli — interactive ESP32 sensor hub test / calibration tool (read-only, uses ROS topics).

Needs the sensor_hub node running (bench service), because the node owns the serial port.
"""
import math
import select
import sys
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from gen2_msgs.msg import GnssPvt
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import BatteryState, MagneticField

HELP = """
=== Gen2 sensor hub CLI ===
  s                          snapshot
  w                          live watch (Enter to stop)
  z <compute|motor>          current zero calibration (disconnect the LOAD first, battery on)
  v <compute|motor> <volts>  voltage calibration against a multimeter reading
  q                          quit
"""


class Hub:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.diag = {}
        self.count = {'compute': 0, 'motor': 0, 'mag': 0, 'gps': 0}
        self.batt = {}
        self.gps = None
        self.mag = None
        node.create_subscription(DiagnosticArray, 'diagnostics', self._diag, 20)
        node.create_subscription(BatteryState, 'power/compute', lambda m: self._b('compute', m), 20)
        node.create_subscription(BatteryState, 'power/motor', lambda m: self._b('motor', m), 20)
        node.create_subscription(MagneticField, 'gps/mag', self._mag, 20)
        node.create_subscription(GnssPvt, 'gps/nav_pvt', self._gps, 20)

    def _diag(self, a):
        with self.lock:
            for s in a.status:
                if 'sensor_hub' in s.name:
                    lvl = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
                    self.diag[s.name.split('sensor_hub: ')[-1]] = (
                        time.monotonic(), lvl, s.message, {kv.key: kv.value for kv in s.values})

    def _b(self, k, m):
        with self.lock:
            self.batt[k] = m
            self.count[k] += 1

    def _mag(self, m):
        with self.lock:
            self.mag = m
            self.count['mag'] += 1

    def _gps(self, m):
        with self.lock:
            self.gps = m
            self.count['gps'] += 1

    def d(self, name, key, default=None):
        with self.lock:
            e = self.diag.get(name)
        return e[3].get(key, default) if e else default


LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def snapshot(hub):
    with hub.lock:
        diag = dict(hub.diag)
        gps, mag = hub.gps, hub.mag
    if not diag:
        print('no /diagnostics from sensor_hub — is the bench service running?')
        return
    now = time.monotonic()
    for name in ('link', 'power compute', 'power motor', 'gnss', 'magnetometer'):
        e = diag.get(name)
        if not e:
            print(f'[{name:13s}] --')
            continue
        t, lvl, msg, kv = e
        age = now - t
        print(f'[{name:13s}] {LEVEL.get(lvl, lvl):5s} {msg}   (diag age {age:.1f}s)')
        if name == 'link':
            print('   ' + '  '.join(f"{k}={kv.get(k)}" for k in (
                'frame_rate_hz', 'crc_errors', 'sequence_drops', 'esp_resets', 'disconnects',
                'esp_usb_drops_since_connect', 'rel_latency_mean_ms', 'rel_latency_max_ms')))
        elif name.startswith('power'):
            print('   ' + '  '.join(f"{k}={kv.get(k)}" for k in (
                'voltage_v', 'current_a', 'power_w', 'cell_voltage_v', 'present', 'energy_wh',
                'raw_mV', 'raw_mA')))
        elif name == 'gnss':
            print('   ' + '  '.join(f"{k}={kv.get(k)}" for k in (
                'epoch_rate_hz', 'fix_type', 'gnss_fix_ok', 'num_sv', 'h_acc_m', 'pdop',
                'ubx_checksum_errors')))
        else:
            print('   ' + '  '.join(f"{k}={kv.get(k)}" for k in ('sample_rate_hz', 'raw_x', 'raw_y', 'raw_z')))
    if gps is not None:
        print(f'   last epoch iTOW {gps.itow_ms}  lat {gps.latitude_deg:.7f}  lon {gps.longitude_deg:.7f}'
              f'  hMSL {gps.height_msl_m:.1f} m  v(NED) {gps.vel_north_mps:.2f},{gps.vel_east_mps:.2f},'
              f'{gps.vel_down_mps:.2f}  head {gps.heading_of_motion_deg:.1f}°')
    if mag is not None:
        f = mag.magnetic_field
        print(f'   |B| = {math.sqrt(f.x**2 + f.y**2 + f.z**2) * 1e6:.1f} uT (uncalibrated; Earth ~50 uT)')


def enter_pressed():
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def watch(hub):
    print('(Enter to stop)')
    last = {}
    t0 = time.monotonic()
    with hub.lock:
        last = dict(hub.count)
    while not enter_pressed():
        time.sleep(0.5)
        dt = time.monotonic() - t0
        t0 = time.monotonic()
        with hub.lock:
            cnt = dict(hub.count)
            bc, bm = hub.batt.get('compute'), hub.batt.get('motor')
        rates = {k: (cnt[k] - last.get(k, 0)) / dt for k in cnt}
        last = cnt

        def b(m):
            if m is None:
                return '   --   '
            return f"{m.voltage:5.2f}V {-m.current:6.2f}A {'on ' if m.present else 'off'}"
        sys.stdout.write(
            f"\rpower {rates['compute']:5.1f}Hz  COMP {b(bc)}  MOT {b(bm)}  mag {rates['mag']:4.1f}Hz"
            f"  gps {rates['gps']:3.1f}Hz  crc {hub.d('link', 'crc_errors', '-')}"
            f"  drops {hub.d('link', 'sequence_drops', '-')}   ")
        sys.stdout.flush()
    print()


def average(hub, which, key, seconds=5.0):
    """Average a raw diagnostic value (published 1 Hz) over `seconds`."""
    name = f'power {which}'
    vals, seen = [], None
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        with hub.lock:
            e = hub.diag.get(name)
        if e and e[0] != seen:
            seen = e[0]
            try:
                vals.append(float(e[3][key]))
            except (KeyError, ValueError):
                pass
        time.sleep(0.05)
    return sum(vals) / len(vals) if vals else None, len(vals)


def calibrate_zero(hub, which):
    print(f'Averaging raw {which} current for 5 s — the {which} LOAD must be disconnected...')
    raw_ma, n = average(hub, which, 'raw_mA')
    if raw_ma is None:
        print('no data')
        return
    print(f'   {n} samples, mean raw current {raw_ma / 1000:.3f} A')
    print(f'=> set in gen2_sensor_hub/config/sensor_hub.yaml:\n     {which}:\n       current_offset_a: {raw_ma / 1000:.3f}')


def calibrate_voltage(hub, which, meter_v):
    print(f'Averaging raw {which} voltage for 5 s...')
    raw_mv, n = average(hub, which, 'raw_mV')
    if not raw_mv:
        print('no data')
        return
    scale = meter_v / (raw_mv / 1000.0)
    print(f'   {n} samples, raw {raw_mv / 1000:.3f} V, meter {meter_v:.3f} V')
    print(f'=> set in gen2_sensor_hub/config/sensor_hub.yaml:\n     {which}:\n       voltage_scale: {scale:.4f}')
    if not 0.8 < scale < 1.2:
        print('   WARNING: scale is far from 1 — check wiring / which PM02 is which before using it.')


def main():
    rclpy.init()
    node = rclpy.create_node('hub_cli')
    hub = Hub(node)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    print(HELP)
    t_end = time.monotonic() + 6.0
    while time.monotonic() < t_end and len(hub.diag) < 5:
        time.sleep(0.1)
    snapshot(hub)
    try:
        while True:
            try:
                line = input('\nhub> ').split()
            except EOFError:
                break
            if not line:
                continue
            c = line[0]
            if c == 'q':
                break
            elif c == 's':
                snapshot(hub)
            elif c == 'w':
                watch(hub)
            elif c == 'z' and len(line) == 2 and line[1] in ('compute', 'motor'):
                calibrate_zero(hub, line[1])
            elif c == 'v' and len(line) == 3 and line[1] in ('compute', 'motor'):
                try:
                    calibrate_voltage(hub, line[1], float(line[2]))
                except ValueError:
                    print('usage: v <compute|motor> <volts>')
            else:
                print(HELP)
    except KeyboardInterrupt:
        pass
    ex.shutdown(timeout_sec=2.0)
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
