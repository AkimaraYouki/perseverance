#!/usr/bin/env python3
"""Bench profile pass/fail check. Listens for a few seconds, then prints PASS/FAIL lines."""
import glob
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from gen2_msgs.msg import MotorStateArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


def main():
    import os
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if 'gen2_bringup' not in uri:
        print('WARN  CYCLONEDDS_URI is not the gen2 config — robot topics may be invisible.\n'
              '      open a new terminal or: export CYCLONEDDS_URI=file://$HOME/gen2_ws/src/'
              'gen2_bringup/config/cyclonedds.xml')
    rclpy.init()
    n = rclpy.create_node('gen2_bench_check')
    cnt = {'hub': 0, 'mot': 0}
    last = {'motors': None}
    diag = {}

    def on_hub(_):
        cnt['hub'] += 1

    def on_mot(m):
        cnt['mot'] += 1
        last['motors'] = m

    def on_diag(a):
        for s in a.status:
            diag[s.name] = (s.level, s.message, {kv.key: kv.value for kv in s.values})

    n.create_subscription(BatteryState, 'power/compute', on_hub, 50)
    n.create_subscription(MotorStateArray, 'motors/state', on_mot, qos_profile_sensor_data)
    n.create_subscription(DiagnosticArray, 'diagnostics', on_diag, 50)
    time.sleep(1.0)
    for k in cnt:
        cnt[k] = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.monotonic() - t0

    results = []

    def chk(name, ok):
        results.append(ok)
        print(('PASS  ' if ok else 'FAIL  ') + name)

    def dval(suffix, key):
        for name, (_, _, kv) in diag.items():
            if name.endswith(suffix) and key in kv:
                return kv[key]
        return None

    hub_hz, mot_hz = cnt['hub'] / dt, cnt['mot'] / dt
    chk(f'can0 is up', open('/sys/class/net/can0/operstate').read().strip() == 'up')
    chk('ESP32 by-id device present', bool(glob.glob('/dev/serial/by-id/usb-Espressif*')))
    chk(f'sensor hub >= 90 Hz (got {hub_hz:.1f})', hub_hz >= 90)
    chk(f'motor state >= 90 Hz (got {mot_hz:.1f})', mot_hz >= 90)
    m = last['motors']
    chk('motor feedback fresh, no drive faults',
        m is not None and len(m.motors) > 0 and
        all((not x.stale) and x.error_code == 0 for x in m.motors))
    chk('motor monitor is read-only (tx_enabled False, tx_frames 0)',
        dval('can: bus', 'tx_enabled') == 'False' and dval('can: bus', 'tx_frames') == '0')
    chk('CAN error frames == 0', dval('can: bus', 'rx_error_frames') == '0')
    chk('sensor hub CRC errors == 0', dval('sensor_hub: link', 'crc_errors') == '0')
    chk('sensor hub sequence drops == 0', dval('sensor_hub: link', 'sequence_drops') == '0')
    print(f'---- {sum(results)} passed, {len(results) - sum(results)} failed')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
