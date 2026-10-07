#!/usr/bin/env python3
"""IMU mounting calibration: finds how the iAHRS sits in the robot body (roll / pitch / yaw axes).

Body frame = REP-103 base_link: x forward, y left, z up. Signs: pitch + = nose DOWN, roll + = RIGHT
side down, yaw + = turning left (counter-clockwise seen from above).

Three still poses (accelerometer = gravity direction, averaged 2 s from /imu/data_raw):
  1. LEVEL       body level (flat floor / table)       -> z axis (up) and the level trim
  2. NOSE DOWN   tilt the front down 20-30 deg         -> x axis (forward)
  3. RIGHT DOWN  tilt the right side down 20-30 deg    -> checks the y axis sign (left)
The rotation is saved as `mount_rpy_deg` (imu_link in base_link) in gen2_sensors/config/iahrs.yaml;
iahrs_node then publishes /imu/data in the body frame. A live check runs at the end.
"""

import math
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

WS = os.path.expanduser(os.environ.get('GEN2_WS', '~/gen2_ws'))
CFG = os.path.join(WS, 'src/gen2_sensors/config/iahrs.yaml')


class Ros:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('imu_cal')
        self.raw, self.body = [], []
        self.lock = threading.Lock()
        self.node.create_subscription(Imu, 'imu/data_raw', lambda m: self._add(self.raw, m), qos_profile_sensor_data)
        self.node.create_subscription(Imu, 'imu/data', lambda m: self._add(self.body, m), qos_profile_sensor_data)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _add(self, buf, m):
        a, w = m.linear_acceleration, m.angular_velocity
        with self.lock:
            buf.append((time.monotonic(), a.x, a.y, a.z, w.x, w.y, w.z))
            del buf[:-4000]

    def window(self, buf, sec):
        t0 = time.monotonic()
        with self.lock:
            return np.array([r[1:] for r in buf if r[0] > t0 - sec])


def still_mean(ros, label):
    input(f'\n>>> {label} — hold STILL, then press Enter ')
    time.sleep(2.0)
    d = ros.window(ros.raw, 2.0)
    if len(d) < 200:
        print('  no /imu/data_raw (is gen2-bench running with the new iahrs_node?)'); sys.exit(1)
    a, w = d[:, :3].mean(axis=0), d[:, 3:].mean(axis=0)
    a_sd, w_max = d[:, :3].std(axis=0).max(), np.abs(d[:, 3:]).max()
    print(f'  accel {a.round(3)} |a| {np.linalg.norm(a):.3f} m/s^2, sd {a_sd:.3f}, max |gyro| {w_max:.3f} rad/s')
    if a_sd > 0.3 or w_max > 0.3:
        print('  !! not still — repeat'); return still_mean(ros, label)
    return a / np.linalg.norm(a)


def rpy_of(R):
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def save(rpy_deg):
    txt = open(CFG).read()
    stamp = time.strftime('%Y-%m-%d')
    new = f'    mount_rpy_deg: [{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}]   # imu_cal {stamp}: imu_link in base_link'
    txt, n = re.subn(r'    mount_rpy_deg: [^\n]*', new, txt)
    if n != 1:
        print('  could not find mount_rpy_deg in', CFG); return False
    open(CFG, 'w').write(txt)
    b = subprocess.run(['colcon', 'build', '--packages-select', 'gen2_sensors'], cwd=WS, capture_output=True, text=True)
    r = subprocess.run(['sudo', '-n', 'systemctl', 'restart', 'gen2-bench'], capture_output=True)
    print(f'  saved {CFG}; build {"ok" if b.returncode == 0 else "FAILED"}; '
          f'bench {"restarted" if r.returncode == 0 else "NOT restarted: sudo systemctl restart gen2-bench"}')
    return b.returncode == 0 and r.returncode == 0


def live(ros, sec=20.0):
    print('\nLive body angles from /imu/data (tilt the robot: nose down -> pitch +, right down -> roll +)')
    t_end = time.monotonic() + sec
    while time.monotonic() < t_end:
        d = ros.window(ros.body, 0.1)
        if len(d):
            ax, ay, az = d[:, :3].mean(axis=0)
            roll = math.degrees(math.atan2(ay, az))
            pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
            print(f'\r  roll {roll:+6.1f} deg   pitch {pitch:+6.1f} deg   yaw rate {d[:, 5].mean():+6.2f} rad/s   ',
                  end='', flush=True)
        time.sleep(0.1)
    print()


def main():
    print(__doc__)
    ros = Ros()
    time.sleep(1.0)
    if len(sys.argv) > 1 and sys.argv[1] == 'check':
        live(ros, 60.0); return
    z = still_mean(ros, '1/3 LEVEL: body level')
    u1 = still_mean(ros, '2/3 NOSE DOWN: tilt the FRONT down 20-30 deg')
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(z @ u1)))))
    if tilt < 10:
        print(f'  only {tilt:.1f} deg of tilt — need 10+'); return
    x = -(u1 - (u1 @ z) * z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    u2 = still_mean(ros, '3/3 RIGHT DOWN: tilt the RIGHT side down 20-30 deg')
    roll_t = math.degrees(math.atan2(float(u2 @ y), float(u2 @ z)))
    pitch_t = math.degrees(math.atan2(-float(u2 @ x), math.hypot(float(u2 @ y), float(u2 @ z))))
    print(f'  check: right-down pose reads roll {roll_t:+.1f} deg, pitch {pitch_t:+.1f} deg (expect roll +20..30, pitch ~0)')
    if roll_t < 5:
        print('  !! roll did not come out positive: was the RIGHT side lowered? Nothing saved.'); return
    R = np.vstack([x, y, z])            # rows = body axes in sensor coordinates: v_body = R v_sensor
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    rpy = [math.degrees(v) for v in rpy_of(R)]
    print(f'\n  mount (imu_link in base_link): roll {rpy[0]:+.2f}  pitch {rpy[1]:+.2f}  yaw {rpy[2]:+.2f} deg')
    print(f'  level pose in body frame: {(R @ z * 9.80665).round(3)} m/s^2 (should be 0 0 +9.81)')
    if input('  save? [yes] ') != 'yes':
        print('  not saved'); return
    if save(rpy):
        time.sleep(8.0)
        live(ros)


if __name__ == '__main__':
    main()
