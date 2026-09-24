"""bag_to_csv — export a gen2 bag to CSV for sim<->real comparison.

Column names follow sim/isaaclab/scripts/play_log.py (t, q, qd, tau, ...), one column per axis:
  motors.csv : t, q_<motor>[rad], qd_<motor>[rad/s], cur_<motor>[A], tau_<motor>[Nm], kt_<motor>,
               temp_<motor>[C], err_<motor>, age_<motor>[s]          (one row per /motors/state)
  imu.csv    : t, qx qy qz qw, roll pitch yaw [rad], angvel_x/y/z [rad/s], acc_x/y/z [m/s^2]
  power.csv  : t, v_compute, i_compute, v_motor, i_motor
  can.csv    : t, rx_frames, rx_error_frames, rx_unknown, tx_frames (from /diagnostics 'can: bus')
  trim.txt   : means over the first --still seconds (static-lean trim, motor zero check)

usage: ros2 run gen2_tools bag_to_csv <bag_dir> [--out DIR] [--start S] [--end S] [--still 5]
"""
import argparse
import csv
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default=None)
    ap.add_argument('--start', type=float, default=0.0, help='seconds from bag start')
    ap.add_argument('--end', type=float, default=float('inf'))
    ap.add_argument('--still', type=float, default=5.0, help='initial still period for trim')
    a = ap.parse_args()
    out = a.out or os.path.join(a.bag.rstrip('/') + '_csv')
    os.makedirs(out, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {'/motors/state', '/imu/data', '/power/compute', '/power/motor', '/diagnostics'}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in wanted if t in types]))

    t0 = None
    motor_rows, motor_names = [], []
    imu_rows, power, can_rows = [], {}, []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t0 = t_ns if t0 is None else t0
        t = (t_ns - t0) * 1e-9
        if t < a.start or t > a.end:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/motors/state':
            row = {'t': t}
            for m in msg.motors:
                if m.name not in motor_names:
                    motor_names.append(m.name)
                n = m.name
                row.update({f'q_{n}': m.position_rad, f'qd_{n}': m.velocity_rad_s,
                            f'cur_{n}': m.current_a, f'tau_{n}': m.torque_nm,
                            f'kt_{n}': getattr(m, 'kt_nm_per_a', float('nan')),
                            f'temp_{n}': m.temperature_c, f'err_{n}': m.error_code,
                            f'age_{n}': m.age_s})
            motor_rows.append(row)
        elif topic == '/imu/data':
            o, w, acc = msg.orientation, msg.angular_velocity, msg.linear_acceleration
            r, p, y = quat_to_rpy(o.x, o.y, o.z, o.w)
            imu_rows.append([t, o.x, o.y, o.z, o.w, r, p, y, w.x, w.y, w.z, acc.x, acc.y, acc.z])
        elif topic in ('/power/compute', '/power/motor'):
            k = 'compute' if topic.endswith('compute') else 'motor'
            power.setdefault(round(t, 2), {})[k] = (msg.voltage, -msg.current)
        elif topic == '/diagnostics':
            for st in msg.status:
                if st.name.endswith('can: bus'):
                    kv = {x.key: x.value for x in st.values}
                    can_rows.append([t] + [kv.get(k, '') for k in
                                           ('rx_frames', 'rx_error_frames', 'rx_unknown', 'tx_frames')])

    def write(name, header, rows):
        if not rows:
            return
        with open(os.path.join(out, name), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
        print(f'{name}: {len(rows)} rows')

    mcols = ['t'] + [f'{k}_{n}' for n in motor_names
                     for k in ('q', 'qd', 'cur', 'tau', 'kt', 'temp', 'err', 'age')]
    write('motors.csv', mcols, [[r.get(c, '') for c in mcols] for r in motor_rows])
    write('imu.csv', ['t', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw', 'angvel_x', 'angvel_y',
                      'angvel_z', 'acc_x', 'acc_y', 'acc_z'], imu_rows)
    write('power.csv', ['t', 'v_compute', 'i_compute', 'v_motor', 'i_motor'],
          [[t, *p.get('compute', ('', '')), *p.get('motor', ('', ''))] for t, p in sorted(power.items())])
    write('can.csv', ['t', 'rx_frames', 'rx_error_frames', 'rx_unknown', 'tx_frames'], can_rows)

    lines = [f'bag: {a.bag}', f'still window: first {a.still:.1f} s']
    still = [r for r in motor_rows if r['t'] <= a.still]
    for n in motor_names:
        qs = [r[f'q_{n}'] for r in still if f'q_{n}' in r and math.isfinite(r[f'q_{n}'])]
        if qs:
            lines.append(f'{n}: mean q {sum(qs) / len(qs):+.5f} rad ({math.degrees(sum(qs) / len(qs)):+.3f} deg), n={len(qs)}')
    istill = [r for r in imu_rows if r[0] <= a.still]
    if istill:
        mp = sum(r[6] for r in istill) / len(istill)
        lines.append(f'imu: mean pitch {mp:+.5f} rad ({math.degrees(mp):+.3f} deg) = static trim, n={len(istill)}')
    else:
        lines.append('imu: no /imu/data in the still window')
    with open(os.path.join(out, 'trim.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(f'-> {out}')


if __name__ == '__main__':
    main()
