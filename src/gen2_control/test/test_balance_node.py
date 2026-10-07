#!/usr/bin/env python3
"""balance_node on vcan0 with 4 fake drives (ids from motors.yaml) and a fake /imu/data. No real motors.
Run: sudo ip link set vcan0 up; ROS_DOMAIN_ID=88 CYCLONEDDS_URI=<local> python3 test_balance_node.py"""
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from gen2_msgs.msg import ControllerState
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

WS = os.path.expanduser('~/gen2_ws')
MOT = os.path.join(WS, 'src/gen2_hardware/config/motors.yaml')
FAKE = os.path.join(WS, 'src/gen2_hardware/test/fake_cubemars_drive.py')
m = yaml.safe_load(open(MOT))['/**']['ros__parameters']['motors']
ids = {n: m[n]['can_id'] for n in ('wheel_l', 'wheel_r', 'leg_l', 'leg_r')}
proto = {n: m[n].get('protocol', 'servo') for n in ids}
FAKE_PROTO = {n: ('mit_legacy' if p == 'mit_legacy' else 'servo') for n, p in proto.items()}
logs = {n: tempfile.TemporaryFile(mode='w+') for n in ids}
HIP_YAML = os.path.join(tempfile.mkdtemp(), 'hip.yaml')   # a params file: -p does not override balance.yaml
open(HIP_YAML, 'w').write(f"balance:\n  ros__parameters:\n    hip_mode: {os.environ.get('HIP_MODE', 'mit')}\n    imu_shm: false\n")
drives = [subprocess.Popen([sys.executable, FAKE, '--id', str(i), '--rate', '500', '--proto', FAKE_PROTO[n]],
                          stdout=logs[n], text=True) for n, i in ids.items()]
share = os.path.join(WS, 'install/gen2_control/share/gen2_control/config')
node = subprocess.Popen(['ros2', 'run', 'gen2_control', 'balance_node', '--ros-args',
                         '--params-file', MOT, '--params-file', os.path.join(share, 'leg_table.yaml'),
                         '--params-file', os.path.join(share, 'balance.yaml'),
                         '--params-file', os.path.join(share, 'balance_tables.yaml'), '-p', 'can_interface:=vcan0',
                         '--params-file', HIP_YAML, '-r', '__node:=balance'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
rclpy.init()
n = rclpy.create_node('balance_itest')
imu_pub = n.create_publisher(Imu, 'imu/data', rclpy.qos.qos_profile_sensor_data)
hb = n.create_publisher(Empty, 'balance/heartbeat', 10)
st = {'m': None}
n.create_subscription(ControllerState, 'controller/state', lambda x: st.__setitem__('m', x), 10)
srv = {k: n.create_client(Trigger, f'balance/{k}') for k in ('stand', 'balance', 'disarm')}
state = {'pitch': 0.0, 'imu': True, 'hb': True}
res = []


def spin(sec):
    t = time.time() + sec
    while time.time() < t:
        if state['imu']:
            q = Imu()
            p = state['pitch']            # nose down = rotation about +y
            q.orientation.w, q.orientation.y = math.cos(p / 2), math.sin(p / 2)
            q.linear_acceleration.z = 9.81
            imu_pub.publish(q)
        if state['hb']:
            hb.publish(Empty())
        rclpy.spin_once(n, timeout_sec=0.004)


def call(k):
    f = srv[k].call_async(Trigger.Request())
    t = time.time() + 3
    while not f.done() and time.time() < t:
        spin(0.02)
    return f.result()


def check(name, ok, detail=''):
    res.append(ok)
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   [{detail}]' if detail else ''), flush=True)


def drive_cmds(name, fn=None):
    logs[name].seek(0)
    lines = [l for l in logs[name].read().splitlines() if l.startswith('CMD')]
    if fn == 'mit':
        return [l for l in lines if l.startswith('CMD mit')]
    return [l for l in lines if fn is None or l.startswith(f'CMD fn={fn} ')]


def ended_at_zero(name):
    c = drive_cmds(name)
    if FAKE_PROTO[name] == 'mit_legacy':
        z = drive_cmds(name, 'mit')
        return bool(c) and c[-1] == 'CMD special FD' and bool(z) and abs(float(z[-1].split('t=')[1])) < 0.01
    return bool(c) and c[-1].startswith('CMD fn=1 ') and c[-1].endswith('target=0.000')


try:
    assert srv['stand'].wait_for_service(timeout_sec=15), 'node not up'
    spin(1.5)
    state['hb'] = False; spin(0.8)
    r = call('stand')
    check('stand refused without operator heartbeat', not r.success, r.message)
    state['hb'] = True; state['pitch'] = math.radians(25); spin(0.3)
    r = call('stand')
    check('stand refused when the body is tilted 25 deg', not r.success, r.message)
    state['pitch'] = math.radians(2); spin(0.3)
    r = call('stand')
    check('stand accepted (upright, heartbeat, fresh motors/IMU)', r.success, r.message)
    spin(1.0)
    s = st['m']
    check('stand: mode stand, wheels 0 A', s.mode == 'stand' and all(c == 0 for c in s.wheel_cur_cmd),
          f'{s.mode} wheel {list(s.wheel_cur_cmd)}')
    HIP_FN = {'mit': 8, 'servo_pos': 6, 'current_pd': 1}[os.environ.get('HIP_MODE', 'mit')]
    check(f'hips get fn={HIP_FN} commands ({os.environ.get("HIP_MODE", "mit")}), wheels get current',
          all(drive_cmds(k, HIP_FN) for k in ('leg_l', 'leg_r')) and
          all(drive_cmds(k, 'mit' if FAKE_PROTO[k] == 'mit_legacy' else 1) for k in ('wheel_l', 'wheel_r')),
          str({k: (len(drive_cmds(k, HIP_FN)), len(drive_cmds(k, 'mit')), len(drive_cmds(k, 1))) for k in ids}))
    check('loop timing (no overruns, worst period < 15 ms)', s.overruns == 0 and s.loop_dt_max_ms < 15,
          f'overruns {s.overruns}, worst {s.loop_dt_max_ms:.1f} ms')
    state['hb'] = False; spin(1.0)
    s = st['m']
    check('heartbeat loss -> fault', s.mode == 'fault' and 'heartbeat' in s.fault, f'{s.mode}: {s.fault}')
    check('fault -> last command to every drive is 0 A',
          all(ended_at_zero(k) for k in ids), str({k: drive_cmds(k)[-1][-24:] if drive_cmds(k) else None for k in ids}))
    state['hb'] = True; spin(0.3)
    check('fault latches (stand refused until disarm)', not call('stand').success)
    call('disarm'); spin(0.3)
    check('stand again after disarm', call('stand').success)
    spin(0.5)
    state['pitch'] = math.radians(50); spin(0.3)
    s = st['m']
    check('tilt 50 deg -> fault', s.mode == 'fault' and 'tilt' in s.fault, f'{s.mode}: {s.fault}')
    call('disarm'); state['pitch'] = 0.0; spin(0.3)
    call('stand'); spin(0.3)
    state['imu'] = False; spin(0.3)
    s = st['m']
    check('IMU stops -> fault', s.mode == 'fault' and 'IMU' in s.fault, f'{s.mode}: {s.fault}')
    state['imu'] = True; call('disarm'); spin(0.3)
    r = call('balance'); spin(0.5)
    s = st['m']
    check('balance accepted with model tables, LQR running', r.success and s.mode == 'balance' and s.l_pend > 0.1,
          f'{r.message}, mode {s.mode}, l_pend {s.l_pend:.3f}, th_kin {s.th_kin:+.3f}')
    call('disarm'); call('stand'); spin(0.3)
    drives[0].send_signal(signal.SIGUSR1); spin(0.3)       # first drive stops uploading
    s = st['m']
    check('motor feedback stale -> fault', s.mode == 'fault' and 'stale' in s.fault, f'{s.mode}: {s.fault}')
    drives[0].send_signal(signal.SIGUSR1)
finally:
    os.killpg(node.pid, signal.SIGINT)
    try:
        node.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(node.pid, signal.SIGKILL)
    for d in drives:
        d.terminate()
    print(f'---- {sum(res)} passed, {len(res) - sum(res)} failed')
    rclpy.try_shutdown()
    sys.exit(0 if all(res) else 1)
