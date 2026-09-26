#!/usr/bin/env python3
"""Integration test: motor_test_node against fake_cubemars_drive.py on vcan0 (no real motor)."""
import os
import signal
import subprocess
import sys
import time

import rclpy
from gen2_msgs.msg import MotorTestStatus
from gen2_msgs.srv import MotorTest
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
drive = subprocess.Popen([sys.executable, os.path.join(HERE, 'fake_cubemars_drive.py')],
                         stdout=subprocess.PIPE, text=True)
cfg = sys.argv[1]
node_p = subprocess.Popen(['ros2', 'run', 'gen2_hardware', 'motor_test_node', '--ros-args',
                           '--params-file', cfg, '-p', 'can_interface:=vcan0'],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                          start_new_session=True)  # own process group: ros2 run + the node
rclpy.init()
n = rclpy.create_node('motor_test_itest')
hb = n.create_publisher(Empty, 'motor_test/heartbeat', 10)
status = {'m': None}
n.create_subscription(MotorTestStatus, 'motor_test/status', lambda m: status.__setitem__('m', m), 10)
start = n.create_client(MotorTest, 'motor_test/start')
stop = n.create_client(Trigger, 'motor_test/stop')
results = []


def spin(sec, beat=True):
    t = time.monotonic() + sec
    while time.monotonic() < t:
        if beat:
            hb.publish(Empty())
        rclpy.spin_once(n, timeout_sec=0.05)


def call(cli, req):
    f = cli.call_async(req)
    t = time.monotonic() + 5
    while not f.done() and time.monotonic() < t:
        hb.publish(Empty())
        rclpy.spin_once(n, timeout_sec=0.05)
    return f.result()


def req(mode='current', value=0.5, dur=1.0, confirm=True, speed=0.0):
    r = MotorTest.Request()
    r.motor, r.mode, r.value, r.duration_s, r.confirm_lifted = 'ak45_a', mode, value, dur, confirm
    r.speed_rad_s = speed
    return r


def wait_done(beat=True, timeout=6):
    t = time.monotonic() + timeout
    spin(0.3, beat)
    while time.monotonic() < t:
        spin(0.1, beat)
        if status['m'] is not None and not status['m'].running:
            return status['m']
    return status['m']


def check(name, ok, detail=''):
    results.append(ok)
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   [{detail}]' if detail else ''), flush=True)


try:
    assert start.wait_for_service(timeout_sec=10), 'service not up'
    spin(1.5)
    r = call(start, req(confirm=False))
    check('rejects without lifted confirmation', not r.accepted, r.message)
    r = call(start, req(value=6.0))
    check('rejects current above limit (5 A)', not r.accepted, r.message)
    r = call(start, req(mode='position', value=800.0, dur=2.0, speed=1.0))
    check('rejects position move above 720 deg', not r.accepted, r.message)
    r = call(start, req(mode='position', value=90.0, dur=2.0, speed=25.0))
    check('rejects position speed above limit', not r.accepted, r.message)
    r = call(start, req(mode='velocity', value=50.0))
    check('rejects velocity above limit', not r.accepted, r.message)

    r = call(start, req(value=0.5, dur=1.0))
    check('accepts 0.5 A / 1 s', r.accepted, r.message)
    m = wait_done()
    check('current test completes normally', m.result == 'done', m.result)
    check('peak current observed > 0.3 A', m.peak_current_a > 0.3, f'{m.peak_current_a:.2f}')

    r = call(start, req(mode='velocity', value=1.0, dur=2.0))
    m = wait_done()
    check('velocity test completes', r.accepted and m.result == 'done', m.result)
    check('velocity scale consistent (mean ~1.0 rad/s)', abs(m.mean_velocity_rad_s - 1.0) < 0.15,
          f'{m.mean_velocity_rad_s:.3f}')

    r = call(start, req(value=0.5, dur=3.0))
    spin(0.5)
    call(stop, Trigger.Request())
    m = wait_done()
    check('STOP service aborts test', 'operator STOP' in m.result, m.result)

    r = call(start, req(value=0.5, dur=3.0))
    spin(0.5)
    m = wait_done(beat=False)
    check('heartbeat loss aborts test', m.result == 'operator heartbeat lost', m.result)

    spin(1.0)
    r = call(start, req(mode='position', value=90.0, dur=3.0, speed=2.0))
    m = wait_done()
    import math
    check('position move 90 deg completes', r.accepted and m.result == 'done', f'{r.message} / {m.result}')
    check('position final error < 1 deg', abs(math.degrees(m.position_error_rad)) < 1.0,
          f'moved {math.degrees(m.moved_rad):.1f} deg, error {math.degrees(m.position_error_rad):+.2f} deg')

    spin(1.0)
    drive.send_signal(signal.SIGUSR2)  # low friction: constant current makes the wheel run away
    r = call(start, req(value=3.0, dur=3.0))
    m = wait_done()
    drive.send_signal(signal.SIGUSR2)
    check('over-speed guard aborts runaway wheel', m.result == 'over-speed guard', m.result)

    spin(1.5)
    r = call(start, req(value=0.5, dur=3.0))
    spin(0.5)
    drive.send_signal(signal.SIGUSR1)  # drive stops uploading
    m = wait_done()
    drive.send_signal(signal.SIGUSR1)
    check('stale feedback aborts test', m.result == 'feedback stale', m.result)
    spin(1.0)
    r = call(start, req(value=0.5, dur=1.0))
    m = wait_done()
    check('recovers after stale (new test ok)', r.accepted and m.result == 'done', m.result)
finally:
    os.killpg(node_p.pid, signal.SIGINT)   # ros2 run does not always forward SIGINT to the node
    try:
        node_p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(node_p.pid, signal.SIGKILL)
    time.sleep(0.3)
    drive.terminate()
    out = drive.communicate(timeout=3)[0].strip().splitlines()
    cur_cmds = [l for l in out if 'fn=1' in l]
    check('last command sent was 0 A', bool(cur_cmds) and cur_cmds[-1].endswith('target=0.000'),
          cur_cmds[-1] if cur_cmds else 'none')
    print(f'---- {sum(results)} passed, {len(results) - sum(results)} failed')
    rclpy.try_shutdown()
    sys.exit(0 if all(results) else 1)
