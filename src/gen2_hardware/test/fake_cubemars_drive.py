#!/usr/bin/env python3
"""Fake CubeMars servo-mode drive on a (v)CAN interface, for testing command paths safely.

Uploads 0x29|id status at `rate` Hz (manual 5.2.1 layout), reacts to current (1) and rpm (3)
commands with a crude first-order model. Control file (JSON-less): writes last command to stdout.
Signals: SIGUSR1 toggles feedback upload (stale drive), SIGUSR2 toggles low friction (runaway).
"""
import argparse
import signal
import socket
import struct
import time

p = argparse.ArgumentParser()
p.add_argument('--iface', default='vcan0')
p.add_argument('--id', type=int, default=1)
p.add_argument('--rate', type=float, default=100.0)
p.add_argument('--pole-pairs', type=float, default=14)
p.add_argument('--gear', type=float, default=10)
p.add_argument('--neg-gain', type=float, default=1.0,
               help='torque gain for negative current (direction asymmetry, e.g. 0.8)')
a = p.parse_args()

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((a.iface,))
s.setblocking(False)
EFF = socket.CAN_EFF_FLAG
upload = [True]
signal.signal(signal.SIGUSR1, lambda *_: upload.__setitem__(0, not upload[0]))
damping = [8.0]  # SIGUSR2 toggles a low-friction "runaway" wheel
signal.signal(signal.SIGUSR2, lambda *_: damping.__setitem__(0, 0.5 if damping[0] > 1 else 8.0))

pos_deg, erpm, cur = 0.0, 0.0, 0.0
mode, target = 'idle', 0.0
last_cmd_t = 0.0
t_prev = time.monotonic()
next_up = t_prev
while True:
    try:
        while True:
            fr = s.recv(16)
            cid, dlc, data = struct.unpack('<IB3x8s', fr)
            if not (cid & EFF) or (cid & 0xFF) != a.id:
                continue
            fn = (cid & 0x1FFFFFFF) >> 8
            if fn == 1:
                mode, target = 'current', struct.unpack('>i', data[:4])[0] / 1000.0
            elif fn == 3:
                mode, target = 'rpm', float(struct.unpack('>i', data[:4])[0])
            elif fn == 5:
                pos_deg = 0.0
            elif fn == 6:  # position-speed loop: int32 deg*1e4, int16 ERPM/10, int16 ERPM/s^2 /10
                p_t = struct.unpack('>i', data[:4])[0] / 10000.0
                spd = struct.unpack('>h', data[4:6])[0] * 10.0
                mode, target = 'pos', (p_t, spd)
            last_cmd_t = time.monotonic()
            tgt = f'{target[0]:.2f}deg@{target[1]:.0f}erpm' if isinstance(target, tuple) else f'{target:.3f}'
            print(f'CMD fn={fn} mode={mode} target={tgt}', flush=True)
    except BlockingIOError:
        pass
    now = time.monotonic()
    dt, t_prev = now - t_prev, now
    if now - last_cmd_t > 1.0:            # drive-side timeout_msec 1000 -> release
        mode, target = 'idle', 0.0
    if mode == 'pos':
        p_t, spd = target
        out_spd = spd / a.pole_pairs / a.gear * 6.0            # output deg/s limit
        want = max(-out_spd, min(out_spd, 5.0 * (p_t - pos_deg)))
        erpm = want / 6.0 * a.pole_pairs * a.gear
        cur = 0.2 * (p_t - pos_deg)
    elif mode == 'current':
        cur = target
        gain = a.neg_gain if cur < 0 else 1.0
        erpm += cur * gain * 20000.0 * dt - erpm * damping[0] * dt
    elif mode == 'rpm':
        erpm += (target - erpm) * min(1.0, 10.0 * dt)
        cur = 0.1 * (target - erpm) / 1000.0
    else:
        cur = 0.0
        erpm -= erpm * 5.0 * dt
    out_rpm = erpm / a.pole_pairs / a.gear
    pos_deg += out_rpm * 6.0 * dt
    if now >= next_up:
        next_up += 1.0 / a.rate
        if upload[0]:
            p16 = max(-32000, min(32000, int(round(((pos_deg + 3200) % 6400 - 3200) * 10))))
            data = struct.pack('>hhhbB', p16, int(max(-32000, min(32000, erpm / 10))),
                               int(max(-6000, min(6000, cur * 100))), 40, 0)
            s.send(struct.pack('<IB3x8s', (0x2900 | a.id) | EFF, 8, data))
    time.sleep(0.001)
