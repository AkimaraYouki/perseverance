#!/usr/bin/env python3
"""Fake CubeMars servo-mode drive on a (v)CAN interface, for testing command paths safely.

Uploads 0x29|id status at `rate` Hz (manual 5.2.1 layout), reacts to current (1) and rpm (3)
commands with a crude first-order model. Control file (JSON-less): writes last command to stdout.
Signals: SIGUSR1 toggles feedback upload (stale drive), SIGUSR2 toggles low friction (runaway).
"""
import argparse
import math
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
p.add_argument('--proto', default='servo', choices=['servo', 'mit_legacy', 'mit_v3'])
p.add_argument('--kt', type=float, default=1.27, help='MIT torque -> current (N·m/A)')
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
R = dict(mit_legacy=(12.56, 20.0, 8.0), mit_v3=(12.56, 60.0, 12.0)).get(a.proto, (12.56, 20.0, 8.0))
mit_on = [a.proto == 'mit_v3']   # legacy needs 'enter motor mode' (FF..FC)


def u2f(x, lo, hi, bits):
    return x * (hi - lo) / ((1 << bits) - 1) + lo


def f2u(x, lo, hi, bits):
    x = max(lo, min(hi, x))
    return int(round((x - lo) / (hi - lo) * ((1 << bits) - 1)))


def mit_reply():
    if not upload[0]:   # SIGUSR1: drive silent (stale test)
        return
    p = f2u(math.radians(pos_deg), -R[0], R[0], 16)
    v = f2u(erpm / a.pole_pairs / a.gear * 2 * math.pi / 60, -R[1], R[1], 12)
    t = f2u(cur * a.kt, -R[2], R[2], 12)
    d = bytes([a.id, p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF, 40 + 40, 0])
    s.send(struct.pack('<IB3x8s', 0x000, 8, d))   # std id 0x000 (master), data[0] = drive id
t_prev = time.monotonic()
next_up = t_prev
while True:
    try:
        while True:
            fr = s.recv(16)
            cid, dlc, data = struct.unpack('<IB3x8s', fr)
            if not (cid & EFF):
                if a.proto != 'mit_legacy' or (cid & 0x7FF) != a.id:
                    continue
                if data[:7] == b'\xff' * 7:
                    if data[7] == 0xFC:
                        mit_on[0] = True
                    elif data[7] == 0xFD:
                        mit_on[0], mode = False, 'idle'
                    elif data[7] == 0xFE:
                        pos_deg = 0.0
                    print(f'CMD special {data[7]:02X}', flush=True)
                else:
                    pi_ = (data[0] << 8) | data[1]; vi = (data[2] << 4) | (data[3] >> 4)
                    kpi = ((data[3] & 0xF) << 8) | data[4]; kdi = (data[5] << 4) | (data[6] >> 4)
                    ti = ((data[6] & 0xF) << 8) | data[7]
                    if mit_on[0]:
                        mode, target = 'mit', (u2f(pi_, -R[0], R[0], 16), u2f(vi, -R[1], R[1], 12),
                                               u2f(kpi, 0, 500, 12), u2f(kdi, 0, 5, 12), u2f(ti, -R[2], R[2], 12))
                        print(f'CMD mit p={target[0]:.3f} v={target[1]:.2f} kp={target[2]:.1f} kd={target[3]:.2f} t={target[4]:.3f}', flush=True)
                last_cmd_t = time.monotonic()
                mit_reply()
                continue
            if (cid & 0xFF) != a.id:
                continue
            fn = (cid & 0x1FFFFFFF) >> 8
            if fn == 1:
                mode, target = 'current', struct.unpack('>i', data[:4])[0] / 1000.0
            elif fn == 3:
                mode, target = 'rpm', float(struct.unpack('>i', data[:4])[0])
            elif fn == 5:
                pos_deg = 0.0
            elif fn == 8 and a.proto == 'mit_v3':
                kpi = (data[0] << 4) | (data[1] >> 4); kdi = ((data[1] & 0xF) << 8) | data[2]
                pi_ = (data[3] << 8) | data[4]; vi = (data[5] << 4) | (data[6] >> 4)
                ti = ((data[6] & 0xF) << 8) | data[7]
                mode, target = 'mit', (u2f(pi_, -R[0], R[0], 16), u2f(vi, -R[1], R[1], 12),
                                       u2f(kpi, 0, 500, 12), u2f(kdi, 0, 5, 12), u2f(ti, -R[2], R[2], 12))
            elif fn == 6:  # position-speed loop: int32 deg*1e4, int16 ERPM/10, int16 ERPM/s^2 /10
                p_t = struct.unpack('>i', data[:4])[0] / 10000.0
                spd = struct.unpack('>h', data[4:6])[0] * 10.0
                mode, target = 'pos', (p_t, spd)
            last_cmd_t = time.monotonic()
            tgt = f'{target[0]:.2f}..' if isinstance(target, tuple) else f'{target:.3f}'
            print(f'CMD fn={fn} mode={mode} target={tgt}', flush=True)
    except BlockingIOError:
        pass
    now = time.monotonic()
    dt, t_prev = now - t_prev, now
    if now - last_cmd_t > 1.0:            # drive-side timeout_msec 1000 -> release
        mode, target = 'idle', 0.0
    if mode == 'mit':
        p_t, v_t, kp, kd, t_ff = target
        w = erpm / a.pole_pairs / a.gear * 2 * math.pi / 60
        tau = kp * (p_t - math.radians(pos_deg)) + kd * (v_t - w) + t_ff
        cur = max(-R[2], min(R[2], tau)) / a.kt
        gain = a.neg_gain if cur < 0 else 1.0
        erpm += cur * gain * 20000.0 * dt - erpm * damping[0] * dt
    elif mode == 'pos':
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
        if upload[0] and a.proto != 'mit_legacy':
            p16 = max(-32000, min(32000, int(round(((pos_deg + 3200) % 6400 - 3200) * 10))))
            data = struct.pack('>hhhbB', p16, int(max(-32000, min(32000, erpm / 10))),
                               int(max(-6000, min(6000, cur * 100))), 40, 0)
            s.send(struct.pack('<IB3x8s', (0x2900 | a.id) | EFF, 8, data))
    time.sleep(0.001)
