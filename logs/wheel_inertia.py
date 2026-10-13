#!/usr/bin/env python3
"""Wheel-axis inertia (desktop comms 2026-10-13_1700): AK45-10 in MIT legacy, torque-only commands at ~1 kHz.

Robot on a stand, wheel touching nothing, gen2-balance STOPPED (one CAN commander). Per wheel and current I,
N_TRIPS times: +I from rest until v >= +V_REV, then coast (t = 0) to rest. One direction only: after a torque
sign change the drive often ignored the next push (2026-10-13).
No torque reversal while spinning: the AK45-10 MIT firmware freezes its replies and ignores commands (~1 min) after
reversals at speed (balance_node note 2026-10-08; seen again here 2026-10-13 with the desktop's reversal method).
Accel + coast segments at both speed signs -> J, Coulomb f, viscous b by least squares. Raw CSV per wheel.
The drive also ignores torque after ~0.4 s of zero-torque commands: MIT enter is re-sent before every push.
Abort: |v| > V_ABORT, a segment longer than SEG_TIMEOUT, no reply for 50 ms, replies bit-identical for 50 ms while
the command changed (frozen) -> exit frame (0 torque).

Usage: python3 wheel_inertia.py <can_id> <label> [currents...]
"""
import csv
import os
import socket
import struct
import sys
import time

IFACE = 'can0'
P_MAX, V_MAX, T_MAX, KP_MAX, KD_MAX = 12.56, 20.0, 8.0, 500.0, 5.0     # motors.yaml wheel mit ranges
KT = 1.27                     # N.m/A output side (datasheet; the drive uses its own Kt for t -> current)
V_REV, V_ABORT = 12.0, 20.5         # 19-20 rad/s = no-load max at 25 V; feedback range +-20
N_TRIPS, N_COAST = 5, 3
SEG_TIMEOUT, DT = 1.5, 0.002


def to_uint(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int(round((x - lo) / (hi - lo) * ((1 << bits) - 1)))


def to_double(u, lo, hi, bits):
    return u * (hi - lo) / ((1 << bits) - 1) + lo


def cmd_frame(cid, t):
    p, v = to_uint(0.0, -P_MAX, P_MAX, 16), to_uint(0.0, -V_MAX, V_MAX, 12)
    kp, kd, tt = 0, 0, to_uint(t, -T_MAX, T_MAX, 12)
    d = bytes([p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (kp >> 8), kp & 0xFF, kd >> 4,
               ((kd & 0xF) << 4) | (tt >> 8), tt & 0xFF])
    return struct.pack('=IB3x8s', cid, 8, d)


def special(cid, last):
    return struct.pack('=IB3x8s', cid, 8, bytes([0xFF] * 7 + [last]))


class Bus:
    def __init__(self, cid):
        self.cid = cid
        self.s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((IFACE,))
        self.s.settimeout(0.0)

    def drain(self):
        """Newest reply from our drive: (v rad/s, t N.m) or None."""
        last = None
        while True:
            try:
                fr = self.s.recv(16)
            except (BlockingIOError, socket.timeout):
                return last
            cid, dlc, d = struct.unpack('=IB3x8s', fr)
            if cid & 0x80000000 or dlc < 6 or d[0] != self.cid:
                continue
            if dlc == 8 and d[0] == 0xFF and d[1] == 0xFF and d[6] == 0xFF:
                continue
            v = to_double((d[3] << 4) | (d[4] >> 4), -V_MAX, V_MAX, 12)
            t = to_double(((d[4] & 0xF) << 8) | d[5], -T_MAX, T_MAX, 12)
            last = (v, t)

    def send(self, fr):
        self.s.send(fr)


def main():
    cid, label = int(sys.argv[1]), sys.argv[2]
    currents = [float(x) for x in sys.argv[3:]] or [1.0, 1.5, 2.0, 2.5]
    out_dir = os.path.expanduser('~/gen2_ws/logs/wheel_inertia')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, time.strftime('%Y-%m-%d_%H%M%S') + f'_{label}_id{cid}.csv')
    bus = Bus(cid)
    rows = []
    t0 = time.perf_counter()
    last_rx = [t0]
    v_now = [0.0]
    frz = [None, 0.0, 0.0]          # last distinct reply, command at that time, time

    def tick(i_cmd, seg):
        bus.send(cmd_frame(cid, i_cmd * KT))
        target = time.perf_counter() + DT
        while time.perf_counter() < target:
            pass
        r = bus.drain()
        now = time.perf_counter()
        if r is not None:
            if r == frz[0] and i_cmd != frz[1]:
                if now - frz[2] > 0.05:
                    raise RuntimeError(f'replies frozen (v {r[0]:.2f}, t {r[1]:.3f}) while the command changed')
            elif r != frz[0]:
                frz[0], frz[1], frz[2] = r, i_cmd, now
            v_now[0] = r[0]
            last_rx[0] = now
            rows.append((now - t0, seg, i_cmd, r[1] / KT, r[0]))
        if now - last_rx[0] > 0.05:
            raise RuntimeError('no reply for 50 ms')
        if abs(v_now[0]) > V_ABORT:
            raise RuntimeError(f'speed {v_now[0]:.1f} rad/s > {V_ABORT}')

    try:
        bus.send(special(cid, 0xFC))           # enter MIT
        time.sleep(0.05)
        bus.drain()
        for _ in range(20):
            tick(0.0, 'idle')
        seg_n = 0

        def wake():
            bus.send(special(cid, 0xFD))
            time.sleep(0.05)
            bus.send(special(cid, 0xFC))
            time.sleep(0.02)
            for _ in range(10):
                tick(0.0, 'enter')

        def run(i_cmd, until, tag, tries=3):
            nonlocal seg_n
            for k in range(tries):
                seg_n += 1
                if i_cmd != 0.0:
                    wake()
                ts = time.perf_counter()
                while not until():
                    tick(i_cmd, f'{tag}_s{seg_n}')
                    lim = 6.0 if i_cmd == 0 else (0.3 if abs(v_now[0]) < 1.0 else SEG_TIMEOUT)
                    if time.perf_counter() - ts > lim:
                        break
                else:
                    return
                if i_cmd == 0.0:
                    raise RuntimeError(f'segment {seg_n} ({tag}) timeout at v {v_now[0]:.1f}')
                print(f'push ignored ({tag}, try {k + 1}), exit/enter and wait 1 s', flush=True)
                for _ in range(10):
                    tick(0.0, 'retry')
                time.sleep(1.0)
                bus.drain()
            raise RuntimeError(f'{tag}: drive ignores torque after {tries} tries')
        for i_a in currents:
            for _ in range(N_TRIPS):
                run(i_a, lambda: v_now[0] >= V_REV, f'I{i_a:+.1f}')
                run(0.0, lambda: abs(v_now[0]) < 0.3, f'coast{i_a:+.1f}')
                for _ in range(20):
                    tick(0.0, 'idle')
    finally:
        for _ in range(3):
            bus.send(cmd_frame(cid, 0.0))
            time.sleep(0.002)
        bus.send(special(cid, 0xFD))            # exit MIT (0 torque)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'seg', 'I_cmd', 'I_fb', 'w'])
            w.writerows(rows)
        print(f'{len(rows)} rows -> {path}')


if __name__ == '__main__':
    main()
