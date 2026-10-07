#!/usr/bin/env python3
"""AK45-10 CAN 작동 테스트 (can0, 1 Mbps). 단계:
  listen   : 2초 수동 수신 (서보 모드 주기 상태 프레임 확인)
  scan     : MIT enter(FF..FC) 를 ID 1~N 에 보내 응답 확인 → 즉시 exit(FF..FD)
  hold ID  : MIT 진입 → 현재 위치에서 약한 kp 로 유지 → 천천히 ±0.3 rad 왕복 → exit
"""
import socket, struct, sys, time

IF = 'can0'
P_MAX, V_MAX, T_MAX = 12.56, 20.0, 8.0   # CMESC_MIT_APP_AK45_10.bin 0x9b78(명령)/0x8c88(응답) 에서 확인
KP_MAX, KD_MAX = 500.0, 5.0

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((IF,)); s.settimeout(0.05)
FMT = '=IB3x8s'

def send(cid, data, ext=False):
    if ext: cid |= socket.CAN_EFF_FLAG
    s.send(struct.pack(FMT, cid, len(data), bytes(data).ljust(8, b'\0')))

def recv_all(dur):
    out, t = [], time.time()
    while time.time() - t < dur:
        try: f = s.recv(16)
        except socket.timeout: continue
        cid, dlc, d = struct.unpack(FMT, f)
        out.append((cid, d[:dlc]))
    return out

def show(cid, d):
    ext = bool(cid & socket.CAN_EFF_FLAG); cid &= socket.CAN_EFF_MASK
    return f'{"EXT" if ext else "STD"} 0x{cid:X} [{len(d)}] {d.hex(" ")}'

def f2u(x, lo, hi, bits):
    x = min(max(x, lo), hi); return int((x - lo) * ((1 << bits) - 1) / (hi - lo))
def u2f(u, lo, hi, bits): return u * (hi - lo) / ((1 << bits) - 1) + lo

def mit_cmd(cid, p, v, kp, kd, t):
    p_, v_, kp_, kd_, t_ = (f2u(p, -P_MAX, P_MAX, 16), f2u(v, -V_MAX, V_MAX, 12),
                            f2u(kp, 0, KP_MAX, 12), f2u(kd, 0, KD_MAX, 12), f2u(t, -T_MAX, T_MAX, 12))
    send(cid, [p_ >> 8, p_ & 0xFF, v_ >> 4, ((v_ & 0xF) << 4) | (kp_ >> 8), kp_ & 0xFF,
               kd_ >> 4, ((kd_ & 0xF) << 4) | (t_ >> 8), t_ & 0xFF])

def mit_parse(d):
    if len(d) < 6: return None
    p = u2f((d[1] << 8) | d[2], -P_MAX, P_MAX, 16)
    v = u2f((d[3] << 4) | (d[4] >> 4), -V_MAX, V_MAX, 12)
    t = u2f(((d[4] & 0xF) << 8) | d[5], -T_MAX, T_MAX, 12)
    extra = f' temp={d[6]-40}C err={d[7]}' if len(d) >= 8 else ''
    return f'id={d[0]} p={p:+.3f}rad v={v:+.2f} t={t:+.2f}{extra}', p

ENTER = [0xFF]*7 + [0xFC]; EXIT = [0xFF]*7 + [0xFD]

def last_pos(frames):
    for cid, d in reversed(frames):
        r = mit_parse(d)
        if r: return r[1]

mode = sys.argv[1] if len(sys.argv) > 1 else 'listen'
if mode == 'listen':
    fr = recv_all(2.0)
    print(f'{len(fr)} frames');  [print(' ', show(*f)) for f in fr[:20]]
elif mode == 'scan':
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    for i in range(1, n + 1):
        send(i, ENTER); fr = recv_all(0.1); send(i, EXIT); fr += recv_all(0.1)
        if fr:
            print(f'ID {i}:'); [print('  ', show(c, d), '|', (mit_parse(d) or ('',))[0]) for c, d in fr]
    print('scan done')
elif mode == 'hold':
    cid = int(sys.argv[2]); amp = 0.3
    send(cid, ENTER); fr = recv_all(0.1)
    p0 = last_pos(fr)
    if p0 is None: sys.exit('응답 없음 — enter 실패')
    print('start', mit_parse(fr[-1][1])[0])
    try:
        import math; t0 = time.time(); nxt = 0.0
        while time.time() - t0 < 6.0:
            tt = time.time() - t0
            mit_cmd(cid, p0 + amp * math.sin(2 * math.pi * 0.25 * tt), 0, 5.0, 0.3, 0)
            fr = recv_all(0.01)
            if fr and tt >= nxt:
                nxt += 0.5; r = mit_parse(fr[-1][1]); tgt = p0 + amp * math.sin(2 * math.pi * 0.25 * tt)
                print(f'{tt:4.1f}s tgt={tgt:+.3f}', r[0], f'err={r[1]-tgt:+.3f}')
        mit_cmd(cid, p0, 0, 5.0, 0.3, 0); recv_all(0.5)
    finally:
        mit_cmd(cid, 0, 0, 0, 0, 0); send(cid, EXIT); print('exit', [show(*f) for f in recv_all(0.1)])
