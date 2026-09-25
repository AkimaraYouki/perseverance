#!/usr/bin/env python3
"""NTP 서버(Jetson chrony)에 질의만 해서 이 PC 시계와의 차를 잰다. 시계는 안 바꾼다 (root 불필요).
offset = 서버 - 이 PC [ms] (양수면 이 PC 가 느리다). 왕복 지연이 작은 표본 여러 개의 중앙값.
    python3 ntp_offset.py 192.168.147.250 [-n 20]
"""
import argparse, socket, struct, time, statistics
ap = argparse.ArgumentParser(); ap.add_argument("server"); ap.add_argument("-n", type=int, default=20); a = ap.parse_args()
NTP_EPOCH = 2208988800
def ts(b): s, f = struct.unpack("!II", b); return s - NTP_EPOCH + f / 2**32
res = []
for _ in range(a.n):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1.0)
    pkt = bytearray(48); pkt[0] = 0x23                      # LI 0, VN 4, mode 3 (client)
    t1 = time.time(); s.sendto(pkt, (a.server, 123))
    try:
        d, _ = s.recvfrom(48); t4 = time.time()
    except socket.timeout:
        continue
    t2, t3 = ts(d[32:40]), ts(d[40:48])
    res.append((((t2 - t1) + (t3 - t4)) / 2 * 1e3, ((t4 - t1) - (t3 - t2)) * 1e3))
    time.sleep(0.05)
if not res: raise SystemExit("응답 없음 — NTP 서버(udp 123) 확인")
res.sort(key=lambda r: r[1]); best = res[: max(3, len(res) // 3)]
off = statistics.median(r[0] for r in best); rtt = statistics.median(r[1] for r in best)
print(f"시계 차 (Jetson - 이 PC) {off:+.2f} ms   왕복 {rtt:.1f} ms   표본 {len(res)}")
