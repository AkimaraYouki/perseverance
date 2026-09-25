#!/usr/bin/env python3
"""DDS 왕복 지연 시험. 로봇에서 echo, 데스크톱에서 ping. 시계 동기 필요 없음 (보낸 쪽 시계만 쓴다).
    (Jetson)   python3 dds_rtt.py echo
    (데스크톱) python3 dds_rtt.py ping --hz 50 --seconds 20 [--reliable]
"""
import argparse, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=["echo", "ping"])
ap.add_argument("--hz", type=float, default=50.0); ap.add_argument("--seconds", type=float, default=20.0)
ap.add_argument("--reliable", action="store_true"); a = ap.parse_args()
qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.RELIABLE if a.reliable else ReliabilityPolicy.BEST_EFFORT)
rclpy.init(); n = Node("dds_rtt_" + a.mode)
if a.mode == "echo":
    pub = n.create_publisher(Header, "/pv_pong", qos)
    n.create_subscription(Header, "/pv_ping", lambda m: pub.publish(m), qos)
    t_end = time.monotonic() + a.seconds
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
else:
    pub = n.create_publisher(Header, "/pv_ping", qos); rtt = []; sent = 0
    def pong(m):
        rtt.append((time.monotonic_ns() - int(m.frame_id)) / 1e6)
    n.create_subscription(Header, "/pv_pong", pong, qos)
    t0 = time.monotonic(); nxt = t0
    while rclpy.ok() and time.monotonic() - t0 < a.seconds:
        if time.monotonic() >= nxt:
            h = Header(); h.frame_id = str(time.monotonic_ns()); pub.publish(h); sent += 1; nxt += 1.0 / a.hz
        rclpy.spin_once(n, timeout_sec=0.002)
    t1 = time.monotonic() + 1.0
    while time.monotonic() < t1: rclpy.spin_once(n, timeout_sec=0.05)
    if rtt:
        r = np.array(rtt)
        print(f"{'RELIABLE' if a.reliable else 'BEST_EFFORT'}: 보냄 {sent} 받음 {len(r)} (손실 {100*(1-len(r)/max(sent,1)):.1f} %)  "
              f"왕복 중앙 {np.median(r):.1f} ms  평균 {r.mean():.1f}  95% {np.percentile(r,95):.1f}  최대 {r.max():.1f}  최소 {r.min():.1f}")
    else:
        print("받은 응답 없음 (echo 가 떠 있는지, 도메인·피어 확인)")
n.destroy_node(); rclpy.try_shutdown()
