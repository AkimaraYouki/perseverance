#!/usr/bin/env python3
"""카메라 뷰어 + 지연 측정 (데스크톱 쪽). gen2_camera 가 내는 CompressedImage 를 **Best Effort** 로 받는다.

rqt_image_view 는 Reliable 로 구독해서 Best Effort 발행자와 연결되지 않을 수 있다 — 그래서 따로 둔다.

지연 = 받은 시각 - 헤더 stamp (발행 쪽에서 프레임을 받은 순간). 두 기계 시계가 다르면 그만큼 틀린다:
Jetson 과 데스크톱을 chrony 로 맞출 것 (`chronyc tracking` 의 System time offset 이 1 ms 이하인지).
같은 기계에서 돌리면 시계 차가 없다.

    python3 camera_view.py                        # 창 + 지연 표시
    python3 camera_view.py --no-show --seconds 10 # 창 없이 지연 통계만
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

ap = argparse.ArgumentParser()
ap.add_argument("--topic", default="/camera/image_raw/compressed")
ap.add_argument("--no-show", action="store_true")
ap.add_argument("--seconds", type=float, default=0.0, help="0 이면 창을 닫을 때까지")
args = ap.parse_args()


class Viewer(Node):
    def __init__(self):
        super().__init__("camera_view")
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, args.topic, self.cb, qos)
        self.lat, self.dec, self.sizes, self.t_arr = [], [], [], []

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds
        lat = (now - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)) / 1e6
        t0 = time.perf_counter()
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        dec = (time.perf_counter() - t0) * 1e3
        self.lat.append(lat); self.dec.append(dec); self.sizes.append(len(msg.data)); self.t_arr.append(time.monotonic())
        if not args.no_show and img is not None:
            recent = [t for t in self.t_arr if t > self.t_arr[-1] - 1.0]
            cv2.putText(img, f"latency {lat:6.1f} ms  decode {dec:4.1f} ms  {len(recent)} fps  {len(msg.data)/1e3:.0f} KB",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow("gen2 camera", img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    v = Viewer()
    t_end = time.monotonic() + args.seconds if args.seconds > 0 else float("inf")
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(v, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    if v.lat:
        L = np.array(v.lat)
        dur = max(v.t_arr[-1] - v.t_arr[0], 1e-6)
        print(f"프레임 {len(L)}  {len(L)/dur:.1f} fps  크기 평균 {np.mean(v.sizes)/1e3:.1f} KB  "
              f"지연 평균 {L.mean():.1f} ms  중앙 {np.median(L):.1f}  95% {np.percentile(L, 95):.1f}  최대 {L.max():.1f} ms  "
              f"(+ 디코드 {np.mean(v.dec):.1f} ms)")
    else:
        print("받은 프레임 없음 — 토픽 이름, QoS(Best Effort), ROS_DOMAIN_ID, DDS 설정을 확인할 것")
    v.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
