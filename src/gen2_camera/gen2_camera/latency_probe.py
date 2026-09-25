"""camera_latency — subscribe to the camera stream (Best Effort) and print latency statistics.

latency = receive time − header.stamp. With the default camera setting (stamp_source: capture) the
stamp is the estimated capture time, so this includes exposure/ISP/encode + DDS (+ Wi-Fi when run on
another machine). Across machines the clocks must be synchronised (chrony); on the Jetson itself
there is no clock error.

    ros2 run gen2_camera camera_latency --seconds 10 [--decode]
"""
import argparse
import io
import statistics
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--decode", action="store_true", help="also time JPEG decode (PIL)")
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("camera_latency")
    lat, sizes, dec, arr, dims = [], [], [], [], None
    Image = None
    if a.decode:
        from PIL import Image  # noqa: N806

    def cb(msg):
        nonlocal dims
        now = n.get_clock().now().nanoseconds
        st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        lat.append((now - st) / 1e6)
        sizes.append(len(msg.data))
        arr.append(time.monotonic())
        if Image is not None:
            t0 = time.perf_counter()
            im = Image.open(io.BytesIO(bytes(msg.data)))
            im.load()
            dec.append((time.perf_counter() - t0) * 1e3)
            dims = im.size

    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    n.create_subscription(CompressedImage, a.topic, cb, qos)
    t_end = time.monotonic() + a.seconds
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if len(lat) < 2:
        print("no frames received")
    else:
        q = sorted(lat)
        span = arr[-1] - arr[0]
        print(f"frames {len(lat)}  fps {(len(lat) - 1) / span:.1f}  size {statistics.mean(sizes) / 1e3:.1f} KB")
        print(f"latency ms: mean {statistics.mean(lat):.1f}  median {q[len(q) // 2]:.1f}  "
              f"p95 {q[int(0.95 * (len(q) - 1))]:.1f}  max {q[-1]:.1f}  min {q[0]:.1f}")
        if dec:
            print(f"decode ms (PIL): mean {statistics.mean(dec):.1f}  image {dims}")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
