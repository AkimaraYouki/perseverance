"""저지연 카메라 퍼블리셔 — CSI(IMX219) 를 GStreamer 로 받아 JPEG 으로 바로 발행한다.

지연을 줄이는 원칙 (jarabot opencv_webcam_pub 를 읽고 정리, 2026-09-25):
  1. 원본 Image 가 아니라 CompressedImage(JPEG) 를 보낸다. 640x480 원본은 921 KB/장이라 Wi-Fi DDS 에서
     수백 개 조각으로 쪼개지고, 조각 하나만 잃어도 재전송/폐기 -> 큐가 밀려 수 초 지연이 된다. JPEG 은 30~60 KB.
  2. 프레임마다 독립인 JPEG. H.264 류는 인코더 선행 버퍼 + 수신 지터버퍼가 지연을 쌓는다.
     (Jetson Orin Nano 에는 하드웨어 영상 인코더 NVENC 가 없다 — JPEG 은 CPU libjpeg-turbo 로 한다.)
  3. **항상 최신 프레임만**: appsink max-buffers=1 drop=true sync=false. 처리가 늦어도 옛 프레임이 쌓이지 않는다.
  4. QoS 는 Best Effort + depth 1 (SensorDataQoS 계열). 늦은 프레임은 버린다.
     ※ 받는 쪽도 Best Effort 로 구독해야 한다 — Reliable 구독자는 Best Effort 발행자와 연결되지 않는다.
  5. 발행 루프에 sleep 을 넣지 않는다 (jarabot 은 매 장 33 ms sleep 이 있었다). 카메라 fps 가 곧 발행 fps.

헤더 stamp 는 프레임을 받은 순간의 ROS 시각이다. 받는 쪽에서 now - stamp 로 지연을 재려면 두 기계 시계를
chrony 로 맞춰야 한다 (tools/camera/camera_view.py 가 시계 차를 따로 보여 준다).
"""

import threading
import time

import gi
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

SINK = "appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"


def build_pipeline(p) -> str:
    if p["pipeline"]:
        return f'{p["pipeline"]} ! {SINK}'
    w, h, fps, q = p["width"], p["height"], p["fps"], p["jpeg_quality"]
    if p["source"] == "argus":        # CSI: ISP(자동노출·화이트밸런스·디모자이크) 를 거친다
        src = (f'nvarguscamerasrc sensor-id={p["sensor_id"]} ! '
               f"video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1 ! "
               f'nvvidconv flip-method={p["flip_method"]} ! video/x-raw,format=I420')
    elif p["source"] == "v4l2":       # USB 웹캠: 카메라가 이미 MJPEG 을 주면 재인코딩 없이 그대로 쓴다
        return (f"v4l2src device=/dev/video0 ! image/jpeg,width={w},height={h},framerate={fps}/1 ! {SINK}")
    else:                             # test: 데스크톱 검증용
        src = f"videotestsrc is-live=true pattern=ball ! video/x-raw,format=I420,width={w},height={h},framerate={fps}/1"
    return f"{src} ! jpegenc quality={q} ! {SINK}"


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        d = dict(source="argus", sensor_id=0, width=640, height=480, fps=30, jpeg_quality=80, flip_method=0,
                 frame_id="camera_link", topic="camera/image_raw/compressed", pipeline="")
        self.p = {k: self.declare_parameter(k, v).value for k, v in d.items()}
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(CompressedImage, self.p["topic"], qos)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        Gst.init(None)
        desc = build_pipeline(self.p)
        self.get_logger().info(f"GStreamer: {desc}")
        self.pipe = Gst.parse_launch(desc)
        self.sink = self.pipe.get_by_name("sink")
        self.pipe.set_state(Gst.State.PLAYING)

        self.n, self.bytes, self.t0 = 0, 0, time.monotonic()
        self.running = True
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()
        self.create_timer(1.0, self.report)

    def loop(self):
        while self.running and rclpy.ok():
            sample = self.sink.emit("try-pull-sample", int(0.5 * Gst.SECOND))
            if sample is None:
                continue
            buf = sample.get_buffer()
            ok, info = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.p["frame_id"]
                msg.format = "jpeg"
                msg.data = bytes(info.data)
            finally:
                buf.unmap(info)
            self.pub.publish(msg)
            self.n += 1
            self.bytes += len(msg.data)

    def report(self):
        dt = max(time.monotonic() - self.t0, 1e-6)
        fps, kbps = self.n / dt, self.bytes * 8 / dt / 1e3
        size = self.bytes / max(self.n, 1) / 1e3
        self.n, self.bytes, self.t0 = 0, 0, time.monotonic()
        st = DiagnosticStatus(name="gen2_camera", hardware_id=self.p["source"],
                              level=DiagnosticStatus.OK if fps > 0.5 * self.p["fps"] else DiagnosticStatus.WARN,
                              message=f"{fps:.1f} fps, {size:.0f} KB/frame")
        st.values = [KeyValue(key="fps", value=f"{fps:.1f}"), KeyValue(key="kbit_s", value=f"{kbps:.0f}"),
                     KeyValue(key="kb_per_frame", value=f"{size:.1f}")]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag.publish(arr)

    def destroy_node(self):
        self.running = False
        self.pipe.set_state(Gst.State.NULL)
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
