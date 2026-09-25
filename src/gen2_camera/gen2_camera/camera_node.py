"""저지연 카메라 퍼블리셔 — CSI(IMX219) 를 GStreamer 로 받아 JPEG 으로 바로 발행한다.

(데스크톱 초안 comms/to-robot/attachments/gen2_camera 를 로봇 쪽에서 받아 보완, 2026-09-25)

지연을 줄이는 원칙 (jarabot opencv_webcam_pub 를 읽고 정리):
  1. 원본 Image 가 아니라 CompressedImage(JPEG) 를 보낸다 (640x480 원본 921 KB/장 → JPEG 수십 KB).
  2. 프레임마다 독립인 JPEG. Orin Nano 에는 NVENC 가 없다. JPEG 은 CPU(jpegenc) 또는 nvjpegenc.
  3. **항상 최신 프레임만**: appsink max-buffers=1 drop=true sync=false.
  4. QoS 는 Best Effort + depth 1. 받는 쪽도 Best Effort 로 구독해야 연결된다.
  5. 발행 루프에 sleep 이 없다. 카메라 fps 가 곧 발행 fps.

로봇 쪽 보완:
  - 헤더 stamp = **촬영 시각 추정** (ROS now − 파이프라인 안에서의 버퍼 나이). 버퍼 나이는 GStreamer 클럭과 버퍼
    PTS 로 잰다 (센서 노출·ISP·인코딩 시간이 여기에 들어간다). 나이가 이상하면(음수, 0.5 s 이상) 받은 시각을 쓴다.
    `stamp_source: publish` 면 항상 받은 시각.
  - 파이프라인 오류/EOS/3 s 무프레임 → 2 s 뒤 자동 재시작 (카메라 케이블·nvargus 문제로 노드가 죽지 않게).
  - 진단 이름 "camera: stream" (LCD 의 CAM 줄이 읽는다), 값: fps, KB/장, 촬영→발행 지연(평균/최대), 재시작 횟수.
"""

import threading
import time

import gi
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
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
        cam = (f'nvarguscamerasrc sensor-id={p["sensor_id"]} ! '
               f"video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1")
        if p["encoder"] == "nvjpeg":  # NVMM 그대로 nvjpegenc (CPU 복사 없음)
            return (f'{cam} ! nvvidconv flip-method={p["flip_method"]} ! '
                    f"video/x-raw(memory:NVMM),format=I420 ! nvjpegenc quality={q} ! {SINK}")
        src = f'{cam} ! nvvidconv flip-method={p["flip_method"]} ! video/x-raw,format=I420'
    elif p["source"] == "v4l2":       # USB 웹캠: 카메라가 이미 MJPEG 을 주면 재인코딩 없이 그대로 쓴다
        return f"v4l2src device=/dev/video0 ! image/jpeg,width={w},height={h},framerate={fps}/1 ! {SINK}"
    else:                             # test: 카메라 없이 검증
        src = f"videotestsrc is-live=true pattern=ball ! video/x-raw,format=I420,width={w},height={h},framerate={fps}/1"
    return f"{src} ! jpegenc quality={q} ! {SINK}"


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        d = dict(source="argus", sensor_id=0, width=640, height=480, fps=30, jpeg_quality=80, flip_method=0,
                 encoder="cpu", stamp_source="capture", frame_id="camera_link",
                 topic="camera/image_raw/compressed", pipeline="", restart_delay_s=2.0, stall_timeout_s=3.0)
        self.p = {k: self.declare_parameter(k, v).value for k, v in d.items()}
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(CompressedImage, self.p["topic"], qos)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        Gst.init(None)
        self.desc = build_pipeline(self.p)
        self.get_logger().info(f"GStreamer: {self.desc}")
        self.pipe = self.sink = self.bus = None
        self.lock = threading.Lock()
        self.n, self.bytes, self.t0 = 0, 0, time.monotonic()
        self.ages = []
        self.restarts, self.last_error = 0, ""
        self.running = True
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()
        self.create_timer(1.0, self.report)

    # ------------------------------------------------------------------ pipeline lifecycle
    def _start(self):
        self.pipe = Gst.parse_launch(self.desc)
        self.sink = self.pipe.get_by_name("sink")
        self.bus = self.pipe.get_bus()
        self.pipe.set_state(Gst.State.PLAYING)

    def _stop(self):
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)
        self.pipe = self.sink = self.bus = None

    def _bus_problem(self):
        while True:
            m = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if m is None:
                return None
            if m.type == Gst.MessageType.EOS:
                return "end of stream"
            err, dbg = m.parse_error()
            return f"{err.message}"

    def loop(self):
        while self.running and rclpy.ok():
            try:
                self._start()
            except Exception as e:  # parse/launch failure (missing plugin, bad parameters)
                self._fail(f"launch: {e}")
                continue
            last_frame = time.monotonic()
            problem = None
            while self.running and rclpy.ok():
                problem = self._bus_problem()
                if problem:
                    break
                sample = self.sink.emit("try-pull-sample", int(0.5 * Gst.SECOND))
                if sample is None:
                    if time.monotonic() - last_frame > self.p["stall_timeout_s"]:
                        problem = f"no frame for {self.p['stall_timeout_s']:.0f} s"
                        break
                    continue
                last_frame = time.monotonic()
                self._publish(sample)
            self._stop()
            if problem and self.running:
                self._fail(problem)

    def _fail(self, why):
        self.restarts += 1
        self.last_error = why
        self.get_logger().error(f"camera pipeline problem: {why} — restarting in {self.p['restart_delay_s']} s")
        self._stop()
        time.sleep(self.p["restart_delay_s"])

    # ------------------------------------------------------------------ frames
    def _publish(self, sample):
        buf = sample.get_buffer()
        now = self.get_clock().now()
        age_ns = None
        clock = self.pipe.get_clock() if self.pipe is not None else None
        if clock is not None and buf.pts != Gst.CLOCK_TIME_NONE:
            age_ns = clock.get_time() - self.pipe.get_base_time() - buf.pts
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return
        try:
            msg = CompressedImage()
            if self.p["stamp_source"] == "capture" and age_ns is not None and 0 <= age_ns < 500_000_000:
                msg.header.stamp = (now - Duration(nanoseconds=age_ns)).to_msg()
            else:
                msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.p["frame_id"]
            msg.format = "jpeg"
            msg.data = bytes(info.data)
        finally:
            buf.unmap(info)
        try:
            self.pub.publish(msg)
        except Exception:  # context shut down (Ctrl+C) while a frame was in flight
            self.running = False
            return
        with self.lock:
            self.n += 1
            self.bytes += len(msg.data)
            if age_ns is not None and 0 <= age_ns < 5_000_000_000:
                self.ages.append(age_ns / 1e6)

    def report(self):
        with self.lock:
            dt = max(time.monotonic() - self.t0, 1e-6)
            fps, kbps = self.n / dt, self.bytes * 8 / dt / 1e3
            size = self.bytes / max(self.n, 1) / 1e3
            ages, self.ages = self.ages, []
            self.n, self.bytes, self.t0 = 0, 0, time.monotonic()
        age_mean = sum(ages) / len(ages) if ages else float("nan")
        age_max = max(ages) if ages else float("nan")
        ok = fps > 0.5 * self.p["fps"]
        st = DiagnosticStatus(name="camera: stream", hardware_id=f'{self.p["source"]}/{self.p["encoder"]}',
                              level=DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR,
                              message=f"{fps:.0f}fps {size:.0f}KB" if ok else
                              (f"no frames: {self.last_error}"[:40] if self.last_error else "no frames"))
        st.values = [KeyValue(key=k, value=v) for k, v in (
            ("fps", f"{fps:.1f}"), ("kbit_s", f"{kbps:.0f}"), ("kb_per_frame", f"{size:.1f}"),
            ("capture_to_publish_ms_mean", f"{age_mean:.1f}"), ("capture_to_publish_ms_max", f"{age_max:.1f}"),
            ("encoder", self.p["encoder"]), ("resolution", f'{self.p["width"]}x{self.p["height"]}'),
            ("restarts", str(self.restarts)), ("last_error", self.last_error))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag.publish(arr)

    def destroy_node(self):
        self.running = False
        self.th.join(timeout=2.0)
        self._stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
