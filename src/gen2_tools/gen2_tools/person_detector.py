#!/usr/bin/env python3
"""Person detection + 17 COCO keypoints on the robot GPU: YOLO11n-pose TensorRT FP16 (~3 ms) on the camera JPEG stream
(the plain detect engine yolo11n_320_fp16.engine also works: no keypoints then).

Run with the vision venv (cuda-python):  ~/venv_vision/bin/python -m gen2_tools.person_detector
Engine: ~/gen2_ws/models/yolo11n_320_fp16.engine (built locally: trtexec --onnx=yolo11n.onnx --fp16).

Out: people/camera (std_msgs/String, JSON): {"stamp": s, "w": W, "h": H, "people": [
       {"box": [x0, y0, x1, y1] (pixels), "score": s, "bearing_deg": +left (from the ankles / knees / hips when seen,
        else the box centre; "bearing_from"), "kps": 17 x [u, v, conf] (pose engine), "hsv_hist": [...] (appearance)}]}
Rate: process_hz (30) while following (follow/status not IDLE), idle_hz (2) otherwise.
Bearing uses the camera intrinsics (fx, cx) — data-sheet values until calibrated. The appearance is a small
HSV histogram of the box (hue x saturation, 8 x 4) for telling two people apart by clothes.
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from gen2_tools.live import Live
import tensorrt as trt
from cuda.bindings import runtime as cudart
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def ck(r):
    err = r[0] if isinstance(r, tuple) else r
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f'CUDA error {err}')
    return r[1] if isinstance(r, tuple) and len(r) > 1 else None


class TrtYolo:
    def __init__(self, path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(path, 'rb') as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.stream = ck(cudart.cudaStreamCreate())
        self.io = {}
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(n))
            dt = trt.nptype(self.engine.get_tensor_dtype(n))
            host = np.zeros(shape, dtype=dt)
            dev = ck(cudart.cudaMalloc(host.nbytes))
            self.ctx.set_tensor_address(n, dev)
            self.io[n] = (host, dev, self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.inp = next(n for n, v in self.io.items() if v[2])
        self.out = next(n for n, v in self.io.items() if not v[2])
        self.size = self.io[self.inp][0].shape[2]

    def __call__(self, x):
        h, d, _ = self.io[self.inp]
        np.copyto(h, x)
        K = cudart.cudaMemcpyKind
        ck(cudart.cudaMemcpyAsync(d, h.ctypes.data, h.nbytes, K.cudaMemcpyHostToDevice, self.stream))
        self.ctx.execute_async_v3(self.stream)
        oh, od, _ = self.io[self.out]
        ck(cudart.cudaMemcpyAsync(oh.ctypes.data, od, oh.nbytes, K.cudaMemcpyDeviceToHost, self.stream))
        ck(cudart.cudaStreamSynchronize(self.stream))
        return oh


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.yolo = TrtYolo(os.path.expanduser(p('engine', '~/gen2_ws/models/yolo11n-pose_320_fp16.engine')))
        self.hz = p('process_hz', 30.0)          # = camera rate
        self.hz_idle = p('idle_hz', 2.0)         # follow off (follow/status IDLE or no person_follow): save GPU / CPU
        self.active_t = -1e9
        self.create_subscription(String, 'follow/status', self._follow_status, 10)
        self.conf = p('score_min', 0.40)
        self.fx, self.cx = p('fx', 530.0), p('cx', 320.0)
        self.last_t, self.n, self.t_inf = 0.0, 0, 0.0
        self.pub = self.create_publisher(String, 'people/camera', 10)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self._img, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        self.get_logger().info(f'YOLO TRT input {self.yolo.size}px, {self.hz} Hz')
        Live(self, {'score_min': 'conf', 'process_hz': 'hz', 'idle_hz': 'hz_idle'})

    def _report(self):
        if self.n:
            self.get_logger().info(f'{self.n / 10.0:.1f} frames/s, preprocess+infer+post {self.t_inf / self.n * 1e3:.1f} ms')
        self.n, self.t_inf = 0, 0.0

    def _follow_status(self, m):
        if not m.data.startswith('IDLE'):
            self.active_t = time.monotonic()

    def _img(self, msg):
        now = time.monotonic()
        hz = self.hz if now - self.active_t < 1.0 else self.hz_idle
        if now - self.last_t < 0.7 / hz:           # 0.7: frame-arrival jitter must not skip every other frame
            return
        self.last_t = now
        # half-resolution JPEG decode (libjpeg DCT scaling): 320x240 is the network size anyway, ~3x faster
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        full = 2.0
        if img is None:
            return
        t0 = time.monotonic()
        H, W = img.shape[:2]
        S = self.yolo.size
        r = min(S / W, S / H)
        nw, nh = int(round(W * r)), int(round(H * r))
        canvas = np.full((S, S, 3), 114, np.uint8)
        dx, dy = (S - nw) // 2, (S - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh))
        x = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.yolo(x)[0]
        pose = out.shape[0] == 56                   # yolo11n-pose: cx, cy, w, h, score, 17 x (x, y, conf)
        score = out[4]                              # pose: person score / detect: class 0 = person
        k = score > self.conf
        boxes, scores, kps = [], [], []
        for j in np.nonzero(k)[0]:
            cxb, cyb, wb, hb = out[0, j], out[1, j], out[2, j], out[3, j]
            boxes.append([float((cxb - wb / 2 - dx) / r), float((cyb - hb / 2 - dy) / r), float(wb / r), float(hb / r)])
            scores.append(float(score[j]))
            if pose:
                kp = out[5:, j].reshape(17, 3).copy()
                kp[:, 0] = (kp[:, 0] - dx) / r
                kp[:, 1] = (kp[:, 1] - dy) / r
                kps.append(kp)
        people = []
        if boxes:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            for i in np.array(cv2.dnn.NMSBoxes(boxes, scores, self.conf, 0.5)).reshape(-1):
                bx, by, bw, bh = boxes[i]
                x0, y0 = max(0, int(bx)), max(0, int(by))
                x1, y1 = min(W, int(bx + bw)), min(H, int(by + bh))
                if x1 - x0 < 4 or y1 - y0 < 4:
                    continue
                hist = cv2.calcHist([hsv[y0:y1, x0:x1]], [0, 1], None, [8, 4], [0, 180, 0, 256])
                hist = (hist / max(1.0, hist.sum())).flatten()
                u_bear = (x0 + x1) / 2                      # bearing pixel: box centre unless the feet / knees are seen
                d = {'box': [int(x0 * full), int(y0 * full), int(x1 * full), int(y1 * full)], 'score': round(scores[i], 3),
                     'hsv_hist': [round(float(v), 4) for v in hist]}
                if pose:
                    kp = kps[i]
                    for a_, b_ in ((15, 16), (13, 14), (11, 12)):      # ankles, knees, hips (COCO)
                        sel = [q for q in (a_, b_) if kp[q, 2] > 0.5]
                        if sel:
                            u_bear = float(np.mean(kp[sel, 0]))
                            d['bearing_from'] = {15: 'ankles', 13: 'knees', 11: 'hips'}[a_]
                            break
                    d['kps'] = [[round(float(q[0] * full), 1), round(float(q[1] * full), 1), round(float(q[2]), 2)] for q in kp]
                d['bearing_deg'] = round(math.degrees(math.atan2(self.cx - full * u_bear, self.fx)), 1)
                people.append(d)
        self.t_inf += time.monotonic() - t0
        self.n += 1
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub.publish(String(data=json.dumps({'stamp': st, 'w': int(W * full), 'h': int(H * full), 'people': people})))


def main():
    rclpy.init()
    try:
        rclpy.spin(PersonDetector())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
