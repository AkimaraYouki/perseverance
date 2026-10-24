#!/usr/bin/env python3
"""Camera-first person tracking on the robot GPU (2026-10-25, desktop): YOLO11n-pose TensorRT -> ByteTrack ids ->
designated target with OSNet-x0.25 re-identification (TensorRT) -> people/target. Replaces person_detector for
person_follow_cam (people/camera is still published in the same format, + 'id' / 'target', so the HUD keeps working).

Why: the old stack registered / kept the person only when a LiDAR leg track sat at the camera bearing, once per LiDAR
scan (10 Hz) — a person with a green box but no matching leg track was never locked (user 2026-10-25).

Run (vision venv):  ~/venv_vision/bin/python -m gen2_tools.person_tracker
Models: ~/gen2_ws/models/yolo11n-pose_320_fp16.engine, osnet_x0_25_msmt17.onnx (the .engine is built from it on the
first start, ~1-3 min, then cached). Without the ReID model it still tracks (ids only, no re-identification).

In:  camera/image_raw/compressed, follow/status (rate: process_hz while following, idle_hz otherwise),
     people/target_cmd (String: 'register' = lock the person nearest straight ahead, 'clear').
Out: people/camera (JSON, as person_detector + 'id', 'target'), people/target (JSON: state NONE|LOCK|LOST|REACQUIRED,
     id, box, bearing_deg (+ left), span_deg [left edge, right edge], feet_row (px, full image), score, sim, stamp).
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from gen2_tools import tracking_core as tc
from gen2_tools.live import Live
from gen2_tools.person_detector import TrtYolo, ck
import tensorrt as trt
from cuda.bindings import runtime as cudart
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def build_engine(onnx, engine, logger, max_batch=8):
    """ONNX -> FP16 TensorRT engine with a dynamic batch 1..max_batch (input 'images' N x 3 x 256 x 128)."""
    b = trt.Builder(logger)
    net = b.create_network(0)
    parser = trt.OnnxParser(net, logger)
    with open(onnx, 'rb') as f:
        if not parser.parse(f.read()):
            raise RuntimeError('ONNX parse: ' + '; '.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    cfg = b.create_builder_config()
    cfg.set_flag(trt.BuilderFlag.FP16)
    prof = b.create_optimization_profile()
    name = net.get_input(0).name
    prof.set_shape(name, (1, 3, 256, 128), (4, 3, 256, 128), (max_batch, 3, 256, 128))
    cfg.add_optimization_profile(prof)
    ser = b.build_serialized_network(net, cfg)
    if ser is None:
        raise RuntimeError('TensorRT build failed')
    with open(engine, 'wb') as f:
        f.write(bytes(ser))


class TrtReid:
    """OSNet ReID: crops (BGR) -> unit 512-d embeddings, batch <= max_batch."""

    def __init__(self, onnx, engine, logger, max_batch=8):
        if not os.path.exists(engine):
            build_engine(onnx, engine, logger, max_batch)
        with open(engine, 'rb') as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.stream = ck(cudart.cudaStreamCreate())
        self.inp = self.engine.get_tensor_name(0)
        self.out = self.engine.get_tensor_name(1)
        self.mb = max_batch
        self.h_in = np.zeros((max_batch, 3, 256, 128), np.float32)
        self.h_out = np.zeros((max_batch, 512), np.float32)
        self.d_in = ck(cudart.cudaMalloc(self.h_in.nbytes))
        self.d_out = ck(cudart.cudaMalloc(self.h_out.nbytes))
        self.ctx.set_tensor_address(self.inp, self.d_in)
        self.ctx.set_tensor_address(self.out, self.d_out)

    def __call__(self, crops):
        out = []
        for s in range(0, len(crops), self.mb):
            part = crops[s:s + self.mb]
            n = len(part)
            for i, c in enumerate(part):
                x = cv2.resize(c, (128, 256))[:, :, ::-1].astype(np.float32) / 255.0
                self.h_in[i] = ((x - MEAN) / STD).transpose(2, 0, 1)
            self.ctx.set_input_shape(self.inp, (n, 3, 256, 128))
            K = cudart.cudaMemcpyKind
            ck(cudart.cudaMemcpyAsync(self.d_in, self.h_in.ctypes.data, n * self.h_in[0].nbytes, K.cudaMemcpyHostToDevice, self.stream))
            self.ctx.execute_async_v3(self.stream)
            ck(cudart.cudaMemcpyAsync(self.h_out.ctypes.data, self.d_out, n * self.h_out[0].nbytes, K.cudaMemcpyDeviceToHost, self.stream))
            ck(cudart.cudaStreamSynchronize(self.stream))
            out += [tc.l2n(v) for v in self.h_out[:n]]
        return out


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        M = os.path.expanduser('~/gen2_ws/models/')
        self.yolo = TrtYolo(os.path.expanduser(p('engine', M + 'yolo11n-pose_320_fp16.engine')))
        self.reid = None
        onnx, eng = os.path.expanduser(p('reid_onnx', M + 'osnet_x0_25_msmt17.onnx')), os.path.expanduser(p('reid_engine', M + 'osnet_x0_25_msmt17_fp16.engine'))
        if os.path.exists(onnx) or os.path.exists(eng):
            try:
                t0 = time.monotonic()
                self.reid = TrtReid(onnx, eng, trt.Logger(trt.Logger.WARNING))
                self.get_logger().info(f'ReID OSNet-x0.25 TensorRT ready ({time.monotonic() - t0:.0f} s)')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'ReID off: {e}')
        self.hz, self.hz_idle = p('process_hz', 30.0), p('idle_hz', 2.0)
        self.conf = p('score_min', 0.40)              # published people
        self.det_low = p('det_low', 0.10)             # ByteTrack second-stage detections (blurred / half hidden)
        self.bt = tc.ByteTracker(high=p('track_high', 0.5), low=self.det_low, new=p('track_new', 0.6),
                                 max_lost=int(p('track_keep_frames', 30)))
        self.tm = tc.TargetManager(sim_min=p('reid_sim_min', 0.65), sim_single=p('reid_sim_single', 0.75),
                                   sim_margin=p('reid_margin', 0.08), gate_deg=p('reacq_gate_deg', 12.0),
                                   gate_rate_deg_s=p('reacq_gate_rate_deg_s', 40.0))
        self.cam_model = dict(model=p('camera_model', 'pinhole'), fx=p('fx', 530.0), cx=p('cx', 320.0),
                              hfov_deg=p('hfov_deg', 160.0), K=p('K', [0.0]), D=p('D', [0.0]))
        self.bm = None
        self.active_t, self.last_t = -1e9, 0.0
        self.tm_n, self.tms = 0, np.zeros(4)
        self.pub = self.create_publisher(String, 'people/camera', 10)
        self.pub_t = self.create_publisher(String, 'people/target', 10)
        self.create_subscription(String, 'follow/status', self._follow_status, 10)
        self.create_subscription(String, 'people/target_cmd', self._cmd, 10)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self._img, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        Live(self, {'score_min': 'conf', 'process_hz': 'hz', 'idle_hz': 'hz_idle'})
        self.get_logger().info(f'YOLO {self.yolo.size}px + ByteTrack + {"OSNet ReID" if self.reid else "no ReID"}, '
                               f'camera model {self.cam_model["model"]}')

    def _follow_status(self, m):
        if not m.data.startswith('IDLE'):
            self.active_t = time.monotonic()

    def _cmd(self, m):
        if m.data == 'register':
            self.tm.register()
            self.active_t = time.monotonic()
            self.get_logger().info('register: locking the person nearest straight ahead')
        elif m.data == 'clear':
            self.tm.clear()

    def _report(self):
        if self.tm_n:
            ms = self.tms / self.tm_n * 1e3
            self.get_logger().info(f'{self.tm_n / 10.0:.1f} frames/s | decode {ms[0]:.1f} yolo {ms[1]:.1f} track {ms[2]:.1f} '
                                   f'reid {ms[3]:.1f} ms | target {self.tm.state} id {self.tm.id} gallery {len(self.tm.gallery)}')
        self.tm_n, self.tms = 0, np.zeros(4)

    def _img(self, msg):
        now = time.monotonic()
        hz = self.hz if now - self.active_t < 1.0 else self.hz_idle
        if now - self.last_t < 0.7 / hz:              # 0.7: frame-arrival jitter must not skip every other frame
            return
        self.last_t = now
        t0 = time.monotonic()
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)   # full size: ReID crops need it
        if img is None:
            return
        H, W = img.shape[:2]
        if self.bm is None:
            c = self.cam_model
            K = c['K'] if len(c['K']) == 9 else None
            D = c['D'] if len(c['D']) == 4 else None
            self.bm = tc.BearingModel(c['model'], W, H, fx=c['fx'], cx=c['cx'], hfov_deg=c['hfov_deg'], K=K, D=D)
        t1 = time.monotonic()
        S = self.yolo.size
        r = min(S / W, S / H)
        nw, nh = int(round(W * r)), int(round(H * r))
        canvas = np.full((S, S, 3), 114, np.uint8)
        dx, dy = (S - nw) // 2, (S - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
        out = self.yolo(canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0)[0]
        pose = out.shape[0] == 56
        score = out[4]
        k = np.flatnonzero(score > self.det_low)
        boxes, scores, kps = [], [], []
        for j in k:
            cxb, cyb, wb, hb = out[0, j], out[1, j], out[2, j], out[3, j]
            boxes.append([float((cxb - wb / 2 - dx) / r), float((cyb - hb / 2 - dy) / r), float(wb / r), float(hb / r)])
            scores.append(float(score[j]))
            if pose:
                kp = out[5:, j].reshape(17, 3).copy()
                kp[:, 0] = (kp[:, 0] - dx) / r
                kp[:, 1] = (kp[:, 1] - dy) / r
                kps.append(kp)
        dets, dsc, dex = [], [], []
        if boxes:
            for i in np.array(cv2.dnn.NMSBoxes(boxes, scores, self.det_low, 0.5)).reshape(-1):
                bx, by, bw, bh = boxes[i]
                dets.append([max(0.0, bx), max(0.0, by), min(W, bx + bw), min(H, by + bh)])
                dsc.append(scores[i])
                dex.append(kps[i] if pose else None)
        t2 = time.monotonic()
        tracks = self.bt.update(dets, dsc, dex)
        fr = self.bt.frame
        t3 = time.monotonic()
        emb = {}
        ask = self.tm.need_embeddings(tracks, fr) if self.reid is not None else []
        if ask:
            sel = [t for t in tracks if t.id in ask]
            crops = []
            for t in sel:
                x0, y0, x1, y1 = [int(v) for v in t.box]
                x0, y0, x1, y1 = max(0, x0), max(0, y0), min(W, x1), min(H, y1)
                crops.append(img[y0:y1, x0:x1] if x1 - x0 > 8 and y1 - y0 > 16 else None)
            ok = [(t, c) for t, c in zip(sel, crops) if c is not None]
            if ok:
                for (t, _), e in zip(ok, self.reid([c for _, c in ok])):
                    emb[t.id] = e
        tg = self.tm.update(tracks, fr, self._bearing, emb)
        t4 = time.monotonic()
        self.tms += np.array([t1 - t0, t2 - t1, t3 - t2, t4 - t3])
        self.tm_n += 1
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        people = []
        for t in tracks:
            if t.last != fr or t.score < self.conf:
                continue
            x0, y0, x1, y1 = [int(v) for v in t.box]
            d = {'id': t.id, 'box': [x0, y0, x1, y1], 'score': round(float(t.score), 3), 'target': tg is not None and t.id == tg.id,
                 'bearing_deg': round(math.degrees(self._bearing(t)), 1)}
            if t.extra is not None:
                d['kps'] = [[round(float(q[0]), 1), round(float(q[1]), 1), round(float(q[2]), 2)] for q in t.extra]
            people.append(d)
        self.pub.publish(String(data=json.dumps({'stamp': st, 'w': W, 'h': H, 'people': people})))
        j = {'stamp': st, 'state': self.tm.state, 'id': self.tm.id, 'w': W, 'h': H}
        if tg is not None:
            b = tg.box
            j.update(box=[int(v) for v in b], bearing_deg=round(math.degrees(self._bearing(tg)), 2),
                     span_deg=[round(math.degrees(self.bm.bearing(b[0])), 2), round(math.degrees(self.bm.bearing(b[2])), 2)],
                     feet_row=round(float(self._feet_row(tg)), 1), score=round(float(tg.score), 3),
                     sim=None if tg.id not in emb else round(float(self.tm.similarity(emb[tg.id]) or 0.0), 3))
        self.pub_t.publish(String(data=json.dumps(j)))

    def _bearing(self, t):
        """Bearing of a track: ankles / knees / hips middle when seen (the ground point), else the box centre."""
        kp = t.extra
        u = (t.box[0] + t.box[2]) / 2
        if kp is not None:
            for a_, b_ in ((15, 16), (13, 14), (11, 12)):
                sel = [q for q in (a_, b_) if kp[q, 2] > 0.5]
                if sel:
                    u = float(np.mean(kp[sel, 0]))
                    break
        return self.bm.bearing(u)

    @staticmethod
    def _feet_row(t):
        kp = t.extra
        if kp is not None:
            rows = [kp[q, 1] for q in (15, 16) if kp[q, 2] > 0.5]
            if rows:
                return max(rows)
        return t.box[3]


def main():
    rclpy.init()
    try:
        rclpy.spin(PersonTracker())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
