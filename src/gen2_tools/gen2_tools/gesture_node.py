#!/usr/bin/env python3
"""Hand-gesture commands for C-WANG (MediaPipe GestureRecognizer, CPU ~55 ms/frame on the Orin Nano).

Runs with the MediaPipe venv:  ~/venv_vision/bin/python -m gen2_tools.gesture_node  (ROS sourced; the venv
uses the system rclpy / OpenCV). Model: ~/gen2_ws/models/gesture_recognizer.task (Google, 7 gestures):
Closed_Fist, Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory, ILoveYou.

A gesture counts after it is held for hold_s with score >= min_score; then its action runs once
(again only after the hand changes). Gaps up to gap_s (no hand / other) do not restart the hold.

Recognition boosts (GCS 2026-10-26, measured on 632 robot-camera frames, user at ~1 m seated, purple ISP tint):
- hand-shape rule on the 21 landmarks when the canned classifier says None: Open_Palm = 4 fingers + thumb
  extended and fingers pointing up; Thumb_Up = 4 fingers curled, thumb extended and the highest point.
  The canned model wants the palm facing the camera — a raised, angled palm was 8/117 frames, with the rule 107/117.
- crop fallback: no hand in the full 640x480 frame -> left / right square halves upscaled x2 (small, far hands).
  Far gestures 93 -> 297 / 351 frames; idle (hand on mouse, drinking) 0 held detections. Costs ~140 ms on frames
  without a hand (laptop CPU); the side that found the hand is tried first next time.
- body pose for far signals (GCS 2026-10-28, user "두 팔 벌리거나 떰스업 하면 재개"): person_tracker keypoints on
  people/camera (COCO 17) of the LOCKED person (target) — a hand is ~18 px at 3 m, a body pose is still clear.
  Both arms spread sideways = restart (as Thumb_Up), one arm straight up above the head = stop (as Open_Palm).
  A hand result (MediaPipe / shape rule) wins over the pose (a thumb held high stays a thumb). Actions (param `actions`, "Gesture:action" strings):
  stop / restart          -> follow/hold (SetBool true / false) — competition hand signals
  follow_on / follow_off  -> follow/enable (SetBool)
  sit                     -> balance/sit
  start                   -> balance/start  (the robot must be held upright — off by default)
Topics: gesture (std_msgs/String "name score"), gesture/action (std_msgs/String).
"""

import json
import os
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import rclpy.executors
from gen2_tools.live import Live
from mediapipe.tasks.python import BaseOptions, vision
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        model = os.path.expanduser(p('model', '~/gen2_ws/models/gesture_recognizer.task'))
        self.hz = p('process_hz', 5.0)
        self.min_score = p('min_score', 0.6)
        self.hold_s = p('hold_s', 0.6)
        self.gap_s = p('gap_s', 0.3)               # missed frames shorter than this keep the hold going
        self.use_rule = p('shape_rule', True)
        self.crop = p('crop_fallback', True)
        self.side = 0                              # crop side (0 left, 1 right) that last found a hand
        self.use_pose = p('pose_gestures', True)   # far signals from body keypoints (people/camera)
        self.pose_any = p('pose_any_person', True)    # locked person (target) only while following; nobody locked -> largest person (False: none)
        self.ppl, self.ppl_t = None, -1e9
        acts = p('actions', ['Open_Palm:stop', 'Thumb_Up:restart'])   # rules: only stop / restart hand signals
        self.actions = dict(a.split(':', 1) for a in acts)
        opt = vision.GestureRecognizerOptions(base_options=BaseOptions(model_asset_path=model),
                                              running_mode=vision.RunningMode.IMAGE, num_hands=1)
        self.rec = vision.GestureRecognizer.create_from_options(opt)
        self.last_t = 0.0
        self.cur, self.cur_t, self.fired, self.seen_t = None, 0.0, None, 0.0
        self.pub = self.create_publisher(String, 'gesture', 10)
        self.pub_act = self.create_publisher(String, 'gesture/action', 10)
        self.cli_follow = self.create_client(SetBool, 'follow/enable')
        self.cli_hold = self.create_client(SetBool, 'follow/hold')
        self.cli = {k: self.create_client(Trigger, f'balance/{k}') for k in ('sit', 'start')}
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self._img, qos_profile_sensor_data)
        self.create_subscription(String, 'people/camera', self._people, 10)
        self.get_logger().info(f'gesture recognizer ready at {self.hz} Hz, actions {self.actions}')
        Live(self, {'process_hz': 'hz', 'min_score': 'min_score', 'hold_s': 'hold_s', 'gap_s': 'gap_s',
                    'shape_rule': 'use_rule', 'crop_fallback': 'crop', 'pose_gestures': 'use_pose',
                    'pose_any_person': 'pose_any'})

    def _recognize(self, rgb):
        """(name, score, landmarks or None) from one image."""
        r = self.rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb)))
        name, score = 'None', 0.0
        if r.gestures and r.gestures[0]:
            name, score = r.gestures[0][0].category_name, r.gestures[0][0].score
        lm = np.array([[q.x, q.y] for q in r.hand_landmarks[0]]) if r.hand_landmarks else None
        return name, score, lm

    def _people(self, m):
        try:
            self.ppl, self.ppl_t = json.loads(m.data).get('people', []), time.monotonic()
        except ValueError:
            pass

    @staticmethod
    def pose(kps):
        """Thumb_Up (both arms spread) / Open_Palm (one arm straight up) / None from 17 COCO keypoints [u, v, conf].
        Image y grows downwards; sizes relative to the shoulder width so any distance works."""
        K = np.asarray(kps, float)
        if K.shape[0] < 13:
            return None
        ok = lambda *i: all(K[j, 2] > 0.4 for j in i)  # noqa: E731
        if not ok(5, 6):
            return None
        ls, rs = K[5, :2], K[6, :2]
        sw = float(abs(ls[0] - rs[0]))
        if sw < 8:                                   # side view / too far: no reliable width
            return None
        cx, sy = 0.5 * (ls[0] + rs[0]), 0.5 * (ls[1] + rs[1])
        torso = float(0.5 * (K[11, 1] + K[12, 1]) - sy) if ok(11, 12) else 1.6 * sw
        torso = max(torso, sw)
        if ok(7, 8, 9, 10):                          # both arms out to the sides at shoulder height
            # 실기 (2026-10-28 T 자세, 316 프레임): 사람 상자가 몸통 폭만 잡아 YOLO 손목이 상자 끝에 붙는다 (어깨 중심에서
            # 0.6–2.1 sw, 대부분 1.1–1.25). 거리 대신 '어깨 높이' 를 주로 본다: 팔마다 손목·팔꿈치가 어깨 높이 ±0.45 torso,
            # 팔꿈치나 손목이 0.75 sw 이상 바깥, 두 팔이 반대쪽. 팔 내림 = 손목 +0.7–0.9 torso 아래, 쭈그림 = 팔꿈치 0.45–0.6 sw.
            arms = []
            for e, w in ((7, 9), (8, 10)):
                dxe, dxw = K[e, 0] - cx, K[w, 0] - cx
                level = abs(K[e, 1] - sy) < 0.45 * torso and abs(K[w, 1] - sy) < 0.45 * torso
                out = max(abs(dxe), abs(dxw)) >= 0.75 * sw and dxe * dxw > 0
                arms.append(np.sign(dxw) if level and out else 0.0)
            if arms[0] * arms[1] < 0:
                return 'Thumb_Up'
        for e, w in ((7, 9), (8, 10)):               # one arm straight up: wrist well above the head, elbow above the shoulder
            if ok(e, w) and K[w, 1] < sy - 0.9 * torso and K[e, 1] < sy - 0.3 * torso and abs(K[w, 0] - cx) < 1.5 * sw:
                return 'Open_Palm'
        return None

    def _pose_now(self, now):
        if not self.use_pose or self.ppl is None or now - self.ppl_t > 0.3:
            return None
        ppl = [q for q in self.ppl if q.get('kps')]
        tg = [q for q in ppl if q.get('target')]
        if tg:
            ppl = tg
        elif not self.pose_any:
            return None
        for q in sorted(ppl, key=lambda q: -(q['box'][2] - q['box'][0]) * (q['box'][3] - q['box'][1])):
            g = self.pose(q['kps'])
            if g:
                return g
        return None

    @staticmethod
    def shape(P):
        """Open_Palm / Thumb_Up from the landmarks (any hand rotation), else None. Image y grows downwards."""
        d = lambda a, b: float(np.linalg.norm(P[a] - P[b]))  # noqa: E731
        ext = [d(0, t) > 1.15 * d(0, j) for t, j in ((8, 6), (12, 10), (16, 14), (20, 18))]   # tip beyond PIP
        palm = d(0, 9)
        thumb_out = d(4, 5) > 0.55 * palm and d(4, 0) > d(3, 0)
        if all(ext) and thumb_out and P[[8, 12, 16, 20], 1].mean() < P[0, 1] - 0.8 * palm:
            return 'Open_Palm'
        if not any(ext) and thumb_out and P[4, 1] < P[2, 1] - 0.25 * palm and P[4, 1] < P[5:21, 1].min():
            return 'Thumb_Up'
        return None

    def _img(self, msg):
        now = time.monotonic()
        if now - self.last_t < 1.0 / self.hz:
            return
        self.last_t = now
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        name, score, lm = self._recognize(rgb)
        src = ''
        if lm is None and self.crop:               # small / far hand: square halves upscaled x2, last good side first
            h, w = rgb.shape[:2]
            for side in (self.side, 1 - self.side):
                x0 = 0 if side == 0 else w - h
                name, score, lm = self._recognize(cv2.resize(rgb[:, x0:x0 + h], (2 * h, 2 * h)))
                if lm is not None:
                    self.side, src = side, ' crop'
                    break
        if (score < self.min_score or name == 'None') and self.use_rule and lm is not None:
            g = self.shape(lm)
            if g:
                name, score, src = g, 1.0, src + ' rule'
        if score < self.min_score or name == 'None':     # no hand sign: far body pose of the locked person
            g = self._pose_now(now)
            if g:
                name, score, src = g, 1.0, ' pose'
        self.pub.publish(String(data=f'{name} {score:.2f}{src}'))
        if score < self.min_score or name == 'None':
            name = None
        if name is not None:
            self.seen_t = now
        elif self.cur is not None and now - self.seen_t < self.gap_s:
            return                                 # short gap: keep holding the current gesture
        if name != self.cur:
            self.cur, self.cur_t = name, now
            if name is None:
                self.fired = None
            return
        if name and name != self.fired and now - self.cur_t >= self.hold_s:
            self.fired = name
            self._act(name)

    def _act(self, name):
        act = self.actions.get(name)
        if not act:
            return
        self.pub_act.publish(String(data=f'{name} -> {act}'))
        self.get_logger().info(f'gesture {name} -> {act}')
        if act in ('stop', 'restart') and self.cli_hold.service_is_ready():
            self.cli_hold.call_async(SetBool.Request(data=act == 'stop'))
        elif act in ('follow_on', 'follow_off') and self.cli_follow.service_is_ready():
            self.cli_follow.call_async(SetBool.Request(data=act == 'follow_on'))
        elif act in self.cli and self.cli[act].service_is_ready():
            self.cli[act].call_async(Trigger.Request())


def main():
    rclpy.init()
    node = GestureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.rec.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
