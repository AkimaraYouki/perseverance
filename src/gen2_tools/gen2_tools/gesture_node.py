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
  without a hand (laptop CPU); the side that found the hand is tried first next time. Actions (param `actions`, "Gesture:action" strings):
  stop / restart          -> follow/hold (SetBool true / false) — competition hand signals
  follow_on / follow_off  -> follow/enable (SetBool)
  sit                     -> balance/sit
  start                   -> balance/start  (the robot must be held upright — off by default)
Topics: gesture (std_msgs/String "name score"), gesture/action (std_msgs/String).
"""

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
        self.get_logger().info(f'gesture recognizer ready at {self.hz} Hz, actions {self.actions}')
        Live(self, {'process_hz': 'hz', 'min_score': 'min_score', 'hold_s': 'hold_s', 'gap_s': 'gap_s',
                    'shape_rule': 'use_rule', 'crop_fallback': 'crop'})

    def _recognize(self, rgb):
        """(name, score, landmarks or None) from one image."""
        r = self.rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb)))
        name, score = 'None', 0.0
        if r.gestures and r.gestures[0]:
            name, score = r.gestures[0][0].category_name, r.gestures[0][0].score
        lm = np.array([[q.x, q.y] for q in r.hand_landmarks[0]]) if r.hand_landmarks else None
        return name, score, lm

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
