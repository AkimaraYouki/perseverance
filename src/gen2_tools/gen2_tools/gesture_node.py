#!/usr/bin/env python3
"""Hand-gesture commands for C-WANG (MediaPipe GestureRecognizer, CPU ~55 ms/frame on the Orin Nano).

Runs with the MediaPipe venv:  ~/venv_vision/bin/python -m gen2_tools.gesture_node  (ROS sourced; the venv
uses the system rclpy / OpenCV). Model: ~/gen2_ws/models/gesture_recognizer.task (Google, 7 gestures):
Closed_Fist, Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory, ILoveYou.

A gesture counts after it is held for hold_s with score >= min_score; then its action runs once
(again only after the hand changes). Actions (param `actions`, "Gesture:action" strings):
  follow_on / follow_off  -> aruco/enable (SetBool)
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
        acts = p('actions', ['Open_Palm:follow_off', 'Thumb_Up:follow_on', 'Thumb_Down:sit'])
        self.actions = dict(a.split(':', 1) for a in acts)
        opt = vision.GestureRecognizerOptions(base_options=BaseOptions(model_asset_path=model),
                                              running_mode=vision.RunningMode.IMAGE, num_hands=1)
        self.rec = vision.GestureRecognizer.create_from_options(opt)
        self.last_t = 0.0
        self.cur, self.cur_t, self.fired = None, 0.0, None
        self.pub = self.create_publisher(String, 'gesture', 10)
        self.pub_act = self.create_publisher(String, 'gesture/action', 10)
        self.cli_follow = self.create_client(SetBool, 'aruco/enable')
        self.cli = {k: self.create_client(Trigger, f'balance/{k}') for k in ('sit', 'start')}
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self._img, qos_profile_sensor_data)
        self.get_logger().info(f'gesture recognizer ready at {self.hz} Hz, actions {self.actions}')

    def _img(self, msg):
        now = time.monotonic()
        if now - self.last_t < 1.0 / self.hz:
            return
        self.last_t = now
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        r = self.rec.recognize(img)
        name, score = 'None', 0.0
        if r.gestures and r.gestures[0]:
            name, score = r.gestures[0][0].category_name, r.gestures[0][0].score
        self.pub.publish(String(data=f'{name} {score:.2f}'))
        if score < self.min_score or name == 'None':
            name = None
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
        if act in ('follow_on', 'follow_off') and self.cli_follow.service_is_ready():
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
