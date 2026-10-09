#!/usr/bin/env python3
"""ArUco target following for C-WANG (runs on the robot).

Camera JPEG -> ArUco (OpenCV 4.6 legacy API) -> marker poses -> target in a *level* robot frame
(x forward, y left; body pitch / roll from the IMU shared memory at the frame time are removed, so the
balancing lean does not move the target) -> cmd_vel/auto (cmd_mux; the operator's sticks always win).

The target carries several markers with different IDs (`ids`): any visible subset is averaged, so a
covered or blurred marker does not lose the target.

Topics: aruco/target (geometry_msgs/PointStamped, frame base_level), aruco/debug (std_msgs/String).
Follow only while `follow` is true (service aruco/enable, or pad X via pad_teleop).
Camera intrinsics default to the IMX219 data-sheet field of view at 640x480 until calibrated.
"""

import math
import mmap
import struct
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool


def quat_to_rot(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class ArucoFollow(Node):
    def __init__(self):
        super().__init__('aruco_follow')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.dict_name = p('dictionary', 'DICT_4X4_50')
        self.size = p('marker_size_m', 0.08)          # docs/aruco sheet: 80 mm black square
        self.ids = set(p('ids', [0, 1, 2, 3]))
        self.hz = p('process_hz', 15.0)
        fx, fy, cx, cy = p('fx', 530.0), p('fy', 529.0), p('cx', 320.0), p('cy', 240.0)
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.D = np.array(p('dist', [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=np.float64)
        self.cam_xyz = np.array(p('camera_xyz', [0.108, 0.0, 0.0153]))       # base_link (desktop CAD)
        self.cam_pitch = math.radians(p('camera_pitch_deg', 0.0))            # + = looking down
        self.d_ref = p('follow_dist_m', 0.8)
        self.k_d, self.k_yaw = p('k_dist', 0.8), p('k_yaw', 1.5)
        self.vx_max, self.wz_max = p('vx_max', 0.4), p('wz_max', 1.0)
        self.lost_s = p('lost_timeout_s', 0.5)
        self.follow = p('follow', False)
        self.dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dict_name))
        self.params = cv2.aruco.DetectorParameters_create()
        self.shm = None
        self.last_t, self.target, self.target_t = 0.0, None, 0.0
        self.pub_t = self.create_publisher(PointStamped, 'aruco/target', 10)
        self.pub_dbg = self.create_publisher(String, 'aruco/debug', 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel/auto', 10)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self._img, qos_profile_sensor_data)
        self.create_service(SetBool, 'aruco/enable', self._enable)
        self.create_timer(0.05, self._control)
        self.get_logger().info(f'{self.dict_name} ids {sorted(self.ids)} size {self.size} m, follow {self.follow}')

    def _enable(self, req, res):
        self.follow = bool(req.data)
        res.success, res.message = True, f'follow {self.follow}'
        self.get_logger().info(res.message)
        return res

    def _attitude(self):
        """Body rotation without yaw (level <- body) from the IMU shared memory."""
        try:
            if self.shm is None:
                with open('/dev/shm/gen2_imu', 'rb') as f:
                    self.shm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            for _ in range(5):
                s0 = struct.unpack_from('<I', self.shm, 0)[0]
                w, x, y, z = struct.unpack_from('<4d', self.shm, 24)
                if s0 % 2 == 0 and struct.unpack_from('<I', self.shm, 0)[0] == s0:
                    R = quat_to_rot(w, x, y, z)
                    yaw = math.atan2(R[1, 0], R[0, 0])
                    c, s = math.cos(-yaw), math.sin(-yaw)
                    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R
        except (OSError, ValueError):
            self.shm = None
        return np.eye(3)

    def _img(self, msg):
        now = time.monotonic()
        if now - self.last_t < 1.0 / self.hz:
            return
        self.last_t = now
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.dict, parameters=self.params)
        R_lb = self._attitude()
        pts, seen = [], []
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                if int(i) not in self.ids:
                    continue
                _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([c], self.size, self.K, self.D)
                t = tvec.reshape(3)                                   # optical: x right, y down, z forward
                p_cam = np.array([t[2], -t[0], -t[1]])                # camera_link: x fwd, y left, z up
                cp, sp = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
                p_cam = np.array([cp * p_cam[0] + sp * p_cam[2], p_cam[1], -sp * p_cam[0] + cp * p_cam[2]])
                pts.append(R_lb @ (p_cam + self.cam_xyz))
                seen.append(int(i))
        dbg = f'markers {seen}'
        if pts:
            p = np.mean(pts, axis=0)
            self.target, self.target_t = p, now
            m = PointStamped()
            m.header.stamp, m.header.frame_id = msg.header.stamp, 'base_level'
            m.point.x, m.point.y, m.point.z = (float(v) for v in p)
            self.pub_t.publish(m)
            dbg += f' target x {p[0]:+.2f} y {p[1]:+.2f} z {p[2]:+.2f} m, dist {math.hypot(p[0], p[1]):.2f}, bearing {math.degrees(math.atan2(p[1], p[0])):+.0f} deg'
        self.pub_dbg.publish(String(data=dbg + f' follow {self.follow}'))

    def _control(self):
        if not self.follow:
            return
        t = Twist()
        if self.target is not None and time.monotonic() - self.target_t < self.lost_s:
            x, y = self.target[0], self.target[1]
            bearing, dist = math.atan2(y, x), math.hypot(x, y)
            t.angular.z = float(max(-self.wz_max, min(self.wz_max, self.k_yaw * bearing)))
            v = self.k_d * (dist - self.d_ref) * max(0.0, math.cos(bearing))   # slow down while turning toward it
            t.linear.x = float(max(-self.vx_max, min(self.vx_max, v)))
        self.pub_cmd.publish(t)          # target lost -> 0 (stand still), the operator can take over any time


def main():
    rclpy.init()
    try:
        rclpy.spin(ArucoFollow())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
