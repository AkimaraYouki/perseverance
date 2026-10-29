#!/usr/bin/env python3
"""C-WANG 사람 추종 HUD (데스크톱): 카메라 영상 + 사람 상자 + 추종 대상 + 라이다 다리 + 상태·수신호 + 위에서 본 미니맵.

로봇은 결과를 숫자로만 보낸다 — 여기서 겹쳐 그린다 (로봇 부담 0).
  /camera/image_raw/compressed   (JPEG 640x480, 180° 뒤집어 바로 선 영상)
  /people/camera   String JSON   {stamp, w, h, people: [{box: [x0,y0,x1,y1], score, bearing_deg, ...}]}   (person_detector,
                   또는 person_tracker: + id, target = 잠긴 사람 -> 초록 굵은 상자 "TARGET #id")
  /people/legs     PoseArray     laser 좌표 다리 쌍 = 사람 후보 (leg_detector)
  /follow/status   String        "FOLLOW target 1.23 m @ +5 deg" (person_follow, 대상 = laser 좌표)
  /gesture, /gesture/action String
  /balance/step_mode_active Bool, /balance/step_phase Int8 (latched)  턱 감지 모드 배지 (우상단)
실행: source ~/gen2_desk/env.sh; python3 ~/gen2_desk/hud.py    (q / Esc 끝, s 화면 저장)
렌즈 왜곡 보정 (2026-10-27): camera_cal.yaml (체커보드 fisheye K, D) 이 있으면 펴서 보여 준다 (~/gen2_desk/camera_cal.yaml,
  없으면 저장소 tools/robot/camera_cal.yaml). 사람 상자·관절도 같은 식으로 옮겨 그린다.
  u/Tab = 보정 켜기/끄기, [ ] = 좌우 기울기 (roll) -/+0.5°, , . = 앞뒤 숙임 (pitch) -/+0.5°, 0 = 기울기 0, w/Enter = 저장
  (~/.config/cw/camera_mount.json — 로봇 cam_tilt_deg 에 옮길 값도 화면에 표시). 수평선·세로선으로 맞춘다.
모든 정보 (사용자 2026-10-29): /follow/debug (추종 노드 JSON) -> 로봇 수평선 (하늘색), 발 목표선 (수평선 + feet_px, 초록 점선),
  지금 발 위치와 '수평선 아래 몇 px', 왼쪽 정보판 (상태·거리 출처·발 px·앞 정지·Nav2·자동 턱·명령·로봇 상태),
  미니맵에 라이다 (scan_level) 점과 앞 정지 구역 (빨강 = 서는 중). /cmd_vel, /controller/state 도 읽는다. i = 정보판 켜기/끄기
"""
import json
import math
import os
import re
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Int8, String
try:
    from gen2_msgs.msg import ControllerState
except ImportError:                                       # gen2_msgs 없이 띄우면 로봇 상태 줄만 빠짐
    ControllerState = None

FX, CX = 530.0, 320.0                  # person_detector 기본값 (보정 전 추정) — 보정 켜면 Rectifier 의 새 카메라 행렬로 바뀜
MOUNT_FILE = os.path.expanduser("~/.config/cw/camera_mount.json")


def load_cal():
    """camera_cal.yaml 의 person_tracker K, D (fisheye) -> (K 3x3, D 4) 또는 None."""
    import yaml
    for f in (os.path.expanduser("~/gen2_desk/camera_cal.yaml"),
              os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "robot", "camera_cal.yaml")):
        try:
            c = yaml.safe_load(open(f))["person_tracker"]["ros__parameters"]
            return np.array(c["K"], float).reshape(3, 3), np.array(c["D"], float).reshape(4, 1), f
        except (OSError, KeyError, TypeError, ValueError):
            continue
    return None


class Rectifier:
    """fisheye 왜곡 펴기 + 장착 기울기 (roll, pitch) 되돌리기. 점도 같은 변환 (상자·관절)."""

    def __init__(self, K, D, size=(640, 480), balance=0.5):
        self.K, self.D, self.size, self.balance = K, D, size, balance
        self.roll = self.pitch = 0.0
        try:
            m = json.load(open(MOUNT_FILE))
            self.roll, self.pitch = float(m.get("roll_deg", 0.0)), float(m.get("pitch_deg", 0.0))
        except (OSError, ValueError):
            pass
        self.build()

    def build(self):
        r, p = math.radians(self.roll), math.radians(self.pitch)
        Rz = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])   # 광축 둘레 (roll)
        Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])   # 좌우 축 둘레 (pitch)
        self.R = Rx @ Rz
        self.P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K, self.D, self.size, self.R, balance=self.balance)
        self.m1, self.m2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, self.R, self.P, self.size, cv2.CV_16SC2)

    def image(self, img):
        return cv2.remap(img, self.m1, self.m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def pts(self, uv):
        a = np.asarray(uv, np.float64).reshape(-1, 1, 2)
        return cv2.fisheye.undistortPoints(a, self.K, self.D, R=self.R, P=self.P).reshape(-1, 2)

    def box(self, b):
        x0, y0, x1, y1 = b
        e = [(x, y) for x in np.linspace(x0, x1, 5) for y in (y0, y1)] + [(x, y) for y in np.linspace(y0, y1, 5) for x in (x0, x1)]
        q = self.pts(e)
        return [int(q[:, 0].min()), int(q[:, 1].min()), int(q[:, 0].max()), int(q[:, 1].max())]

    def save(self):
        os.makedirs(os.path.dirname(MOUNT_FILE), exist_ok=True)
        json.dump(dict(roll_deg=round(self.roll, 2), pitch_deg=round(self.pitch, 2), note="cw HUD 2026-10-27; 로봇 cam_tilt_deg = pitch_deg"),
                  open(MOUNT_FILE, "w"), indent=1)
LASER_X, CAM_X = -0.0774, 0.1080       # base_link 기준 x [m] (footprint.yaml)
HITBOX = (-0.221, 0.162, -0.164, 0.164)
SKEL = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]   # COCO 팔·몸통·다리
STATE_COL = {"FOLLOW": (60, 200, 60), "OCCLUDED": (0, 200, 255), "SEARCH": (0, 140, 255), "HOLD": (60, 60, 230),
             "IDLE": (150, 150, 150)}


class Hud(Node):
    def __init__(self):
        super().__init__("cwang_hud")
        self.img, self.img_t, self.img_n = None, 0.0, 0
        self.people, self.people_t = [], 0.0
        self.legs, self.legs_t = [], 0.0
        self.status, self.status_t = "", 0.0
        self.gesture, self.action, self.action_t = "", "", 0.0
        self.create_subscription(CompressedImage, "camera/image_raw/compressed", self._img, qos_profile_sensor_data)
        self.create_subscription(String, "people/camera", self._people, 10)
        self.step_rays, self.step_rays_t = [], 0.0       # 카메라 턱 후보 (person_tracker step/rays, 2026-10-28 GCS)
        self.create_subscription(String, "step/rays", self._rays, 10)
        self.create_subscription(PoseArray, "people/legs", self._legs, 10)
        self.create_subscription(String, "follow/status", lambda m: self._set("status", m.data), 10)
        self.create_subscription(String, "gesture", lambda m: setattr(self, "gesture", m.data), 10)
        self.create_subscription(String, "gesture/action", self._action, 10)
        # 턱 감지 모드 (balance_node, 바뀔 때만 발행 + latched): 콜백은 값만 저장, 그리기는 draw() 에서
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.step_on, self.step_phase = False, 0
        self.create_subscription(Bool, "balance/step_mode_active", lambda m: setattr(self, "step_on", bool(m.data)), latched)
        self.create_subscription(Int8, "balance/step_phase", lambda m: setattr(self, "step_phase", int(m.data)), latched)
        self.fps_t, self.fps = time.monotonic(), 0.0
        self.dbg, self.dbg_t = {}, 0.0
        self.create_subscription(String, "follow/debug", self._dbg, 10)
        self.cmd, self.cmd_t = (0.0, 0.0), 0.0
        self.create_subscription(Twist, "cmd_vel", self._cmd, 10)
        self.scan, self.scan_t = np.zeros((0, 2)), 0.0
        self.create_subscription(LaserScan, "scan_level", self._scan, qos_profile_sensor_data)
        self.cs, self.cs_t = None, 0.0
        if ControllerState is not None:
            self.create_subscription(ControllerState, "controller/state", self._cs, 10)

    def _dbg(self, m):
        try:
            self.dbg, self.dbg_t = json.loads(m.data), time.monotonic()
        except json.JSONDecodeError:
            pass

    def _cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.angular.z), time.monotonic()

    def _scan(self, m):
        r = np.asarray(m.ranges, float)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 6.0))
        self.scan, self.scan_t = np.stack([r[ok] * np.cos(a[ok]) + LASER_X, r[ok] * np.sin(a[ok])], 1), time.monotonic()

    def _cs(self, m):
        if self.cs is None or time.monotonic() - self.cs_t > 0.1:      # 100 Hz -> 10 Hz 로만 저장
            self.cs, self.cs_t = m, time.monotonic()

    def _set(self, k, v):
        setattr(self, k, v); setattr(self, k + "_t", time.monotonic())

    def _img(self, m):
        a = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if a is not None:
            self.img, self.img_t = a, time.monotonic()
            self.img_n += 1

    def _rays(self, m):
        try:
            self.step_rays, self.step_rays_t = json.loads(m.data).get("rays", []), time.monotonic()
        except json.JSONDecodeError:
            pass

    def _people(self, m):
        try:
            self.people, self.people_t = json.loads(m.data).get("people", []), time.monotonic()
        except json.JSONDecodeError:
            pass

    def _legs(self, m):
        self.legs, self.legs_t = [(p.position.x, p.position.y) for p in m.poses], time.monotonic()

    def _action(self, m):
        self.action, self.action_t = m.data, time.monotonic()


def cam_bearing(xl, yl):
    """laser 좌표 점 -> 카메라 방위 [rad] (+ = 왼쪽). laser 와 카메라는 x 로 18.5 cm 떨어져 있어 가까울수록 다르다."""
    return math.atan2(yl, xl + LASER_X - CAM_X)


def u_of(b):
    return int(round(CX - FX * math.tan(b))) if abs(b) < math.radians(80) else None


def _px(rect, pts):
    """Raw-image points -> drawing points (rectified view: same lens transform as the boxes)."""
    q = rect.pts(pts) if rect is not None else np.asarray(pts, float)
    return [(int(round(a)), int(round(b))) for a, b in q]


def _dashed(img, pts, col, on=8, off=6):
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        L = max(1, int(math.hypot(x1 - x0, y1 - y0)))
        for s_ in range(0, L, on + off):
            e_ = min(L, s_ + on)
            cv2.line(img, (x0 + (x1 - x0) * s_ // L, y0 + (y1 - y0) * s_ // L), (x0 + (x1 - x0) * e_ // L, y0 + (y1 - y0) * e_ // L),
                     col, 2, cv2.LINE_AA)


def _info_lines(n: Hud, now):
    """(text, colour) lines for the info panel from follow/debug, cmd_vel and controller/state."""
    out = []
    dd = n.dbg if now - n.dbg_t < 1.0 else {}
    if dd:
        st = dd.get("state", "?")
        out.append((f"{st}{'  NAV2' if dd.get('nav') else ''}{'  HOLD' if dd.get('hold') else ''}{'  blocked' if dd.get('blocked') else ''}",
                    STATE_COL.get(st.split("+")[0], (230, 230, 230))))
        if dd.get("dist") is not None:
            out.append((f"target {dd['dist']:.2f} m @ {dd.get('bearing_deg', 0):+.0f} deg  [{dd.get('src')}]  keep {dd.get('d_ref', 0):.2f} m",
                        (230, 230, 230)))
        f = dd.get("feet") or {}
        if f.get("dv") is not None or f.get("on"):
            dv = f.get("dv")
            out.append((f"feet {'ON ' if f.get('on') else 'off'}  {('%.0f' % dv) if dv is not None else '-'} / {f.get('px', 0):.0f} px below horizon"
                        f"  (k {f.get('k', 0):.3f})", (60, 220, 60) if f.get("on") else (170, 170, 170)))
        fr = dd.get("front") or {}
        if fr.get("stop_m", 0) > 0:
            mm = fr.get("min_m")
            out.append((f"front lidar {('%.2f m' % mm) if mm is not None else '-'} / stop {fr['stop_m']:.2f} m" + ("  STOP" if fr.get("hit") else ""),
                        (60, 60, 255) if fr.get("hit") else (230, 230, 230)))
        a = dd.get("auto") or {}
        if a.get("on"):
            sd = a.get("step_d")
            out.append((f"auto step {a.get('state')}" + (f"  step {sd:.1f} m" if sd else ""), (0, 165, 255)))
        out.append((f"explore {'on' if dd.get('explore') else 'off'}  leg {dd.get('leg_h', 0) * 1000:.0f} mm  cam pitch {dd.get('pitch_deg', 0):+.1f} deg",
                    (170, 170, 170)))
    else:
        out.append(("no follow/debug (follow node old or off)", (150, 150, 150)))
    if now - n.cmd_t < 1.0:
        out.append((f"cmd vx {n.cmd[0]:+.2f} m/s  wz {n.cmd[1]:+.2f} rad/s", (230, 230, 230)))
    cs = n.cs if n.cs is not None and now - n.cs_t < 1.0 else None
    if cs is not None:
        out.append((f"robot {cs.mode}  v {cs.v:+.2f}  pitch {math.degrees(cs.pitch):+.1f}  roll {math.degrees(cs.roll):+.1f}"
                    f"  h {cs.h[0] * 1000:.0f}/{cs.h[1] * 1000:.0f}", (230, 230, 230) if cs.mode == "balance" else (0, 200, 255)))
        if cs.fault:
            out.append((f"fault: {cs.fault}", (60, 60, 255)))
    return out


def draw(n: Hud, rect=None, panel=True):
    global FX, CX
    now = time.monotonic()
    H, W = 480, 640
    img = n.img.copy() if n.img is not None and now - n.img_t < 1.0 else np.full((H, W, 3), 40, np.uint8)
    if rect is not None and n.img is not None and now - n.img_t < 1.0:
        img = rect.image(img)
        FX, CX = float(rect.P[0, 0]), float(rect.P[0, 2])     # 방위 -> 열 (라이다 표시·추종 대상 선)
    if n.img is None or now - n.img_t >= 1.0:
        cv2.putText(img, "NO CAMERA", (W // 2 - 90, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    H, W = img.shape[:2]
    # 추종 대상 (laser 좌표)
    st = n.status if now - n.status_t < 1.5 else ""
    state = st.split()[0] if st else "NO STATUS"
    tgt = None
    m = re.search(r"target ([\d.]+) m @ ([+-]?\d+) deg", st)
    if m:
        d, b = float(m.group(1)), math.radians(float(m.group(2)))
        tgt = (d * math.cos(b), d * math.sin(b))
    tb = cam_bearing(*tgt) if tgt else None
    # 사람 상자
    fresh = now - n.people_t < 1.0
    for p in (n.people if fresh else []):
        x0, y0, x1, y1 = rect.box(p["box"]) if rect is not None else p["box"]
        b = math.radians(p.get("bearing_deg", 0.0))
        # 새 스택 (person_tracker): 잠긴 사람이 데이터에 표시됨 (id·target). 옛 스택: follow/status 방위 근처 상자
        is_t = bool(p["target"]) if "target" in p else (tb is not None and abs(b - tb) < math.radians(8))
        col, th = ((60, 220, 60), 3) if is_t else ((200, 200, 200), 1)
        cv2.rectangle(img, (x0, y0), (x1, y1), col, th)
        kp = p.get("kps")
        if kp and rect is not None:
            q = rect.pts([(a[0], a[1]) for a in kp])
            kp = [[float(q[i, 0]), float(q[i, 1]), kp[i][2]] for i in range(len(kp))]
        if kp:   # COCO 17 점: 하반신 (11–16 엉덩이·무릎·발목) 은 방위 계산에 쓰여 굵게
            for i, j in SKEL:
                if kp[i][2] > 0.3 and kp[j][2] > 0.3:
                    lower = i >= 11 and j >= 11
                    cv2.line(img, (int(kp[i][0]), int(kp[i][1])), (int(kp[j][0]), int(kp[j][1])),
                             (0, 220, 255) if lower else (180, 180, 180), 3 if lower else 1, cv2.LINE_AA)
            for q, (u_, v_, cf) in enumerate(kp):
                if cf > 0.3:
                    cv2.circle(img, (int(u_), int(v_)), 4 if q >= 11 else 2, (0, 220, 255) if q >= 11 else (220, 220, 220), -1)
        lab = f"{'TARGET ' if is_t else ''}{('#' + str(p['id']) + ' ') if 'id' in p else ''}{p.get('score', 0):.2f}" + (f" ({p['bearing_from']})" if p.get("bearing_from") else "")
        cv2.putText(img, lab, (x0 + 3, max(14, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1 + is_t)
    # 라이다 다리 (화면 아래 띠에 방위로)
    for (xl, yl) in (n.legs if now - n.legs_t < 1.0 else []):
        u = u_of(cam_bearing(xl, yl))
        if u is not None and 0 <= u < W:
            cv2.drawMarker(img, (u, H - 30), (255, 200, 0), cv2.MARKER_TRIANGLE_UP, 14, 2)
            cv2.putText(img, f"{math.hypot(xl, yl):.1f}m", (u - 16, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
    # 카메라 턱 후보 선 (주황): 원본 영상 픽셀 -> 보정 화면이면 같은 변환. 거리는 카메라 높이 0.27 m, 몸 기울기 0 으로 어림 (~)
    for r in (n.step_rays if now - n.step_rays_t < 0.6 else []):
        if len(r) < 4 or not r[3]:
            continue
        q = rect.pts(r[3]) if rect is not None else np.asarray(r[3], float)
        pts = np.asarray(q, np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, (0, 165, 255), 3, cv2.LINE_AA)
        d = 0.27 / math.tan(r[0]) if r[0] > math.radians(2) else None
        x0, y0 = int(pts[0, 0, 0]), int(pts[0, 0, 1])
        cv2.putText(img, f"step ~{d:.1f}m" if d and d < 5 else "step", (max(2, x0), max(14, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
    if tb is not None:
        u = u_of(tb)
        if u is not None and 0 <= u < W:
            cv2.line(img, (u, 40), (u, H - 45), (60, 220, 60), 1, cv2.LINE_AA)
    if rect is not None:                                   # 기울기 맞추기: 수평선 (가운데 행) + 세로선 + 값
        cy_, cx_ = int(rect.P[1, 2]), int(rect.P[0, 2])
        cv2.line(img, (0, cy_), (W, cy_), (255, 0, 255), 1, cv2.LINE_AA)
        for xx in (cx_ - 160, cx_, cx_ + 160):
            cv2.line(img, (xx, 40), (xx, H - 40), (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"rectified  roll {rect.roll:+.1f}  pitch {rect.pitch:+.1f} deg  ([ ] , . 0  Enter=save  Tab=raw)", (8, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
    # 로봇 수평선 (하늘색) + 발 목표선 (수평선 + feet_px, 초록 점선) + 지금 발 (원본 영상 픽셀 -> 보정 화면이면 같은 변환)
    dd = n.dbg if now - n.dbg_t < 1.0 else {}
    hz = dd.get("horizon") or []
    f = dd.get("feet") or {}
    if len(hz) >= 2:
        try:
            line = _px(rect, [(u_, v_) for u_, v_ in hz])
            cv2.polylines(img, [np.asarray(line, np.int32).reshape(-1, 1, 2)], False, (255, 200, 60), 1, cv2.LINE_AA)
            cv2.putText(img, "horizon", (6, max(14, line[0][1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 60), 1)
            px = float(f.get("px", 0))
            goal = _px(rect, [(u_, v_ + px) for u_, v_ in hz])
            _dashed(img, goal, (60, 220, 60) if f.get("on") else (120, 160, 120))
            cv2.putText(img, f"feet goal {px:.0f}px", (6, max(14, goal[0][1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (60, 220, 60) if f.get("on") else (120, 160, 120), 1)
            if f.get("u") is not None and f.get("row") is not None:
                (fu, fv), = _px(rect, [(f["u"], f["row"])])
                (hu, hv), = _px(rect, [(f["u"], f["row"] - f["dv"])])
                cv2.line(img, (fu, hv), (fu, fv), (60, 220, 60), 2, cv2.LINE_AA)
                cv2.circle(img, (fu, fv), 6, (60, 220, 60), -1)
                cv2.putText(img, f"{f['dv']:.0f}/{px:.0f}px", (fu + 8, fv - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 60), 2)
        except (cv2.error, ValueError):
            pass
    # 정보판 (왼쪽, 반투명)
    if panel:
        lines = _info_lines(n, now)
        y0 = 100 if rect is not None else 80              # 보정 화면: 기울기 안내 줄 (y 70) 아래
        ov = img.copy()
        cv2.rectangle(ov, (4, y0 - 16), (4 + 9 * max(len(t) for t, _ in lines) // 1 + 8, y0 + 18 * len(lines) - 6), (20, 20, 20), -1)
        img[:] = cv2.addWeighted(ov, 0.55, img, 0.45, 0)
        for i, (t, c) in enumerate(lines):
            cv2.putText(img, t, (8, y0 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
    # 상단 띠: 상태 + 수신호
    cv2.rectangle(img, (0, 0), (W, 34), (25, 25, 25), -1)
    stt = st if st else "no follow/status (person_follow not running)"
    tw_ = cv2.getTextSize(stt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0]
    sc_ = 0.6 if tw_ <= W - 150 else max(0.35, 0.6 * (W - 150) / tw_)   # 긴 상태줄은 작게 (오른쪽 손짓 칸 전까지)
    cv2.putText(img, stt, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, sc_, STATE_COL.get(state.split("+")[0], (200, 200, 200)), 1 if sc_ < 0.6 else 2)
    g = n.gesture.split()[0] if n.gesture else ""
    gtxt = f"hand: {g}" + (f"  [{n.action}]" if n.action and now - n.action_t < 3 else "")
    cv2.putText(img, gtxt, (W - 8 - 9 * len(gtxt), 23), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    # 턱 감지 모드 배지 (우상단, 켜졌을 때만): 0 접근, 1 막힘 -> 후진, 2 재주행 -> 점프
    if n.step_on:
        ph_txt = {0: "APPROACH", 1: "BLOCKED > BACK", 2: "RUN > JUMP"}.get(n.step_phase, "")
        (tw, _), _ = cv2.getTextSize("STEP", cv2.FONT_HERSHEY_DUPLEX, 1.1, 2)
        (pw, _), _ = cv2.getTextSize(ph_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        bw = max(tw, pw) + 24
        x0, y0 = W - bw - 8, 42
        cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + 70), (255, 255, 255), -1)
        cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + 70), (0, 140, 255), 3)
        cv2.putText(img, "STEP", (x0 + (bw - tw) // 2, y0 + 36), cv2.FONT_HERSHEY_DUPLEX, 1.1, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(img, ph_txt, (x0 + (bw - pw) // 2, y0 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 110, 230), 2, cv2.LINE_AA)
    # 미니맵 (위에서 본 모습, base_link, 위 = 앞)
    S, R = 170, 3.0
    mp = np.full((S, S, 3), 25, np.uint8)
    c = (S // 2, S - 20)
    sc = (S - 30) / R
    to = lambda x, y: (int(c[0] - y * sc), int(c[1] - x * sc))  # noqa: E731
    for r in (1, 2, 3):
        cv2.circle(mp, c, int(r * sc), (70, 70, 70), 1)
    cv2.rectangle(mp, to(HITBOX[1], HITBOX[3]), to(HITBOX[0], HITBOX[2]), (180, 120, 40), 1)
    for (xs, ys) in (n.scan[::2] if now - n.scan_t < 1.0 else []):
        if abs(xs) < R and abs(ys) < R:
            cv2.circle(mp, to(xs, ys), 1, (170, 170, 170), -1)
    fr = dd.get("front") or {}
    if fr.get("stop_m", 0) > 0:
        hw = fr.get("half_w", 0.2)
        cv2.rectangle(mp, to(fr["stop_m"], hw), to(0.05, -hw), (60, 60, 255) if fr.get("hit") else (90, 90, 160), 2 if fr.get("hit") else 1)
    for (xl, yl) in (n.legs if now - n.legs_t < 1.0 else []):
        cv2.circle(mp, to(xl + LASER_X, yl), 3, (255, 200, 0), -1)
    if tgt:
        cv2.circle(mp, to(tgt[0] + LASER_X, tgt[1]), 7, (60, 220, 60), 2)
    cv2.putText(mp, "1/2/3 m", (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    img[H - S - 4:H - 4, W - S - 4:W - 4] = mp
    # 바닥 정보
    n.fps = 0.9 * n.fps + 0.1 * (n.img_n / max(1e-3, now - n.fps_t)); n.img_n, n.fps_t = 0, now
    cv2.putText(img, f"cam {n.fps:4.1f} fps  people {len(n.people) if fresh else '-'}  legs {len(n.legs) if now - n.legs_t < 1 else '-'}",
                (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
    return img


def main():
    rclpy.init()
    n = Hud()
    cal = load_cal()
    rect = Rectifier(cal[0], cal[1]) if cal else None
    on = rect is not None
    panel = True
    if cal:
        n.get_logger().info(f"lens calibration {cal[2]} — u toggles the rectified view")
    cv2.namedWindow("C-WANG HUD", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("C-WANG HUD", 960, 720)
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            frame = draw(n, rect if on else None, panel)
            cv2.imshow("C-WANG HUD", frame)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("i"):
                panel = not panel
            if k == ord("s"):
                p = os.path.expanduser(f"~/Desktop/hud_{time.strftime('%H%M%S')}.png")
                cv2.imwrite(p, frame); n.get_logger().info(f"saved {p}")
            if rect is not None:
                if k in (ord("u"), 9):                         # u 또는 Tab (한글 입력이면 u 가 안 들어옴)
                    on = not on
                d = {ord("["): (-0.5, 0), ord("]"): (0.5, 0), ord(","): (0, -0.5), ord("."): (0, 0.5)}.get(k)
                if d:
                    rect.roll += d[0]; rect.pitch += d[1]; rect.build()  # noqa: E702
                if k == ord("0"):
                    rect.roll = rect.pitch = 0.0; rect.build()  # noqa: E702
                if k in (ord("w"), 13, 10):                    # w 또는 Enter (한글 입력이면 w 가 ㅈ)
                    rect.save()
                    n.get_logger().info(f"saved {MOUNT_FILE}: roll {rect.roll:+.1f} pitch {rect.pitch:+.1f} deg "
                                        f"(robot person_follow cam_tilt_deg = {rect.pitch:+.1f})")
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
