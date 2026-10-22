#!/usr/bin/env python3
"""C-WANG 관제 GUI — 기능 켜기/끄기 + 동작 파라미터 튜닝 (GCS 노트북, 데스크톱 겸용).

  cw-gui      (= source ~/gen2_desk/env.sh && python3 ~/gen2_desk/cw_gui.py, 바탕화면 아이콘 "C-WANG 관제")

- 로봇 쪽 (추종 스택) 은 cw 와 같은 ssh 스크립트로, 노트북 쪽 (패드·손짓·SLAM·RViz·HUD) 은 터미널 없이 뒤에서 띄운다
  (로그 ~/gen2_desk/logs/*.log, '로그' 탭). cw check / cw down 과 같은 프로세스 패턴이라 섞어 써도 된다.
- 튜닝 탭: 게인 (최적화된 값) 은 다루지 않는다. 속도 한계, 추종·인식·장애물 회피·SLAM 의 동작 파라미터만.
  칸마다 언제 적용되는지 표시: 실시간 (= 로봇 해제 (disarmed) 하고 변경, 서 있으면 잠김) / 재시작 때 (노트북에서 도는 패드·손짓·SLAM) / 잠김 (노드가 실시간 변경을 안 받음).
  노드가 `live_params` (string array) 를 내놓으면 그 이름들은 실시간으로 바뀐다 (comms 2026-10-13_1300).
- 다른 컴퓨터 (데스크톱) 에서 패드·SLAM 이 이미 돌면 켜지 않는다 (heartbeat 2 개 / TF 충돌 방지).
"""
import glob
import math
import os
import re
import signal
import struct
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import yaml
from PyQt5 import QtCore, QtGui, QtWidgets

from attitude_view import AttitudeView, AttitudeWindow   # cw_gui.py 옆 파일들 (setup_gcs.sh 가 같이 복사)
from status_panel import StatusPanel

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

ROBOT = 'parksudo@192.168.50.124'
HOME = os.path.expanduser('~')
DESK = f'{HOME}/gen2_desk'
LOGS = f'{DESK}/logs'
PRESETS = f'{DESK}/gui_presets'
LENV = f'source {DESK}/env.sh'
SLAM_BASE = f'{DESK}/install/gen2_bringup/share/gen2_bringup/config/slam.yaml'
SLAM_YAML = f'{DESK}/slam_gui.yaml'
os.environ['PATH'] = f'{HOME}/bin:' + os.environ.get('PATH', '')   # 바탕화면 아이콘으로 켜면 ~/bin 이 없다 (wifi, cw)
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4', '-o', 'ControlMaster=auto',
       '-o', f'ControlPath={HOME}/.ssh/cm-%r@%h:%p', '-o', 'ControlPersist=120']

# ------------------------------------------------------------------ 파라미터 표 (게인 제외)
# (노드, 이름, 설명, 최소, 최대, 간격, 기본값, 단위)  기본값 = 소스/설정 파일 값 (2026-10-15). bool 이면 최소~단위 무시
PF, RPP, LCM, GCM = '/person_follow', '/controller_server', '/local_costmap/local_costmap', '/global_costmap/global_costmap'
GROUPS = [
    ('주행 속도 한계', '조종·추종·우회 각각의 최대 속도. 균형 노드 한계 (맨 아래) 가 전체 상한', [
        ('/pad_teleop', 'vx_max', '패드 최대 전진 속도', 0.1, 1.2, 0.05, 0.83, 'm/s'),   # 0.83 = 3 km/h (로봇 10-14)
        ('/pad_teleop', 'wz_max', '패드 최대 회전 속도', 0.2, 3.0, 0.1, 2.0, 'rad/s'),
        (PF, 'vx_max', '추종 최대 전진 속도', 0.0, 1.2, 0.05, 0.83, 'm/s'),
        (PF, 'wz_max', '추종 최대 회전 속도', 0.0, 2.5, 0.1, 1.0, 'rad/s'),
        (PF, 'search_wz', '사람 찾을 때 회전 속도', 0.0, 1.5, 0.05, 0.5, 'rad/s'),
        (RPP, 'FollowPath.desired_linear_vel', '우회 주행 속도 (Nav2)', 0.1, 1.2, 0.05, 0.83, 'm/s'),
        (RPP, 'FollowPath.rotate_to_heading_angular_vel', '우회 제자리 회전 속도', 0.1, 2.5, 0.1, 1.0, 'rad/s'),
        (RPP, 'FollowPath.regulated_linear_scaling_min_speed', '우회 곡선 최소 속도', 0.05, 0.7, 0.05, 0.25, 'm/s'),
        (RPP, 'FollowPath.min_approach_linear_velocity', '우회 목표 근처 최소 속도', 0.0, 0.5, 0.05, 0.1, 'm/s'),
        ('/balance', 'tune.vmax_kmh', '균형 노드 최고 속도 (전체 상한)', 0.5, 3.5, 0.1, 3.0, 'km/h'),   # 로봇이 0.5–3.5 밖은 거절
    ]),
    ('사람 추종', '누구를 얼마나 떨어져서 따라가는지, 놓쳤을 때 어떻게 하는지', [
        (PF, 'follow_dist_m', '따라갈 거리', 0.3, 2.0, 0.05, 0.8, 'm'),
        (PF, 'bearing_tol_deg', '방향 허용 오차 (이 안이면 회전 안 함)', 0.0, 30.0, 1.0, 10.0, '°'),
        (PF, 'occluded_s', '가려짐 허용 시간 (넘으면 찾기)', 0.0, 15.0, 0.5, 5.0, 's'),
        (PF, 'predict_s', '가려졌을 때 예측해서 갈 시간', 0.0, 5.0, 0.1, 1.5, 's'),
        (PF, 'grace_s', '잠깐 놓쳐도 추종 유지', 0.0, 3.0, 0.1, 0.5, 's'),
        (PF, 'legs_only_s', '카메라 없이 다리만으로 이어 가기', 0.0, 5.0, 0.1, 1.0, 's'),
        (PF, 'max_gate_m', '대상이 한 번에 움직일 수 있는 거리', 0.5, 5.0, 0.1, 2.5, 'm'),
        (PF, 'appearance_min', '옷 색 일치 기준 (근처)', 0.0, 1.0, 0.05, 0.5, ''),
        (PF, 'reacquire_min', '옷 색 일치 기준 (오래 놓친 뒤)', 0.0, 1.0, 0.05, 0.65, ''),
    ]),
    ('카메라로 사람 확인 · 거리 추정', '물건 (의자 다리 등) 을 사람으로 따라가지 않게 카메라로 다시 확인. '
                               '라이다 다리가 없으면 발목의 화면 높이로 거리 추정 (카메라만 추적 켜기/끄기는 첫 탭)', [
        (PF, 'cam_confirm_s', '카메라 시야 안에서 카메라 확인 없이 버틸 시간', 0.1, 3.0, 0.1, 0.5, 's'),
        (PF, 'cam_fov_deg', '카메라 시야 (이 각도 안이면 카메라 확인 필요)', 5.0, 45.0, 1.0, 25.0, '°'),
        (PF, 'cam_only_score', '카메라만 추적할 때 최소 확신도', 0.3, 0.95, 0.05, 0.6, ''),
        (PF, 'cam_fx', '카메라 초점거리', 300.0, 900.0, 5.0, 530.0, 'px'),
        (PF, 'cam_tilt_deg', '카메라 아래로 숙인 각도', -15.0, 15.0, 0.5, 0.0, '°'),
        (PF, 'cam_x_m', '카메라 앞쪽 위치 (몸 중심에서)', 0.0, 0.3, 0.005, 0.108, 'm'),
        (PF, 'cam_h_off_m', '카메라 높이 − 다리 높이', 0.0, 0.2, 0.005, 0.081, 'm'),
        (PF, 'cam_max_dist_m', '카메라 거리 최대 (넘으면 버림)', 1.0, 10.0, 0.5, 5.0, 'm'),
    ]),
    ('사람 인식', '카메라 (YOLO) · 라이다 다리 검출 — 로봇이 live_params 로 열면 실시간', [
        ('/person_detector', 'score_min', '사람 검출 최소 확신도', 0.1, 0.9, 0.05, 0.40, ''),
        ('/person_detector', 'process_hz', '사람 검출 주기 (추종 중)', 5.0, 30.0, 1.0, 30.0, 'Hz'),
        ('/person_detector', 'idle_hz', '사람 검출 주기 (추종 꺼짐)', 0.5, 10.0, 0.5, 2.0, 'Hz'),
        ('/leg_detector', 'max_range_m', '다리 검출 최대 거리', 1.0, 8.0, 0.25, 4.0, 'm'),
        ('/leg_detector', 'track_gate_m', '다리 추적 허용 이동', 0.1, 1.5, 0.05, 0.5, 'm'),
        ('/leg_detector', 'track_keep_s', '다리 놓쳐도 유지', 0.2, 3.0, 0.1, 1.0, 's'),
    ]),
    ('장애물 회피', '직진 통로가 막히면 Nav2 로 돌아간다 (켜기/끄기는 첫 탭)', [
        (PF, 'corridor_margin_m', '직진 통로 여유 폭', 0.0, 0.5, 0.01, 0.12, 'm'),
        (PF, 'nav_hold_s', '막힌 뒤 Nav2 유지 시간', 0.0, 10.0, 0.5, 2.0, 's'),
        (PF, 'nav_replan_s', 'Nav2 경로 다시 계산 주기', 0.1, 3.0, 0.1, 0.5, 's'),
        (LCM, 'inflation_layer.inflation_radius', '장애물에서 떨어질 거리 (팽창 반경)', 0.1, 1.0, 0.05, 0.40, 'm'),
    ]),
    ('SLAM (노트북)', '"SLAM 다시 시작" 때 적용 — 지도가 지워지니 저장 먼저', [
        ('/slam_toolbox', 'resolution', '지도 해상도 (칸 크기)', 0.02, 0.10, 0.01, 0.05, 'm'),
        ('/slam_toolbox', 'max_laser_range', '라이다 최대 사용 거리', 2.0, 12.0, 0.5, 8.0, 'm'),
        ('/slam_toolbox', 'minimum_travel_distance', '이만큼 움직이면 스캔 추가', 0.0, 0.5, 0.05, 0.10, 'm'),
        ('/slam_toolbox', 'minimum_travel_heading', '이만큼 돌면 스캔 추가', 0.0, 0.5, 0.05, 0.10, 'rad'),
        ('/slam_toolbox', 'minimum_time_interval', '스캔 최소 간격', 0.0, 1.0, 0.05, 0.2, 's'),
        ('/slam_toolbox', 'map_update_interval', '지도 그림 갱신 주기', 0.5, 10.0, 0.5, 2.0, 's'),
        ('/slam_toolbox', 'do_loop_closing', '루프 클로징 (한 바퀴 돌면 지도 맞추기)', 0, 0, 0, True, ''),
    ]),
    ('패드 · 손짓 (노트북)', '"다시 시작" 때 적용 (노드가 live_params 를 내놓으면 실시간)', [
        ('/pad_teleop', 'deadzone', '스틱 데드존', 0.0, 0.3, 0.01, 0.08, ''),
        ('/pad_teleop', 'height_rate', '몸 높이 바뀌는 속도 (RT/LT)', 0.01, 0.15, 0.01, 0.05, 'm/s'),
        ('/gesture', 'process_hz', '손짓 처리 주기', 1.0, 30.0, 1.0, 15.0, 'Hz'),
        ('/gesture', 'min_score', '손짓 최소 확신도', 0.3, 1.0, 0.05, 0.6, ''),
        ('/gesture', 'hold_s', '손짓을 이만큼 유지해야 인정', 0.1, 2.0, 0.1, 0.6, 's'),
    ]),
]
# 소스에서 실시간 반영이 확인된 것 (add_on_set_parameters_callback / Nav2 동적 파라미터)
KNOWN_LIVE = {
    PF: {'follow_dist_m', 'vx_max', 'wz_max', 'occluded_s', 'search_wz', 'appearance_min', 'use_nav', 'nav_replan_s',
         'reacquire_min', 'grace_s', 'legs_only_s', 'max_gate_m', 'predict_s', 'corridor_margin_m', 'nav_hold_s',
         'bearing_tol_deg', 'camera_only', 'cam_fx', 'cam_tilt_deg', 'cam_x_m', 'cam_h_off_m', 'cam_max_dist_m',
         'cam_confirm_s', 'cam_fov_deg', 'cam_only_score'},
    RPP: {'FollowPath.desired_linear_vel', 'FollowPath.rotate_to_heading_angular_vel',
          'FollowPath.regulated_linear_scaling_min_speed', 'FollowPath.min_approach_linear_velocity'},
    LCM: {'inflation_layer.inflation_radius'},
    '/cmd_mux': {'lidar_limit'},
}
# 읽기 전용 표시 (게인이라 GUI 가 바꾸지 않음) — 로봇 2026-10-15 요청 3
BAL_INFO = [('tune.jump_min_v', '점프 최소 속도', 'm/s'), ('tune.jump_max_v', '점프 최대 속도 (0 = 꺼짐)', 'm/s'),
            ('tune.accel_max', '가속 한계', 'm/s²'), ('tune.jerk_max', '저크 한계', 'm/s³'), ('th_trim_deg', '균형점 보정', '°'),
            ('tune.v_ki', '경사 보정 (속도 적분)', ''), ('tune.bal_adapt_x_max', '균형점 학습: 위치 오차 이내', 'm'),
            ('tune.bal_adapt_max_deg', '균형점 학습 한계', '°'), ('tune.x_turn_wz', '회전 중 위치 리셋 (0 = 유지)', 'rad/s')]
MARKERS = ['점프 평지', '점프 턱 5.6 cm']    # 시험 표시 버튼 (gcs/marker → 로봇 health_logger 'MARKER: …')
MIRROR = {LCM: [GCM]}                       # 로컬 코스트맵 값을 글로벌에도
RESTART = {'/pad_teleop': 'pad', '/gesture': 'gesture', '/slam_toolbox': 'slam'}   # 노트북에서 띄우는 노드
ASK_LIVE = ['/balance', '/pad_teleop', '/gesture', '/person_detector', '/leg_detector', PF]   # live_params 물어볼 노드
# 실시간 = 해제 (disarmed) 하고 변경 (사용자 2026-10-13): 균형·주행 중에는 어떤 값도 보내지 않는다


# ------------------------------------------------------------------ ROS (뒤 스레드)
def _pv(v):
    if isinstance(v, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))


def _from_pv(p):
    return {ParameterType.PARAMETER_BOOL: p.bool_value, ParameterType.PARAMETER_INTEGER: p.integer_value,
            ParameterType.PARAMETER_DOUBLE: p.double_value, ParameterType.PARAMETER_STRING: p.string_value,
            ParameterType.PARAMETER_STRING_ARRAY: list(p.string_array_value)}.get(p.type)


def _fmt(v):
    return str(v) if isinstance(v, bool) else f'{v:g}'


class Ros(QtCore.QObject):
    """rclpy 노드를 뒤 스레드에서 돌린다. 결과는 Qt 신호로 GUI 스레드에 넘긴다."""
    graph = QtCore.pyqtSignal(set)                  # 지금 보이는 노드 이름들
    params = QtCore.pyqtSignal(str, dict)            # 노드, {이름: 값 | None (없음)}
    done = QtCore.pyqtSignal(str, bool, str)         # 무엇, 성공, 메시지
    topic = QtCore.pyqtSignal(str, str)              # 토픽, 내용
    mode = QtCore.pyqtSignal(str)                    # balance mode (disarmed | stand | balance | fault | '' = 안 들어옴)
    panel = QtCore.pyqtSignal(dict)                  # 상태판 값 (status_panel.py)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = Node('gcs_gui')
        self.cli = {}
        self.lock = threading.Lock()
        for t in ('follow/status', 'cmd_mux/status', 'gesture/action'):
            self.node.create_subscription(String, t, lambda m, t=t: self.topic.emit(t, m.data), 10)
        # 점프 결과 — balance_node 는 거절을 로그 (/rosout) 로만 남긴다. pad_teleop 의 "ok: jump requested" 는 접수일 뿐
        self.node.create_subscription(Log, 'rosout', self._rosout, 100)
        # 턱 감지 모드 (로봇 2026-10-23): 바뀔 때만 발행 + latched → 늦게 붙어도 마지막 값을 받게 transient_local
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import Bool, Int8
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # latched 값은 구독하자마자 오는데 그때는 GUI 가 아직 신호에 안 붙었을 수 있다 → 저장해 두고 _emit_state (0.5 s) 가 다시 넘긴다
        self._step = {}
        self.node.create_subscription(Bool, 'balance/step_mode_active', lambda m: self._step.__setitem__('step_mode', str(int(m.data))), latched)
        self.node.create_subscription(Int8, 'balance/step_phase', lambda m: self._step.__setitem__('step_phase', str(m.data)), latched)
        try:   # 균형 상태 (100 Hz) — 최신 것만, 0.5 초마다 GUI 로
            from gen2_msgs.msg import ControllerState
            self._cs = None
            self.node.create_subscription(ControllerState, 'controller/state', self._state, qos_profile_sensor_data)
            self.node.create_timer(0.5, self._emit_state)
        except ImportError:
            pass
        # 상태판 (오른쪽) — 값마다 (받은 시각, 값), 0.2 초마다 GUI 로
        self._panel, self._v_last, self._acc = {}, None, 0.0
        from sensor_msgs.msg import BatteryState
        for k in ('motor', 'compute'):
            self.node.create_subscription(BatteryState, f'power/{k}', lambda m, k=k: self._power(k, m), 10)
        # 모터 온도·상태 = motor_monitor 의 /diagnostics (1 Hz). motors/state (100 Hz) 는 파이썬으로 받으면 한 코어의 14 % (2026-10-14 잼)
        from diagnostic_msgs.msg import DiagnosticArray
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self._diag, 10)
        # 라이다 끊김 감시: /scan 은 도착 시각만 필요 → raw (역직렬화 안 함)
        from sensor_msgs.msg import LaserScan
        self._scan_t = None
        self.node.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data, raw=True)
        self.marker_pub = self.node.create_publisher(String, 'gcs/marker', 10)
        self.node.create_timer(0.2, self._emit_panel)          # 진단은 1 Hz 라 3 초까지 유효 (_emit_panel)
        self.node.create_timer(2.0, self._graph)
        self._att, self._yaw, self._odom_sub, self._att_users, self.yaw0 = None, None, None, 0, None
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()
        self.closed = False

    def _spin(self):
        try:
            self.ex.spin()
        except Exception:   # 종료 때
            pass

    def _rosout(self, m):
        if m.name == 'balance' and (m.msg.startswith('jump: ') or m.msg.startswith('step-detect')):
            self.topic.emit('jump', m.msg)

    def att_on(self, on):
        """자세 그림이 보이는 동안만 /odom (요) 을 받는다. 롤·피치는 늘 받는 controller/state 에서."""
        from nav_msgs.msg import Odometry
        with self.lock:
            self._att_users = max(0, self._att_users + (1 if on else -1))     # 아래 줄 + 큰 창
            if self._att_users and self._odom_sub is None:
                # raw = 역직렬화 없이 바이트만 (파이썬 메시지 변환이 비싸다): 필요한 값만 꺼낸다
                self._odom_sub = self.node.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data, raw=True)
            elif not self._att_users and self._odom_sub is not None:
                self.node.destroy_subscription(self._odom_sub)
                self._odom_sub, self._yaw = None, None

    def _on_odom(self, b):
        """nav_msgs/Odometry CDR: 캡슐 4 B, stamp 8 B, frame_id·child_frame_id 문자열, pose.position 3d, orientation 4d,
        공분산 36d, twist.linear 3d, angular 3d. 평면 오도메트리라 요 = 2·atan2(qz, qw)."""
        try:
            le = '<' if b[1] == 1 else '>'
            o = 12
            for _ in range(2):
                o = (o - 4 + 3 & ~3) + 4
                n, = struct.unpack_from(le + 'I', b, o)
                o += 4 + n
            o = 4 + ((o - 4 + 7) & ~7)
            _, _, qz, qw = struct.unpack_from(le + '4d', b, o + 24)
            wz, = struct.unpack_from(le + 'd', b, o + 24 + 32 + 288 + 40)
            self._yaw = (time.monotonic(), math.degrees(2 * math.atan2(qz, qw)), wz)
        except struct.error:
            pass

    def attitude(self):
        """(t, 롤°, 피치°, 요° | None, wz rad/s | None) — 롤·피치 = controller/state (REP-103 부호), 요 = odom."""
        a = self._att
        if a is None:
            return None
        y = self._yaw if self._yaw and time.monotonic() - self._yaw[0] < 1.0 else None
        return (a[0], a[1], a[2], y[1] if y else None, y[2] if y else None)

    def _state(self, m):
        now = time.monotonic()
        self._cs = (now, m.mode, m.fault, m.pitch, m.v)
        if self._v_last is not None and now - self._v_last[0] > 1e-3:      # 가속도 = dv/dt, 0.2 초 평활
            dt = now - self._v_last[0]
            a = (m.v - self._v_last[1]) / dt
            self._acc = a if dt > 0.5 else self._acc + (a - self._acc) * min(1.0, dt / 0.2)
        self._v_last = (now, m.v)
        self._att = (now, -math.degrees(m.roll), math.degrees(m.pitch))     # roll: ControllerState + = 왼쪽 내려감 → REP-103
        # roll: ControllerState 는 + = 왼쪽 내려감 → REP-103 (자세 그림과 같게) + = 오른쪽 내려감 으로
        self._panel['cs'] = (now, dict(mode=m.mode, fault=m.fault, v=m.v, acc=self._acc, vx_cmd=m.vx_cmd, wz_cmd=m.wz_cmd,
                                       pitch=math.degrees(m.pitch), roll=-math.degrees(m.roll), h=tuple(m.h),
                                       imu_age_ms=m.imu_age_ms))

    def _power(self, k, m):
        self._panel[k] = (time.monotonic(), (m.voltage, m.percentage, m.present))

    def _diag(self, m):
        """motor_monitor 'motor: <이름>' 항목 (1 Hz): 최고 온도 + 문제 있는 모터 (WARN 이상)."""
        mot = {}
        for st in m.status:
            if 'motor: ' not in st.name:
                continue
            kv = {v.key: v.value for v in st.values}
            if kv.get('configured', '').lower() != 'true':
                continue
            lv = st.level if isinstance(st.level, int) else ord(st.level)     # byte → bytes 로 온다
            try:
                t = float(kv.get('temperature_c', 'nan'))
                if not float(kv.get('age_s', 'inf')) < 1.0:     # 응답 없는 드라이브의 온도는 0 으로 온다
                    t = math.nan
            except ValueError:
                t = math.nan
            mot[st.name.split('motor: ')[-1]] = (t, lv, st.message)
        if not mot:
            return
        temps = [(t, n) for n, (t, _, _) in mot.items() if not math.isnan(t)]
        bad = {}                                  # 같은 문제끼리 묶는다: "no feedback: wheel_l, wheel_r …"
        for n, (_, lv, msg) in mot.items():
            if lv >= 1:
                bad.setdefault((lv, msg.split(' (')[0]), []).append(n)
        hot = max(temps) if temps else (math.nan, '')
        level = max(lv for lv, _ in bad) if bad else 0
        text = ' · '.join(f'{msg}: {", ".join(sorted(ns))}' for (lv, msg), ns in sorted(bad.items(), reverse=True))
        self._panel['temp'] = (time.monotonic(), (hot[0], hot[1], level, text))

    def _on_scan(self, _):
        self._scan_t = time.monotonic()

    def marker(self, text):
        self.marker_pub.publish(String(data=text))

    def _emit_panel(self):
        now = time.monotonic()
        cs = self._panel.get('cs')
        if cs and now - cs[0] < 1.5:      # 로봇이 보일 때만 센서 끊김 판단
            scan_age = None if self._scan_t is None else now - self._scan_t
            self._panel['sensors'] = (now, (scan_age, cs[1]['imu_age_ms']))
        self.panel.emit({k: v for k, (t, v) in list(self._panel.items()) if now - t < (3.0 if k == 'temp' else 1.5)})

    def _emit_state(self):
        for k, v in list(self._step.items()):
            self.topic.emit(k, v)
        if self._cs:
            t, mode, fault, pitch, v = self._cs
            if time.monotonic() - t < 1.5:
                s = f'{mode}' + (f' ({fault})' if fault else '') + f' · 기울기 {pitch*57.3:+.1f}° · 속도 {v:+.2f} m/s'
                self.topic.emit('controller/state', s)
                self.mode.emit(mode)
                return
        self.topic.emit('controller/state', '')
        self.mode.emit('')

    def _graph(self):
        names = {(ns.rstrip('/') + '/' + n) for n, ns in self.node.get_node_names_and_namespaces()}
        self.graph.emit(names)

    def _client(self, typ, name):
        with self.lock:
            if name not in self.cli:
                self.cli[name] = self.node.create_client(typ, name)
            return self.cli[name]

    def _call(self, typ, name, req, what, on_ok=None):
        c = self._client(typ, name)
        if not c.service_is_ready():
            self.done.emit(what, False, f'{name} 없음 (노드가 꺼져 있거나 연결 안 됨)')
            return
        f = c.call_async(req)

        def cb(f):
            try:
                r = f.result()
            except Exception as e:  # noqa: BLE001
                self.done.emit(what, False, str(e))
                return
            if on_ok:
                on_ok(r)
            else:
                self.done.emit(what, getattr(r, 'success', True), getattr(r, 'message', ''))
        f.add_done_callback(cb)

    def get_params(self, node, names):
        names = list(names)
        req = GetParameters.Request(names=names)

        def ok(r):
            vals = {n: (None if v.type == ParameterType.PARAMETER_NOT_SET else _from_pv(v)) for n, v in zip(names, r.values)}
            vals.update({n: None for n in names[len(r.values):]})
            self.params.emit(node, vals)
        self._call(GetParameters, f'{node}/get_parameters', req, f'읽기 {node}', ok)

    def set_params(self, node, vals):
        req = SetParameters.Request(parameters=[Parameter(name=n, value=_pv(v)) for n, v in vals.items()])

        def ok(r):
            bad = [f'{n}: {x.reason}' for n, x in zip(vals, r.results) if not x.successful]
            self.done.emit(f'적용 {node}', not bad, '; '.join(bad) or ', '.join(f'{n}={_fmt(v)}' for n, v in vals.items()))
            good = {n: v for (n, v), x in zip(vals.items(), r.results) if x.successful}
            if good:
                self.params.emit(node, good)
        self._call(SetParameters, f'{node}/set_parameters', req, f'적용 {node}', ok)

    def set_bool(self, srv, v, what):
        self._call(SetBool, srv, SetBool.Request(data=bool(v)), what)

    def trigger(self, srv, what):
        self._call(Trigger, srv, Trigger.Request(), what)

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        try:
            rclpy.shutdown()                 # spin() 이 빠져나온다
            self.th.join(timeout=2.0)
            self.ex.shutdown(timeout_sec=0.5)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


# ------------------------------------------------------------------ 프로세스 (노트북 쪽)
def pgrep(pat):
    return subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True).stdout.split()


def running(pat):
    return bool(pgrep(pat))


def kill(pat, sig=signal.SIGINT):
    for p in pgrep(pat):
        try:
            os.kill(int(p), sig)
        except (ProcessLookupError, ValueError, PermissionError):
            pass


def launch(name, cmd):
    """터미널 없이 뒤에서 (세션 분리 — GUI 를 닫아도 계속). 로그 = ~/gen2_desk/logs/<name>.log"""
    os.makedirs(LOGS, exist_ok=True)
    log = open(f'{LOGS}/{name}.log', 'a')
    log.write(f'\n===== {time.strftime("%F %T")} {cmd}\n')
    log.flush()
    subprocess.Popen(['bash', '-c', f'{LENV}; exec {cmd}'], stdout=log, stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL, start_new_session=True)


def _scroll(w, horizontal=True):
    """창을 반쪽 화면 (960 px) 으로 스냅할 수 있게: 좁아지면 내용은 스크롤. horizontal=False 면 폭은 내용 그대로 (세로만)."""
    sa = QtWidgets.QScrollArea()
    sa.setWidgetResizable(True)
    sa.setFrameShape(QtWidgets.QFrame.NoFrame)
    sa.setWidget(w)
    if not horizontal:
        sa.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        sa.setFixedWidth((w.minimumWidth() or w.sizeHint().width()) + sa.verticalScrollBar().sizeHint().width())
    return sa


def ros_args(vals):
    return ' '.join(f'-p {k}:={str(v).lower() if isinstance(v, bool) else v}' for k, v in vals.items())


# 노트북 쪽 기능: 키 → (이름, 프로세스 패턴, 다른 컴퓨터에서 돌면 막을 ROS 노드)
LOCAL = {
    'pad':     ('패드 (joy + pad_teleop)', 'lib/gen2_tools/pad_teleop', '/pad_teleop'),
    'gesture': ('손짓 인식 (손바닥 정지 · 엄지 재개)', f'^{DESK}/venv_vision/bin/python -m gen2_tools.gesture_node', None),
    'slam':    ('SLAM 지도 (노트북)', '[a]sync_slam_toolbox_node', '/slam_toolbox'),
    'rviz':    ('RViz (지도·라이다·코스트맵·카메라)', '^rviz2 -d', None),
    'hud':     ('HUD (카메라 + 사람 상자)', f'^/usr/bin/python3 {DESK}/hud.py', None),
    'dash':    ('상태판 창 (desk_dash · 모터)', f'^/usr/bin/python3 {DESK}/desk_dash.py', None),   # = wifi -r (exec 로 바뀜)
}


# ------------------------------------------------------------------ 위젯
class Lamp(QtWidgets.QLabel):
    COLORS = {'on': '#2ecc71', 'off': '#7f8c8d', 'warn': '#f39c12', 'err': '#e74c3c'}

    def __init__(self):
        super().__init__()
        self.setFixedSize(16, 16)
        self.set('off')

    def set(self, s):
        self.setStyleSheet(f'background:{self.COLORS[s]}; border-radius:8px;')


class ParamRow(QtWidgets.QWidget):
    """한 파라미터: 슬라이더 + 숫자 (bool 이면 체크박스) + 적용 시점 표시 + 로봇의 지금 값."""
    changed = QtCore.pyqtSignal(object)
    TAGS = {'live': ('실시간', '#27ae60'), 'restart': ('재시작 때', '#2980b9'),
            'locked': ('잠김', '#999'), 'absent': ('노드 꺼짐', '#999')}

    def __init__(self, node, name, label, lo, hi, step, default, unit):
        super().__init__()
        self.node, self.name, self.default = node, name, default
        self.lo, self.step = lo, step
        self.is_bool = isinstance(default, bool)
        self.robot, self.touched, self.kind = None, False, 'absent'
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(0, 1, 0, 1)
        lab = QtWidgets.QLabel(label)
        lab.setToolTip(f'{node}  {name}  (기본 {_fmt(default)})')
        lab.setMinimumWidth(250)
        h.addWidget(lab)
        if self.is_bool:
            self.box = QtWidgets.QCheckBox('켜기')
            self.box.toggled.connect(self._user)
            h.addWidget(self.box, 1)
        else:
            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, int(round((hi - lo) / step)))
            self.spin = QtWidgets.QDoubleSpinBox()
            self.spin.setDecimals(len(f'{step:g}'.split('.')[1]) if '.' in f'{step:g}' else 0)
            self.spin.setRange(lo, hi)
            self.spin.setSingleStep(step)
            self.spin.setSuffix(f' {unit}' if unit else '')
            self.spin.setFixedWidth(120)
            self.slider.valueChanged.connect(lambda i: self.spin.setValue(self.lo + i * self.step))
            self.spin.valueChanged.connect(self._user)
            h.addWidget(self.slider, 1)
            h.addWidget(self.spin)
        self.tag = QtWidgets.QLabel()
        self.tag.setFixedWidth(70)
        self.tag.setAlignment(QtCore.Qt.AlignCenter)
        self.cur = QtWidgets.QLabel('지금: —')
        self.cur.setFixedWidth(95)
        h.addWidget(self.tag)
        h.addWidget(self.cur)
        self.set_value(default)
        self.set_kind('absent', '')

    def _sync_slider(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round((v - self.lo) / self.step)))
        self.slider.blockSignals(False)

    def _user(self, v):
        if not self.is_bool:
            self._sync_slider(v)
        self.touched = True
        self._mark()
        self.changed.emit(self)

    def value(self):
        return self.box.isChecked() if self.is_bool else self.spin.value()

    def set_value(self, v):
        if self.is_bool:
            self.box.blockSignals(True)
            self.box.setChecked(bool(v))
            self.box.blockSignals(False)
        else:
            self.spin.blockSignals(True)
            self.spin.setValue(float(v))
            self.spin.blockSignals(False)
            self._sync_slider(float(v))
        self._mark()

    def set_robot(self, v):
        if v is None or isinstance(v, (list, str)):
            return
        self.robot = bool(v) if self.is_bool else float(v)
        if not self.touched:            # 처음엔 지금 값을 보여 준다
            self.set_value(self.robot)
        self.cur.setText(f'지금: {_fmt(self.robot)}')
        self._mark()

    def set_kind(self, kind, reason):
        self.kind = kind
        text, color = self.TAGS[kind]
        self.tag.setText(text)
        self.tag.setStyleSheet(f'color:white; background:{color}; border-radius:3px; padding:1px')
        self.tag.setToolTip(reason)
        self.setEnabled(kind in ('live', 'restart', 'absent'))

    def differs(self):
        return self.robot is not None and (self.value() != self.robot if self.is_bool
                                           else abs(self.value() - self.robot) > 1e-6)

    def _mark(self):
        if self.robot is None:
            self.cur.setStyleSheet('color:#888')
        elif self.differs():
            self.cur.setStyleSheet('color:#e67e22; font-weight:bold')     # 아직 적용 안 됨
        else:
            self.cur.setStyleSheet('color:#27ae60')


class Group(QtWidgets.QGroupBox):
    def __init__(self, title, note, specs, gui):
        super().__init__(title)
        self.gui = gui
        self.v = QtWidgets.QVBoxLayout(self)
        self.note = QtWidgets.QLabel(note)
        self.note.setStyleSheet('color:#888')
        self.note.setWordWrap(True)
        self.v.addWidget(self.note)
        self.rows = []
        for spec in specs:
            r = ParamRow(*spec)
            r.changed.connect(gui.on_row_changed)
            self.rows.append(r)
            self.v.addWidget(r)

    def add_buttons(self, *buttons):
        h = QtWidgets.QHBoxLayout()
        for b in buttons:
            h.addWidget(b)
        h.addStretch()
        self.v.addLayout(h)


# ------------------------------------------------------------------ 창
class Gui(QtWidgets.QMainWindow):
    sshres = QtCore.pyqtSignal(str, int, str)    # 무엇, 종료 코드, 출력

    def __init__(self):
        super().__init__()
        self.setWindowTitle('C-WANG 관제')
        self.resize(1340, 960)        # 최소 ≈ 870×430 → 화면 반쪽 (960) 스냅 가능
        self.ros = Ros()
        QtWidgets.QApplication.instance().aboutToQuit.connect(self.ros.shutdown)
        self.pool = ThreadPoolExecutor(4)
        self.nodes = set()
        self.robot_info = {}
        self.live_params = {}          # 노드 → live_params 에 적힌 이름들
        self.bal_mode = ''
        self.pending = {}              # 노드 → {이름: 값}
        self.att_win = None            # 자세 창
        self.step_on, self.step_phase = False, 0     # 턱 감지 모드 (balance/step_mode_active, step_phase)

        w = QtWidgets.QWidget()
        self.setCentralWidget(w)
        top = QtWidgets.QVBoxLayout(w)
        top.addWidget(self._status_bar())
        mid = QtWidgets.QHBoxLayout()
        top.addLayout(mid, 1)
        left = QtWidgets.QSplitter(QtCore.Qt.Vertical)     # 위: 탭, 아래: 자세 (경계를 끌어 크기 조절)
        left.setChildrenCollapsible(False)
        mid.addWidget(left, 1)
        self.tabs = QtWidgets.QTabWidget()
        left.addWidget(self.tabs)
        self.att = AttitudeView(self.ros, compact=True)
        big = QtWidgets.QPushButton('크게 보기')
        big.setToolTip('자세를 따로 뜨는 큰 창으로')
        big.clicked.connect(self.show_attitude)
        self.att.buttons.addWidget(big)
        left.addWidget(self.att)
        left.setStretchFactor(0, 3)
        left.setStretchFactor(1, 1)
        self.spanel = StatusPanel()
        mid.addWidget(_scroll(self.spanel, horizontal=False))
        self.tabs.addTab(self._features_tab(), '기능 켜기/끄기')
        self.tabs.addTab(_scroll(self._tune_tab()), '파라미터 튜닝')
        self.tabs.addTab(self._log_tab(), '로그')

        self.ros.graph.connect(self._on_graph)
        self.ros.params.connect(self._on_params)
        self.ros.done.connect(self._on_done)
        self.ros.topic.connect(self._on_topic)
        self.ros.mode.connect(self._on_mode)
        self.ros.panel.connect(self.spanel.update_values)
        self.sshres.connect(self._on_ssh)

        self.send_timer = QtCore.QTimer(self, singleShot=True, interval=300)
        self.send_timer.timeout.connect(self.flush)
        self.poll = QtCore.QTimer(self, interval=1000)
        self.poll.timeout.connect(self._poll_local)
        self.poll.start()
        self.rpoll = QtCore.QTimer(self, interval=5000)
        self.rpoll.timeout.connect(self._poll_robot)
        self.rpoll.start()
        self._poll_robot()
        QtCore.QTimer.singleShot(2500, self.read_all)

    # ---------------- 위: 상태 한 줄
    def _status_bar(self):
        f = QtWidgets.QFrame()
        f.setFrameShape(QtWidgets.QFrame.StyledPanel)
        g = QtWidgets.QGridLayout(f)
        self.l_robot, self.t_robot = Lamp(), QtWidgets.QLabel('로봇: 확인 중…')
        self.l_ros, self.t_ros = Lamp(), QtWidgets.QLabel('균형: —')
        self.t_follow = QtWidgets.QLabel('추종: —')
        self.t_mux = QtWidgets.QLabel('명령: —')
        for t in (self.t_follow, self.t_mux):
            t.setStyleSheet('color:#555')
        for t in (self.t_robot, self.t_ros, self.t_follow, self.t_mux):   # 글이 길어져도 창 폭을 안 늘린다 (반쪽 화면 스냅)
            t.setWordWrap(True)
            t.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        g.addWidget(self.l_robot, 0, 0)
        g.addWidget(self.t_robot, 0, 1)
        g.addWidget(self.l_ros, 0, 2)
        g.addWidget(self.t_ros, 0, 3)
        g.addWidget(self.t_follow, 1, 1)
        g.addWidget(self.t_mux, 1, 3)
        # 턱 감지 모드 배지 (켜졌을 때만) + 점프 순간 배지 (2 초)
        self.b_step = QtWidgets.QLabel(' STEP ')
        self.b_step.setStyleSheet('background:#111; color:#fff; font-weight:bold; font-size:15pt; padding:2px 8px; border-radius:3px')
        self.b_step.setToolTip('턱 감지 모드 (평지 전용) — 끄기: 패드 십자키 ↓ 또는 기능 탭')
        self.b_step.hide()
        self.b_jump = QtWidgets.QLabel(' JUMP ')
        self.b_jump.setStyleSheet('background:#e67e22; color:#fff; font-weight:bold; font-size:15pt; padding:2px 8px; border-radius:3px')
        self.b_jump.hide()
        bb = QtWidgets.QHBoxLayout()
        bb.addWidget(self.b_step)
        bb.addWidget(self.b_jump)
        bb.addStretch()
        g.addLayout(bb, 2, 1, 1, 3)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(3, 1)
        sit = QtWidgets.QPushButton('로봇 앉기')
        sit.setToolTip('balance/sit — 천천히 앉아서 해제 (패드 BACK 과 같음)')
        sit.setStyleSheet('background:#c0392b; color:white; font-weight:bold; padding:8px 14px')
        sit.clicked.connect(lambda: self.ros.trigger('/balance/sit', '로봇 앉기'))
        stop = QtWidgets.QPushButton('추종 정지')
        stop.setStyleSheet('background:#e67e22; color:white; font-weight:bold; padding:8px 14px')
        stop.clicked.connect(self.follow_off)
        g.addWidget(stop, 0, 4, 2, 1)
        g.addWidget(sit, 0, 5, 2, 1)
        return f

    def show_attitude(self):
        if self.att_win is None:
            self.att_win = AttitudeWindow(self.ros)
        self.att_win.show()
        self.att_win.raise_()
        self.att_win.activateWindow()

    # ---------------- 기능 탭
    def _row(self, grid, r, title, lamp, buttons, note=''):
        grid.addWidget(lamp, r, 0)
        t = QtWidgets.QLabel(title)
        t.setStyleSheet('font-weight:bold')
        grid.addWidget(t, r, 1)
        st = QtWidgets.QLabel(note)
        st.setStyleSheet('color:#888')
        st.setWordWrap(True)                       # 좁은 창 (반쪽 화면) 에서 버튼이 안 밀려나게
        st.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        grid.addWidget(st, r, 2)
        h = QtWidgets.QHBoxLayout()
        for b in buttons:
            h.addWidget(b)
        h.addStretch()
        grid.addLayout(h, r, 3)
        return st

    def _btn(self, text, fn, tip=''):
        b = QtWidgets.QPushButton(text)
        b.clicked.connect(fn)
        if tip:
            b.setToolTip(tip)
        return b

    def _features_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        h = QtWidgets.QHBoxLayout()
        h.addWidget(self._btn('▶ 한 번에 켜기 (cw up)', self.all_up,
                              '추종 스택 → 패드 → 손짓 → RViz → 상태판 → HUD'))
        h.addWidget(self._btn('■ 노트북 쪽 모두 끄기 (cw down)', self.all_down,
                              '패드·손짓·RViz·HUD·상태판 (로봇·SLAM 은 그대로 — 패드가 끊기면 로봇은 스스로 멈춤·앉기)'))
        h.addStretch()
        v.addLayout(h)

        box = QtWidgets.QGroupBox('로봇 (젯슨)')
        g = QtWidgets.QGridLayout(box)
        g.setColumnStretch(2, 1)
        self.lamp, self.note = {}, {}
        rows = [
            ('stack', '사람 추종 스택', [self._btn('켜기', lambda: self.robot_cmd('stack_on')),
                                    self._btn('끄기', lambda: self.robot_cmd('stack_off'))]),
            ('follow', '추종 활성화 (= 패드 X)', [self._btn('켜기 (앞사람 등록)', lambda: self.ros.set_bool(
                '/follow/enable', True, '추종 켜기')), self._btn('끄기', self.follow_off)]),
            ('hold', '일시 정지 (= 손바닥)', [self._btn('정지', lambda: self.ros.set_bool('/follow/hold', True, '일시 정지')),
                                        self._btn('재개', lambda: self.ros.set_bool('/follow/hold', False, '재개'))]),
            ('nav', '장애물 우회 (Nav2)', [self._btn('켜기', lambda: self.ros.set_params(PF, {'use_nav': True})),
                                        self._btn('끄기', lambda: self.ros.set_params(PF, {'use_nav': False}))]),
            ('camonly', '카메라만으로 추적 (라이다 다리 없을 때)', [
                self._btn('켜기', lambda: self.ros.set_params(PF, {'camera_only': True}), '켜기/끄기는 균형 중에도 됨'),
                self._btn('끄기', lambda: self.ros.set_params(PF, {'camera_only': False}))]),
            ('jump', '점프 (패드 Y) — 읽기 전용', []),
            ('step', '턱 감지 모드 (평지 전용, 패드 십자키 ↑/↓)', [
                self._btn('켜기', lambda: self.ros.set_bool('/balance/step_mode', True, '턱 감지 모드 켜기'),
                          '앞으로 밀면: 0.14 m/s 접근 → 턱에 막히면 0.3 m 후진 → 0.3 m/s 재주행 → 자동 점프. 스틱 놓으면 취소'),
                self._btn('끄기', lambda: self.ros.set_bool('/balance/step_mode', False, '턱 감지 모드 끄기'))]),
            ('lidar', '장애물 앞 정지·감속 (라이다)', [
                self._btn('켜기', lambda: self.ros.set_params('/cmd_mux', {'lidar_limit': True})),
                self._btn('끄기', self.lidar_off, '로봇 기본 = 꺼짐. 켜면 자기 다리·바퀴를 장애물로 잡아 못 갈 수 있다 (2026-10-09)')]),
        ]
        for i, (k, t, b) in enumerate(rows):
            self.lamp[k] = Lamp()
            self.note[k] = self._row(g, i, t, self.lamp[k], b)
        v.addWidget(box)

        box = QtWidgets.QGroupBox('시험 표시 (시험 직전에 — 로봇 로그에 MARKER 로 남음)')
        h = QtWidgets.QHBoxLayout(box)
        for m in MARKERS:
            h.addWidget(self._btn(m, lambda _=0, m=m: self.send_marker(m)))
        h.addWidget(self._btn('자유 메모…', self.marker_note))
        h.addStretch()
        v.addWidget(box)

        box = QtWidgets.QGroupBox('이 노트북')
        g = QtWidgets.QGridLayout(box)
        g.setColumnStretch(2, 1)
        for i, k in enumerate(LOCAL):
            btns = [self._btn('켜기', lambda _=0, k=k: self.local_on(k)),
                    self._btn('끄기', lambda _=0, k=k: self.local_off(k))]
            if k == 'slam':
                btns += [self._btn('지도 지우기', self.slam_reset, '지금 자리부터 새로'),
                         self._btn('지도 저장', self.slam_save, '~/maps/이름.pgm/.yaml')]
            if k in ('pad', 'gesture'):
                btns.append(self._btn('다시 시작 (튜닝값 적용)', lambda _=0, k=k: self.local_restart(k)))
            self.lamp[k] = Lamp()
            self.note[k] = self._row(g, i, LOCAL[k][0], self.lamp[k], btns)
        v.addWidget(box)
        hint = QtWidgets.QLabel('패드: START 일어서기 · BACK 앉기 · LB+RB 비상 해제 · X 추종 · Y 점프 (위 점프 줄이 켜짐일 때) · A 누르는 동안 정지.  '
                                '데스크톱과 노트북에서 패드·SLAM 을 동시에 켜지 말 것 (이 창이 막아 줌).')
        hint.setWordWrap(True)
        hint.setStyleSheet('color:#888')
        v.addWidget(hint)
        v.addStretch()
        return _scroll(w)

    def send_marker(self, text):
        self.ros.marker(text)
        self.log(f'시험 표시 보냄: {text}')

    def marker_note(self):
        text, ok = QtWidgets.QInputDialog.getText(self, '시험 표시', '로봇 로그에 남길 메모:')
        if ok and text.strip():
            self.send_marker(text.strip())

    # ---------------- 튜닝 탭
    def _tune_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        h = QtWidgets.QHBoxLayout()
        self.auto = QtWidgets.QCheckBox('바꾸면 바로 적용')
        self.auto.setChecked(True)
        h.addWidget(self.auto)
        h.addWidget(self._btn('지금 값 모두 적용', self.apply_all))
        h.addWidget(self._btn('다시 읽기', self.read_all))
        h.addWidget(self._btn('기본값으로', self.defaults_all))
        h.addStretch()
        h.addWidget(self._btn('프리셋 저장', self.save_preset))
        h.addWidget(self._btn('프리셋 불러오기', self.load_preset))
        h.addWidget(self._btn('바뀐 값 복사 (comms 용)', self.copy_diff, '로봇 설정에 영구 반영하려면 comms 로 요청'))
        v.addLayout(h)
        lg = QtWidgets.QLabel('게인은 여기 없다 (최적화된 값). <b>실시간</b> = 로봇을 해제 (disarmed) 하고 바꾸면 바로 반영 '
                              '(서 있거나 주행 중엔 잠김) · <b>재시작 때</b> = 노트북에서 다시 켤 때 · '
                              '<b>잠김</b> = 노드가 실시간 변경을 아직 안 받음. 지금 값이 주황이면 아직 반영 안 됨. '
                              '로봇 노드를 다시 켜면 설정 파일 값으로 돌아간다 (영구 반영은 "바뀐 값 복사" → comms).')
        lg.setStyleSheet('color:#888')
        lg.setWordWrap(True)
        v.addWidget(lg)
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        inner = QtWidgets.QWidget()
        iv = QtWidgets.QVBoxLayout(inner)
        bi = QtWidgets.QGroupBox('균형 기준값 (읽기 전용 — 게인이라 여기서 안 바꿈, 바꾸려면 comms)')
        bl = QtWidgets.QVBoxLayout(bi)
        self.bal_info = QtWidgets.QLabel('균형 노드 안 보임')
        self.bal_info.setWordWrap(True)
        self.bal_info.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        bl.addWidget(self.bal_info)
        iv.addWidget(bi)
        self.groups = [Group(t, n, s, self) for t, n, s in GROUPS]
        self.rows = [r for g in self.groups for r in g.rows]
        gmap = {g.title(): g for g in self.groups}
        gmap['SLAM (노트북)'].add_buttons(self._btn('SLAM 다시 시작 (지도 지워짐)', self.slam_restart),
                                       self._btn('지도 저장', self.slam_save))
        gmap['패드 · 손짓 (노트북)'].add_buttons(self._btn('패드 다시 시작', lambda: self.local_restart('pad')),
                                           self._btn('손짓 다시 시작', lambda: self.local_restart('gesture')))
        for g in self.groups:
            iv.addWidget(g)
        iv.addStretch()
        sa.setWidget(inner)
        v.addWidget(sa, 1)
        self.update_kinds()
        return w

    # ---------------- 로그 탭
    def _log_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        h = QtWidgets.QHBoxLayout()
        self.logsel = QtWidgets.QComboBox()
        self.logsel.addItem('이 창의 기록')
        h.addWidget(self.logsel, 1)
        h.addWidget(self._btn('새로고침', self.show_log))
        v.addLayout(h)
        self.logview = QtWidgets.QPlainTextEdit(readOnly=True)
        self.logview.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.logview.setMaximumBlockCount(3000)
        v.addWidget(self.logview, 1)
        self.events = []
        self.logsel.currentIndexChanged.connect(self.show_log)
        return w

    def log(self, s, err=False):
        line = f'{time.strftime("%H:%M:%S")} {"⚠ " if err else ""}{s}'
        self.events.append(line)
        if self.logsel.currentIndex() == 0:
            self.logview.appendPlainText(line)
        self.statusBar().showMessage(line, 8000)

    def show_log(self):
        files = sorted(glob.glob(f'{LOGS}/*.log'))
        cur = self.logsel.currentText()
        self.logsel.blockSignals(True)
        self.logsel.clear()
        self.logsel.addItem('이 창의 기록')
        self.logsel.addItems([os.path.basename(f) for f in files])
        self.logsel.setCurrentIndex(max(self.logsel.findText(cur), 0))
        self.logsel.blockSignals(False)
        if self.logsel.currentIndex() == 0:
            self.logview.setPlainText('\n'.join(self.events))
        else:
            try:
                with open(f'{LOGS}/{self.logsel.currentText()}', errors='replace') as f:
                    self.logview.setPlainText(''.join(f.readlines()[-400:]))
            except OSError as e:
                self.logview.setPlainText(str(e))
        self.logview.moveCursor(QtGui.QTextCursor.End)

    # ---------------- 로봇 (ssh)
    def ssh(self, what, cmd):
        def job():
            try:
                r = subprocess.run(SSH + [ROBOT, cmd], capture_output=True, text=True, timeout=20)
                self.sshres.emit(what, r.returncode, (r.stdout + r.stderr).strip())
            except subprocess.TimeoutExpired:
                self.sshres.emit(what, 255, '시간 초과')
        self.pool.submit(job)

    def robot_cmd(self, k):
        if k == 'stack_on':
            if PF in self.nodes:
                self.log('사람 추종 스택 이미 켜짐')
                return
            self.ssh('추종 스택 켜기', 'setsid nohup ~/gen2_ws/logs/run_follow.sh > ~/gen2_ws/logs/run_follow.out 2>&1 '
                     '< /dev/null & echo started')
            self.log('추종 스택 켜는 중 — 10–20 초 뒤 "추종 활성화" 또는 패드 X')
        elif k == 'stack_off':
            self.ssh('추종 스택 끄기', '~/gen2_ws/logs/stop_follow.sh')
            if running(LOCAL['hud'][1]):
                self.local_off('hud')

    def follow_off(self):
        """추종 끄기 (= 패드 X 끄기) + 추종 HUD 닫기"""
        self.ros.set_bool('/follow/enable', False, '추종 끄기')
        if running(LOCAL['hud'][1]):
            self.local_off('hud')

    def _poll_robot(self):
        self.ssh('_poll', 'printf "%s %s %s\\n" $(systemctl is-active gen2-bench) $(systemctl is-active gen2-balance) '
                 '$(pgrep -f "[s]lam_toolbox" >/dev/null && echo slam || echo noslam)')

    def _on_ssh(self, what, code, out):
        if what == '_poll':
            if code == 0:
                p = (out.split() + ['?'] * 3)[:3]
                self.robot_info = {'bench': p[0], 'balance': p[1], 'slam': p[2] == 'slam'}
                self.l_robot.set('on' if p[0] == 'active' and p[1] == 'active' else 'warn')
                self.t_robot.setText(f'로봇 연결됨 · bench {p[0]} · balance {p[1]}'
                                     + (' · ⚠ 로봇에서 SLAM 이 돎' if self.robot_info['slam'] else ''))
            else:
                self.l_robot.set('err')
                self.t_robot.setText('로봇 연결 안 됨 (젯슨 전원 · Wi-Fi ROBOT · ssh 키 확인)')
            return
        self.log(f'{what}: {"OK" if code == 0 else "실패"} {out[-300:]}', err=code != 0)

    # ---------------- 노트북 프로세스
    def _other_pc(self, k):
        node = LOCAL[k][2]
        return node and node in self.nodes and not running(LOCAL[k][1])

    def local_values(self, node):
        return {r.name: r.value() for r in self.rows if r.node == node}

    def write_slam_yaml(self):
        try:
            with open(SLAM_BASE) as f:
                cfg = yaml.safe_load(f)
        except OSError as e:
            self.log(f'SLAM 설정 원본을 못 읽음: {e}', err=True)
            return False
        cfg['slam_toolbox']['ros__parameters'].update(self.local_values('/slam_toolbox'))
        with open(SLAM_YAML, 'w') as f:
            yaml.safe_dump(cfg, f)
        return True

    def local_on(self, k):
        name, pat, _ = LOCAL[k]
        if running(pat):
            self.log(f'{name}: 이미 켜짐')
            return
        if self._other_pc(k):
            self.log(f'{name}: 다른 컴퓨터에서 이미 돌고 있음 — 그쪽을 먼저 끌 것', err=True)
            QtWidgets.QMessageBox.warning(self, '동시 관제', f'{name} 이(가) 다른 컴퓨터 (데스크톱?) 에서 돌고 있습니다.\n'
                                                       '두 대에서 켜면 heartbeat·지도가 충돌합니다. 그쪽을 먼저 끄세요.')
            return
        if k == 'pad':
            if not running('lib/joy/joy_node'):
                launch('joy_node', 'ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0')
            launch('pad_teleop', f'ros2 run gen2_tools pad_teleop --ros-args {ros_args(self.local_values("/pad_teleop"))}')
            if not glob.glob('/dev/input/js*'):
                self.log('게임패드가 안 보임 (블루투스 연결 확인)', err=True)
        elif k == 'gesture':
            launch('gesture', f'{DESK}/venv_vision/bin/python -m gen2_tools.gesture_node --ros-args '
                              f'{ros_args(self.local_values("/gesture"))}')
        elif k == 'slam':
            if self.robot_info.get('slam'):
                self.log('로봇에서 SLAM 이 돌고 있음 — 둘이면 지도·TF 가 싸운다', err=True)
                return
            if not self.write_slam_yaml():
                return
            launch('slam', f'ros2 launch {DESK}/slam_gui.launch.py params:={SLAM_YAML}')
        elif k == 'rviz':
            launch('rviz', f'rviz2 -d {DESK}/gen2_desktop.rviz')
        elif k == 'hud':
            launch('hud', f'/usr/bin/python3 {DESK}/hud.py')
        elif k == 'dash':   # 터미널 화면 프로그램이라 창 하나 (wifi -r 과 같은 것). 오류로 끝나면 창을 남긴다
            subprocess.Popen(['gnome-terminal', '--title=상태판', '--', 'bash', '-c',
                              f'{LENV}; /usr/bin/python3 {DESK}/desk_dash.py; rc=$?; '   # 오류로 끝나면 창을 남긴다 (Ctrl+C·GUI 끄기 제외)
                              f'case $rc in 0|130|143) ;; *) echo; echo "[오류 $rc] Enter 로 닫기"; read ;; esac'],
                             start_new_session=True, stderr=subprocess.DEVNULL)
        self.log(f'{name}: 켬')

    def local_off(self, k):
        name, pat, _ = LOCAL[k]
        if k == 'pad':
            kill('lib/gen2_tools/pad_teleop')
            kill('lib/joy/joy_node')
        elif k == 'slam':
            kill('[a]sync_slam_toolbox_node|[g]en2_bringup slam.launch|[s]lam_gui.launch')
        elif k == 'hud':   # SIGINT 로 안 끝나면 (창 이벤트 대기 중) 2 초 뒤 SIGTERM
            kill(pat)
            QtCore.QTimer.singleShot(2000, lambda: kill(pat, signal.SIGTERM))
        else:
            kill(pat)
        self.log(f'{name}: 끔')

    def local_restart(self, k):
        if k == 'pad' and QtWidgets.QMessageBox.question(
                self, '패드 다시 시작', '패드를 다시 시작하는 동안 (1–2 초) heartbeat 가 끊깁니다.\n'
                                    '로봇이 서 있으면 멈추고 앉습니다. 계속할까요?') != QtWidgets.QMessageBox.Yes:
            return
        if k == 'pad':
            kill('lib/gen2_tools/pad_teleop')
        else:
            kill(LOCAL[k][1])
        QtCore.QTimer.singleShot(1500, lambda: self.local_on(k))

    def slam_restart(self):
        if running(LOCAL['slam'][1]) and QtWidgets.QMessageBox.question(
                self, 'SLAM 다시 시작', '지금 지도가 지워집니다 (먼저 "지도 저장"). 다시 시작할까요?') != QtWidgets.QMessageBox.Yes:
            return
        self.local_off('slam')
        QtCore.QTimer.singleShot(2500, lambda: self.local_on('slam'))

    def slam_reset(self):
        self.pool.submit(lambda: self._svc_cli("ros2 service call /slam_toolbox/reset slam_toolbox/srv/Reset '{}'",
                                               'SLAM 지도 지우기'))

    def slam_save(self):
        n, ok = QtWidgets.QInputDialog.getText(self, '지도 저장', '이름 (~/maps/이름):',
                                               text=time.strftime('room_%m%d_%H%M'))
        if not ok or not n.strip():
            return
        os.makedirs(f'{HOME}/maps', exist_ok=True)
        self.pool.submit(lambda: self._svc_cli(
            f"ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{{name: {{data: '{HOME}/maps/{n.strip()}'}}}}\"",
            f'지도 저장 ~/maps/{n.strip()}'))

    def _svc_cli(self, cmd, what):
        r = subprocess.run(['bash', '-c', f'{LENV}; timeout 10 {cmd}'], capture_output=True, text=True)
        self.sshres.emit(what, r.returncode, (r.stdout.strip().splitlines() or [r.stderr.strip()])[-1])

    def lidar_off(self):
        if QtWidgets.QMessageBox.question(self, '라이다 정지 끄기', '끄면 앞에 장애물이 있어도 멈추거나 느려지지 않습니다.\n'
                                          '끌까요?') == QtWidgets.QMessageBox.Yes:
            self.ros.set_params('/cmd_mux', {'lidar_limit': False})

    def all_up(self):
        if PF not in self.nodes:
            self.robot_cmd('stack_on')
        for k in ('pad', 'gesture', 'rviz', 'hud', 'dash'):
            if not running(LOCAL[k][1]):
                self.local_on(k)
        self.log('준비 — 패드 START 로 일어서기·균형, X 로 사람 추종 (지도는 SLAM 켜기)')

    def all_down(self):
        for k in ('pad', 'gesture', 'rviz', 'hud', 'dash'):
            self.local_off(k)
        self.log('노트북 쪽 끔 (SLAM·로봇은 그대로)')

    # ---------------- 상태 갱신
    def _poll_local(self):
        for k, (_, pat, node) in LOCAL.items():
            on = running(pat)
            other = self._other_pc(k)
            if k == 'pad':
                joy = running('lib/joy/joy_node')
                pad = bool(glob.glob('/dev/input/js*'))
                self.lamp[k].set('on' if on and joy and pad else 'warn' if on or joy else 'err' if other else 'off')
                self.note[k].setText(('켜짐' if on else '꺼짐') + (' · 게임패드 연결됨' if pad else ' · 게임패드 없음')
                                     + (' · ⚠ 다른 컴퓨터에서 패드가 돎' if other else ''))
            else:
                self.lamp[k].set('on' if on else 'err' if other else 'off')
                self.note[k].setText('켜짐' if on else '⚠ 다른 컴퓨터에서 돎' if other else '꺼짐')

    def _on_graph(self, names):
        appeared = {n for n in names - self.nodes}
        self.nodes = names
        stack = PF in names
        self.lamp['stack'].set('on' if stack else 'off')
        parts = [n for n, k in (('사람검출', '/person_detector'), ('다리', '/leg_detector'),
                                ('추종', PF), ('Nav2', RPP)) if k in names]
        self.note['stack'].setText(' · '.join(parts) if parts else '꺼짐')
        if not any('balance' in n for n in names):
            self.l_ros.set('off')
        if stack:
            self.ros.get_params(PF, ['use_nav', 'camera_only'])
        else:
            for k in ('follow', 'hold', 'nav', 'camonly'):
                self.lamp[k].set('off')
                self.note[k].setText('추종 스택 꺼짐')
        if '/balance' in names:
            self.ros.get_params('/balance', [n for n, _, _ in BAL_INFO])
        else:
            self.lamp['jump'].set('off')
            self.note['jump'].setText('균형 노드 안 보임')
            self.lamp['step'].set('off')
            self.note['step'].setText('balance_node 없음')
            self.b_step.hide()
        if '/cmd_mux' in names:
            self.ros.get_params('/cmd_mux', ['lidar_limit'])
        else:
            self.lamp['lidar'].set('off')
            self.note['lidar'].setText('cmd_mux 안 보임')
        for nd in ASK_LIVE:
            if nd in names:
                self.ros.get_params(nd, ['live_params'])
            else:
                self.live_params.pop(nd, None)
        if appeared & {r.node for r in self.rows}:   # 새로 뜬 노드: 지금 값 읽기
            self.read_all(only=appeared)
        self.update_kinds()
        if self.logsel.currentIndex():
            self.show_log()

    def _on_params(self, node, vals):
        if node in ASK_LIVE and 'live_params' in vals:
            lp = vals['live_params']
            live = set(lp) if isinstance(lp, list) else set()
            if live != self.live_params.get(node):
                self.live_params[node] = live
                self.update_kinds()
        if node == PF and vals.get('use_nav') is not None:
            self.lamp['nav'].set('on' if vals['use_nav'] else 'warn')
            self.note['nav'].setText('켜짐 — 막히면 돌아감' if vals['use_nav'] else '꺼짐 — 직진만')
        if node == PF and vals.get('camera_only') is not None:
            self.lamp['camonly'].set('on' if vals['camera_only'] else 'off')
            self.note['camonly'].setText('켜짐 — 라이다가 없어도 카메라로 거리 추정' if vals['camera_only'] else '꺼짐 — 다리 (라이다) 있어야 추종')
        if node == '/balance' and vals.get('tune.jump_max_v') is not None:
            jmax, jmin = vals['tune.jump_max_v'], vals.get('tune.jump_min_v') or 0.0
            self.lamp['jump'].set('on' if jmax > 0 else 'off')
            self.note['jump'].setText(f'켜짐 — {jmin:.2f}–{jmax:.2f} m/s 로 직진 중에 Y' if jmax > 0 else '꺼짐 (tune.jump_max_v 0)')
            self.bal_info.setText('  ·  '.join(f'{lab} {_fmt(vals[n])}{(" " + u) if u else ""}'
                                              for n, lab, u in BAL_INFO if vals.get(n) is not None))
        if node == '/cmd_mux' and vals.get('lidar_limit') is not None:
            # 꺼짐이 로봇 기본 (2026-10-09 사용자: 자기 다리·바퀴가 잡혀 주행이 막힘). 회피는 person_follow 가 한다
            self.lamp['lidar'].set('on' if vals['lidar_limit'] else 'warn')
            self.note['lidar'].setText('켜짐' if vals['lidar_limit'] else '꺼짐 (의도 — 회피는 추종 쪽 Nav2 가 함)')
        for r in self.rows:
            if r.node == node and r.name in vals:
                r.set_robot(vals[r.name])

    def _on_mode(self, m):
        if m != self.bal_mode:
            self.bal_mode = m
            self.update_kinds()

    def kind(self, node, name):
        """(실시간 | 재시작 때 | 잠김 | 노드 꺼짐, 이유)"""
        live = name in KNOWN_LIVE.get(node, ()) or name in self.live_params.get(node, ())
        if node in RESTART and not (live and node in self.nodes):
            return 'restart', '노트북에서 다시 시작할 때 이 값으로 띄운다'
        if node not in self.nodes:
            return ('absent', f'{node} 가 안 보임 — 켜지면 실시간으로 바꿀 수 있다') if live or node in KNOWN_LIVE else \
                ('locked', f'{node} 가 안 보임')
        if not live:
            return 'locked', f'{node} 가 이 값을 실시간으로 안 받음 (시작할 때만 읽음) — live_params 로 열어 달라고 요청함'
        if self.bal_mode != 'disarmed':
            return 'locked', f'실시간 = 해제 (disarmed) 하고 변경 — 지금 {self.bal_mode or "상태 모름"}. 패드 BACK 으로 앉혀서 해제한 뒤'
        return 'live', '바꾸면 바로 반영'

    def update_kinds(self):
        for r in self.rows:
            r.set_kind(*self.kind(r.node, r.name))

    def _on_topic(self, t, s):
        if t == 'follow/status':
            st = s.split()[0] if s else ''
            # 끝의 [camera] / [legs] = 추적 출처 (로봇 2026-10-15). camera = 발목 화면 높이로 거리 추정 → 회색 표시
            m = re.match(r'(.*?)\s*\[(camera|legs)\]\s*$', s)
            if m:
                s = m.group(1) + (' — 카메라 (거리 추정)' if m.group(2) == 'camera' else ' — 라이다 다리')
                self.t_follow.setStyleSheet('color:#888' if m.group(2) == 'camera' else 'color:#555')
            self.t_follow.setText(f'추종: {s}')
            self.lamp['follow'].set('on' if st not in ('', 'IDLE') else 'off')
            self.note['follow'].setText(s or '—')
            hold = st == 'HOLD'
            self.lamp['hold'].set('warn' if hold else 'off')
            self.note['hold'].setText('정지 중' if hold else '—')
        elif t == 'cmd_mux/status':
            self.t_mux.setText(f'명령: {s}')
        elif t == 'controller/state':
            self.t_ros.setText(f'균형: {s}' if s else '균형: 상태 안 들어옴 (/controller/state)')
            self.l_ros.set(('err' if s.startswith('fault') else 'on') if s else 'off')
        elif t == 'gesture/action':
            self.log(f'손짓: {s}')
        elif t == 'jump':
            self._on_jump(s)
        elif t == 'step_mode':
            self.step_on = s == '1'
            self._show_step()
        elif t == 'step_phase':
            self.step_phase = int(s)
            self._show_step()

    def _show_step(self):
        """상단 STEP 배지 (로봇 LCD 처럼 반전) + 단계, 기능 탭 줄."""
        phase = {0: '접근', 1: '막힘 → 후진', 2: '재주행 → 점프'}.get(self.step_phase, str(self.step_phase))
        self.b_step.setText(f' STEP · {phase} ')
        self.b_step.setVisible(self.step_on)
        self.lamp['step'].set('warn' if self.step_on else 'off')
        self.note['step'].setText(f'켜짐 — {phase} (평지 전용: 앞으로만 밀기, 스틱 놓으면 취소)' if self.step_on
                                  else '꺼짐 (패드 십자키 ↑ 켜기 / ↓ 끄기)')

    def _on_jump(self, s):
        # balance_node.cpp (2026-10-23): "jump: refused, operator (v 0.76 m/s, need 0.20-0.40 forward and |wz| <= 1.0)",
        # "jump: start, step-detect (v 0.30 m/s)", "step-detect: blocked -> back off (odo …)". 10-13 형식 (", who" 없음) 도 받는다
        who_ko = {'operator': 'Y', 'step-detect': '턱 감지'}
        m = re.match(r'jump: refused(?:, (\S+))? \(v (\S+) m/s, need (\S+)-(\S+) forward and \|wz\| <= (\S+)\)', s)
        if m:
            who, v, lo, hi, wz = m.groups()
            self.log(f'점프 거절 ({who_ko.get(who, who or "Y")}): 속도 {v} m/s — {lo}–{hi} m/s 로 직진 중 (회전 {wz} rad/s 이하) 에만', err=True)
            return
        m = re.match(r'jump: start(?:, (\S+))? \(v (\S+) m/s\)', s)
        if m:
            self.log(f'점프 시작 ({who_ko.get(m.group(1), m.group(1) or "Y")}, 속도 {m.group(2)} m/s)')
            self.b_jump.show()
            QtCore.QTimer.singleShot(2000, self.b_jump.hide)
            return
        if s.startswith('step-detect: '):
            ko = {'approach': '접근', 'blocked -> back off': '막힘 → 후진', 'run at the step': '재주행 → 점프'}
            m = re.match(r'step-detect: (.*?) \(', s)
            self.log(f'턱 감지: {ko.get(m.group(1), m.group(1)) if m else s[13:]}' + (s[s.index(' ('):] if ' (' in s else ''))
            return
        self.log(f'점프: {s[6:]}')

    def _on_done(self, what, ok, msg):
        if what.startswith('읽기'):
            return
        self.log(f'{what}: {"OK" if ok else "실패"} {msg}', err=not ok)

    # ---------------- 튜닝 동작
    def on_row_changed(self, r):
        if r.kind != 'live':
            return
        self.pending.setdefault(r.node, {})[r.name] = r.value()
        if self.auto.isChecked():
            self.send_timer.start()

    def flush(self):
        pend, self.pending = self.pending, {}
        for node, vals in pend.items():
            vals = {n: v for n, v in vals.items() if self.kind(node, n)[0] == 'live'}
            if vals:
                for nd in [node] + MIRROR.get(node, []):
                    if nd in self.nodes:
                        self.ros.set_params(nd, vals)

    def apply_all(self):
        for r in self.rows:
            if r.kind == 'live':
                self.pending.setdefault(r.node, {})[r.name] = r.value()
        self.flush()

    def read_all(self, only=None):
        by = {}
        for r in self.rows:
            if only is None or r.node in only:
                by.setdefault(r.node, []).append(r.name)
        for node, names in by.items():
            if node in self.nodes:
                self.ros.get_params(node, names)

    def defaults_all(self):
        if QtWidgets.QMessageBox.question(self, '기본값', '모든 값을 기본값 (설정 파일 값) 으로 돌릴까요?'
                                          + ('\n(바로 적용 켜짐: 실시간 칸은 로봇에도 보냄)' if self.auto.isChecked() else '')) \
                != QtWidgets.QMessageBox.Yes:
            return
        for r in self.rows:
            r.set_value(r.default)
            r.touched = True
            self.on_row_changed(r)

    def save_preset(self):
        os.makedirs(PRESETS, exist_ok=True)
        f, _ = QtWidgets.QFileDialog.getSaveFileName(self, '프리셋 저장', f'{PRESETS}/{time.strftime("%m%d_%H%M")}.yaml',
                                                     'YAML (*.yaml)')
        if f:
            data = {}
            for r in self.rows:
                data.setdefault(r.node, {})[r.name] = r.value()
            with open(f, 'w') as fh:
                yaml.safe_dump(data, fh, allow_unicode=True)
            self.log(f'프리셋 저장: {f}')

    def load_preset(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, '프리셋 불러오기', PRESETS, 'YAML (*.yaml)')
        if not f:
            return
        with open(f) as fh:
            data = yaml.safe_load(fh) or {}
        n = 0
        for r in self.rows:
            v = (data.get(r.node) or {}).get(r.name)
            if v is not None:
                r.set_value(v)
                r.touched = True
                self.on_row_changed(r)
                n += 1
        self.log(f'프리셋 불러옴 ({n} 개): {f}' + ('' if self.auto.isChecked() else ' — "지금 값 모두 적용" 으로 보내기'))

    def copy_diff(self):
        lines = [f'- {r.node} `{r.name}`: {_fmt(r.default)} → **{_fmt(r.value())}**'
                 for r in self.rows if (r.value() != r.default if r.is_bool else abs(r.value() - r.default) > 1e-9)]
        txt = '\n'.join(lines) or '(기본값과 다른 것 없음)'
        QtWidgets.QApplication.clipboard().setText(txt)
        self.log('바뀐 값 복사함:\n' + txt)

    def closeEvent(self, e):
        if self.att_win is not None:
            self.att_win.close()
        self.poll.stop()
        self.rpoll.stop()
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.ros.shutdown()
        e.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName('C-WANG 관제')
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    g = Gui()
    g.show()
    if os.environ.get('CW_GUI_SHOT'):   # 시험: 화면 저장 후 끝
        def shot():
            for i in range(g.tabs.count()):
                g.tabs.setCurrentIndex(i)
                app.processEvents()
                g.grab().save(f'{os.environ["CW_GUI_SHOT"]}_{i}.png')
            app.quit()
        QtCore.QTimer.singleShot(int(os.environ.get('CW_GUI_SHOT_MS', '4000')), shot)
    tick = QtCore.QTimer()          # 파이썬 시그널 처리용
    tick.start(300)
    tick.timeout.connect(lambda: None)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
