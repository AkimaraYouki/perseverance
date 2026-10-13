#!/usr/bin/env python3
"""C-WANG 관제 GUI — 기능 켜기/끄기 + 파라미터 튜닝 (GCS 노트북, 데스크톱 겸용).

  source ~/gen2_desk/env.sh && python3 ~/gen2_desk/cw_gui.py      (바탕화면 아이콘 "C-WANG 관제" 도 같음)

- 로봇 쪽 (추종 스택) 은 cw 와 같은 ssh 스크립트로, 노트북 쪽 (패드·손짓·SLAM·RViz·HUD) 은 터미널 없이 뒤에서 띄운다
  (로그 ~/gen2_desk/logs/*.log, '로그' 탭). cw check / cw down 과 같은 프로세스 패턴이라 섞어 써도 된다.
- 튜닝 탭: 실시간 반영되는 파라미터만 (person_follow 전부, cmd_mux lidar_limit, Nav2 RPP·코스트맵 팽창).
  패드·손짓은 '다시 시작' 때 적용. balance_node 는 시작할 때만 파라미터를 읽어서 여기서 다루지 않는다.
- 다른 컴퓨터 (데스크톱) 에서 패드·SLAM 이 이미 돌면 켜지 않는다 (heartbeat 2 개 / TF 충돌 방지).
"""
import glob
import os
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import yaml
from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

ROBOT = 'parksudo@192.168.50.124'
HOME = os.path.expanduser('~')
DESK = f'{HOME}/gen2_desk'
LOGS = f'{DESK}/logs'
PRESETS = f'{DESK}/gui_presets'
LENV = f'source {DESK}/env.sh'
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4', '-o', 'ControlMaster=auto',
       '-o', f'ControlPath={HOME}/.ssh/cm-%r@%h:%p', '-o', 'ControlPersist=120']

# ------------------------------------------------------------------ 파라미터 표
# (이름, 설명, 최소, 최대, 간격, 기본값, 단위)  — 기본값 = 소스/설정 파일 값 (2026-10-13)
FOLLOW = [
    ('follow_dist_m', '따라갈 거리', 0.3, 2.0, 0.05, 0.8, 'm'),
    ('vx_max', '최대 전진 속도', 0.0, 1.0, 0.05, 0.7, 'm/s'),
    ('wz_max', '최대 회전 속도', 0.0, 2.5, 0.1, 1.0, 'rad/s'),
    ('k_dist', '거리 게인', 0.0, 3.0, 0.05, 0.8, ''),
    ('k_yaw', '방향 게인', 0.0, 5.0, 0.1, 1.5, ''),
    ('bearing_tol_deg', '방향 허용 오차', 0.0, 30.0, 1.0, 10.0, '°'),
    ('search_wz', '찾을 때 회전 속도', 0.0, 1.5, 0.05, 0.5, 'rad/s'),
    ('occluded_s', '가려짐 허용 시간', 0.0, 15.0, 0.5, 5.0, 's'),
    ('predict_s', '가려졌을 때 예측 시간', 0.0, 5.0, 0.1, 1.5, 's'),
    ('grace_s', '잠깐 놓쳐도 추종 유지', 0.0, 3.0, 0.1, 0.5, 's'),
    ('legs_only_s', '카메라 없이 다리만으로', 0.0, 5.0, 0.1, 1.0, 's'),
    ('max_gate_m', '대상 이동 허용 범위', 0.5, 5.0, 0.1, 2.5, 'm'),
    ('appearance_min', '옷 색 일치 (근처)', 0.0, 1.0, 0.05, 0.5, ''),
    ('reacquire_min', '옷 색 일치 (재획득)', 0.0, 1.0, 0.05, 0.65, ''),
]
AVOID = [
    ('corridor_margin_m', '직진 통로 여유 폭', 0.0, 0.5, 0.01, 0.12, 'm'),
    ('nav_hold_s', '막힌 뒤 Nav2 유지', 0.0, 10.0, 0.5, 2.0, 's'),
    ('nav_replan_s', 'Nav2 경로 다시 계산 주기', 0.1, 3.0, 0.1, 0.5, 's'),
]
RPP = [
    ('FollowPath.desired_linear_vel', '우회 주행 속도', 0.1, 1.0, 0.05, 0.7, 'm/s'),
    ('FollowPath.lookahead_dist', '앞보기 거리', 0.2, 2.0, 0.05, 0.6, 'm'),
    ('FollowPath.min_lookahead_dist', '앞보기 최소', 0.1, 1.5, 0.05, 0.3, 'm'),
    ('FollowPath.max_lookahead_dist', '앞보기 최대', 0.3, 3.0, 0.05, 1.2, 'm'),
    ('FollowPath.rotate_to_heading_angular_vel', '제자리 회전 속도', 0.1, 2.5, 0.1, 1.0, 'rad/s'),
    ('FollowPath.max_angular_accel', '최대 회전 가속', 0.5, 6.0, 0.1, 3.0, 'rad/s²'),
    ('FollowPath.regulated_linear_scaling_min_speed', '곡선에서 최소 속도', 0.05, 0.7, 0.05, 0.25, 'm/s'),
    ('FollowPath.min_approach_linear_velocity', '목표 근처 최소 속도', 0.0, 0.5, 0.05, 0.1, 'm/s'),
]
COSTMAP = [
    ('inflation_layer.inflation_radius', '장애물 팽창 반경', 0.1, 1.0, 0.05, 0.40, 'm'),
    ('inflation_layer.cost_scaling_factor', '팽창 비용 감쇠', 0.5, 10.0, 0.5, 4.0, ''),
]
PAD = [
    ('vx_max', '패드 최대 전진 속도', 0.1, 1.0, 0.05, 0.7, 'm/s'),
    ('wz_max', '패드 최대 회전 속도', 0.2, 3.0, 0.1, 2.0, 'rad/s'),
    ('deadzone', '스틱 데드존', 0.0, 0.3, 0.01, 0.08, ''),
    ('height_rate', '몸 높이 변화 속도', 0.01, 0.15, 0.01, 0.05, 'm/s'),
]
GESTURE = [
    ('process_hz', '손짓 처리 주기', 1.0, 30.0, 1.0, 15.0, 'Hz'),
    ('min_score', '최소 확신도', 0.3, 1.0, 0.05, 0.6, ''),
    ('hold_s', '손짓 유지 시간', 0.1, 2.0, 0.1, 0.6, 's'),
]
# 실시간 그룹: (제목, 대상 노드들, 표, 설명)
LIVE_GROUPS = [
    ('사람 추종', ['/person_follow'], FOLLOW, '로봇 person_follow — 바로 반영'),
    ('장애물 회피 — 추종 쪽', ['/person_follow'], AVOID, '직진 통로가 막히면 Nav2 로 우회'),
    ('장애물 회피 — Nav2 RPP', ['/controller_server'], RPP, '우회 경로 따라가기 (Regulated Pure Pursuit)'),
    ('장애물 회피 — 코스트맵', ['/local_costmap/local_costmap', '/global_costmap/global_costmap'], COSTMAP,
     '로컬·글로벌 코스트맵 둘 다'),
]


# ------------------------------------------------------------------ ROS (뒤 스레드)
def _pv(v):
    if isinstance(v, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))


def _from_pv(p):
    return {ParameterType.PARAMETER_BOOL: p.bool_value, ParameterType.PARAMETER_INTEGER: p.integer_value,
            ParameterType.PARAMETER_DOUBLE: p.double_value, ParameterType.PARAMETER_STRING: p.string_value}.get(p.type)


class Ros(QtCore.QObject):
    """rclpy 노드를 뒤 스레드에서 돌린다. 결과는 Qt 신호로 GUI 스레드에 넘긴다."""
    graph = QtCore.pyqtSignal(set)                  # 지금 보이는 노드 이름들
    params = QtCore.pyqtSignal(str, dict)            # 노드, {이름: 값}
    done = QtCore.pyqtSignal(str, bool, str)         # 무엇, 성공, 메시지
    topic = QtCore.pyqtSignal(str, str)              # 토픽, 내용

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = Node('gcs_gui')
        self.cli = {}
        self.lock = threading.Lock()
        for t in ('follow/status', 'cmd_mux/status', 'gesture/action'):
            self.node.create_subscription(String, t, lambda m, t=t: self.topic.emit(t, m.data), 10)
        try:   # 균형 상태 (100 Hz) — 최신 것만, 0.5 초마다 GUI 로
            from gen2_msgs.msg import ControllerState
            self._cs = None
            self.node.create_subscription(ControllerState, 'controller/state', self._state, qos_profile_sensor_data)
            self.node.create_timer(0.5, self._emit_state)
        except ImportError:
            pass
        self.node.create_timer(2.0, self._graph)
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

    def _state(self, m):
        self._cs = (time.monotonic(), m.mode, m.fault, m.pitch, m.v, m.vx_cmd, m.wz_cmd)

    def _emit_state(self):
        if self._cs:
            t, mode, fault, pitch, v, vx, wz = self._cs
            if time.monotonic() - t < 1.5:
                s = f'{mode}' + (f' ({fault})' if fault else '') + f' · 기울기 {pitch*57.3:+.1f}° · 속도 {v:+.2f} m/s'
                self.topic.emit('controller/state', s)
                return
        self.topic.emit('controller/state', '')

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
                ok = getattr(r, 'success', True)
                self.done.emit(what, ok, getattr(r, 'message', ''))
        f.add_done_callback(cb)

    def get_params(self, node, names):
        req = GetParameters.Request(names=list(names))

        def ok(r):
            vals = {n: _from_pv(v) for n, v in zip(names, r.values) if v.type != ParameterType.PARAMETER_NOT_SET}
            self.params.emit(node, vals)
        self._call(GetParameters, f'{node}/get_parameters', req, f'읽기 {node}', ok)

    def set_params(self, node, vals):
        req = SetParameters.Request(parameters=[Parameter(name=n, value=_pv(v)) for n, v in vals.items()])

        def ok(r):
            bad = [f'{n}: {x.reason}' for n, x in zip(vals, r.results) if not x.successful]
            self.done.emit(f'적용 {node}', not bad, '; '.join(bad) or ', '.join(f'{n}={v:g}' if not isinstance(v, bool)
                                                                                  else f'{n}={v}' for n, v in vals.items()))
            if not bad:
                self.params.emit(node, dict(vals))
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


def ros_args(vals):
    return ' '.join(f'-p {k}:={v}' for k, v in vals.items())


# 노트북 쪽 기능: 키 → (이름, 프로세스 패턴, 다른 컴퓨터에서 돌면 막을 ROS 노드)
LOCAL = {
    'pad':     ('패드 (joy + pad_teleop)', 'lib/gen2_tools/pad_teleop', '/pad_teleop'),
    'gesture': ('손짓 인식 (손바닥 정지 · 엄지 재개)', f'^{DESK}/venv_vision/bin/python -m gen2_tools.gesture_node', None),
    'slam':    ('SLAM 지도 (노트북)', '[a]sync_slam_toolbox_node', '/slam_toolbox'),
    'rviz':    ('RViz (지도·라이다·코스트맵·카메라)', '^rviz2 -d', None),
    'hud':     ('HUD (카메라 + 사람 상자)', f'^/usr/bin/python3 {DESK}/hud.py', None),
    'dash':    ('상태판 (전원·배터리·모터)', 'desk_dash_robot.py|wifi -r', None),
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
    changed = QtCore.pyqtSignal(str, float)

    def __init__(self, name, label, lo, hi, step, default, unit):
        super().__init__()
        self.name, self.default, self.step, self.lo = name, default, step, lo
        self.robot = None
        self.touched = False             # 사용자가 바꾼 칸은 로봇 값으로 덮어쓰지 않는다
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(0, 1, 0, 1)
        lab = QtWidgets.QLabel(label)
        lab.setToolTip(name)
        lab.setMinimumWidth(170)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, int(round((hi - lo) / step)))
        self.spin = QtWidgets.QDoubleSpinBox()
        dec = max(0, len(f'{step:g}'.split('.')[1]) if '.' in f'{step:g}' else 0)
        self.spin.setDecimals(dec)
        self.spin.setRange(lo, hi)
        self.spin.setSingleStep(step)
        self.spin.setSuffix(f' {unit}' if unit else '')
        self.spin.setFixedWidth(120)
        self.cur = QtWidgets.QLabel('로봇: —')
        self.cur.setMinimumWidth(95)
        self.cur.setStyleSheet('color:#888')
        h.addWidget(lab)
        h.addWidget(self.slider, 1)
        h.addWidget(self.spin)
        h.addWidget(self.cur)
        self.slider.valueChanged.connect(lambda i: self.spin.setValue(self.lo + i * self.step))
        self.spin.valueChanged.connect(self._spin)
        self.set_value(default)

    def _spin(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round((v - self.lo) / self.step)))
        self.slider.blockSignals(False)
        self._mark()
        self.touched = True
        self.changed.emit(self.name, v)

    def value(self):
        return self.spin.value()

    def set_value(self, v):
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)
        self._spin_silent(v)

    def _spin_silent(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round((v - self.lo) / self.step)))
        self.slider.blockSignals(False)
        self._mark()

    def set_robot(self, v):
        self.robot = v
        if not self.touched and v is not None:   # 처음엔 로봇의 지금 값을 보여 준다
            self.set_value(v)
        self.cur.setText('로봇: —' if v is None else f'로봇: {v:g}')
        self._mark()

    def _mark(self):
        if self.robot is None:
            self.cur.setStyleSheet('color:#888')
        elif abs(self.robot - self.spin.value()) > 1e-6:
            self.cur.setStyleSheet('color:#e67e22; font-weight:bold')     # 아직 적용 안 됨
        else:
            self.cur.setStyleSheet('color:#27ae60')


class Group(QtWidgets.QGroupBox):
    """파라미터 묶음 — live: 노드에 바로 보냄 / 아니면 다음 시작 때 쓰는 값."""

    def __init__(self, title, nodes, table, note, gui, live=True):
        super().__init__(title)
        self.nodes, self.gui, self.live = nodes, gui, live
        v = QtWidgets.QVBoxLayout(self)
        n = QtWidgets.QLabel(note)
        n.setStyleSheet('color:#888')
        v.addWidget(n)
        self.rows = {}
        for spec in table:
            r = ParamRow(*spec)
            r.changed.connect(self._changed)
            self.rows[spec[0]] = r
            v.addWidget(r)
        self.pending = {}
        self.timer = QtCore.QTimer(self, singleShot=True, interval=300)
        self.timer.timeout.connect(self.flush)

    def _changed(self, name, val):
        self.pending[name] = val
        if self.live and self.gui.auto.isChecked():
            self.timer.start()

    def flush(self, all_rows=False):
        if not self.live:
            return
        vals = {n: r.value() for n, r in self.rows.items()} if all_rows else dict(self.pending)
        self.pending.clear()
        if vals:
            for nd in self.nodes:
                self.gui.ros.set_params(nd, vals)

    def read(self):
        if self.live:
            for nd in self.nodes:
                self.gui.ros.get_params(nd, list(self.rows))

    def on_params(self, node, vals):
        if node != self.nodes[0]:
            return
        for n, v in vals.items():
            if n in self.rows and isinstance(v, (int, float)) and not isinstance(v, bool):
                self.rows[n].set_robot(float(v))

    def defaults(self):
        for r in self.rows.values():
            r.set_value(r.default)
            r.touched = True
            self.pending[r.name] = r.default
        if self.live and self.gui.auto.isChecked():
            self.timer.start()

    def values(self):
        return {n: r.value() for n, r in self.rows.items()}


# ------------------------------------------------------------------ 창
class Gui(QtWidgets.QMainWindow):
    sshres = QtCore.pyqtSignal(str, int, str)    # 무엇, 종료 코드, 출력

    def __init__(self):
        super().__init__()
        self.setWindowTitle('C-WANG 관제')
        self.resize(980, 760)
        self.ros = Ros()
        self.pool = ThreadPoolExecutor(4)
        self.nodes = set()
        self.robot_ok = None
        self.robot_info = {}
        self.follow_status = ''
        self.follow_on = False

        w = QtWidgets.QWidget()
        self.setCentralWidget(w)
        top = QtWidgets.QVBoxLayout(w)
        top.addWidget(self._status_bar())
        self.tabs = QtWidgets.QTabWidget()
        top.addWidget(self.tabs, 1)
        self.tabs.addTab(self._features_tab(), '기능 켜기/끄기')
        self.tabs.addTab(self._tune_tab(), '파라미터 튜닝')
        self.tabs.addTab(self._log_tab(), '로그')

        self.ros.graph.connect(self._on_graph)
        self.ros.params.connect(self._on_params)
        self.ros.done.connect(self._on_done)
        self.ros.topic.connect(self._on_topic)
        self.sshres.connect(self._on_ssh)

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
        g.addWidget(self.l_robot, 0, 0)
        g.addWidget(self.t_robot, 0, 1)
        g.addWidget(self.l_ros, 0, 2)
        g.addWidget(self.t_ros, 0, 3)
        g.addWidget(self.t_follow, 1, 1)
        g.addWidget(self.t_mux, 1, 3)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(3, 1)
        sit = QtWidgets.QPushButton('로봇 앉기')
        sit.setToolTip('balance/sit — 천천히 앉아서 해제 (패드 BACK 과 같음)')
        sit.setStyleSheet('background:#c0392b; color:white; font-weight:bold; padding:8px 14px')
        sit.clicked.connect(lambda: self.ros.trigger('/balance/sit', '로봇 앉기'))
        stop = QtWidgets.QPushButton('추종 정지')
        stop.setStyleSheet('background:#e67e22; color:white; font-weight:bold; padding:8px 14px')
        stop.clicked.connect(lambda: self.ros.set_bool('/follow/enable', False, '추종 끄기'))
        g.addWidget(stop, 0, 4, 2, 1)
        g.addWidget(sit, 0, 5, 2, 1)
        return f

    # ---------------- 기능 탭
    def _row(self, grid, r, title, lamp, buttons, note=''):
        grid.addWidget(lamp, r, 0)
        t = QtWidgets.QLabel(title)
        t.setStyleSheet('font-weight:bold')
        grid.addWidget(t, r, 1)
        st = QtWidgets.QLabel(note)
        st.setStyleSheet('color:#888')
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

        # 로봇
        box = QtWidgets.QGroupBox('로봇 (젯슨)')
        g = QtWidgets.QGridLayout(box)
        g.setColumnStretch(2, 1)
        self.lamp = {}
        self.note = {}
        rows = [
            ('stack', '사람 추종 스택', [self._btn('켜기', lambda: self.robot_cmd('stack_on')),
                                    self._btn('끄기', lambda: self.robot_cmd('stack_off'))]),
            ('follow', '추종 활성화 (= 패드 X)', [self._btn('켜기 (앞사람 등록)', lambda: self.ros.set_bool(
                '/follow/enable', True, '추종 켜기')), self._btn('끄기', lambda: self.ros.set_bool(
                    '/follow/enable', False, '추종 끄기'))]),
            ('hold', '일시 정지 (= 손바닥)', [self._btn('정지', lambda: self.ros.set_bool('/follow/hold', True, '일시 정지')),
                                        self._btn('재개', lambda: self.ros.set_bool('/follow/hold', False, '재개'))]),
            ('nav', '장애물 우회 (Nav2)', [self._btn('켜기', lambda: self.ros.set_params('/person_follow', {'use_nav': True})),
                                        self._btn('끄기', lambda: self.ros.set_params('/person_follow', {'use_nav': False}))]),
            ('lidar', '장애물 앞 정지·감속 (라이다)', [
                self._btn('켜기', lambda: self.ros.set_params('/cmd_mux', {'lidar_limit': True})),
                self._btn('끄기', self.lidar_off, '끄면 앞에 벽이 있어도 멈추지 않는다')]),
        ]
        for i, (k, t, b) in enumerate(rows):
            self.lamp[k] = Lamp()
            self.note[k] = self._row(g, i, t, self.lamp[k], b)
        v.addWidget(box)

        # 노트북
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
        hint = QtWidgets.QLabel('패드: START 일어서기 · BACK 앉기 · LB+RB 비상 해제 · X 추종 · Y 점프 · A 누르는 동안 정지.  '
                                '데스크톱과 노트북에서 패드·SLAM 을 동시에 켜지 말 것 (이 창이 막아 줌).')
        hint.setWordWrap(True)
        hint.setStyleSheet('color:#888')
        v.addWidget(hint)
        v.addStretch()
        return w

    # ---------------- 튜닝 탭
    def _tune_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        h = QtWidgets.QHBoxLayout()
        self.auto = QtWidgets.QCheckBox('바꾸면 바로 적용')
        self.auto.setChecked(True)
        h.addWidget(self.auto)
        h.addWidget(self._btn('지금 값 모두 적용', lambda: [g.flush(True) for g in self.groups]))
        h.addWidget(self._btn('로봇에서 다시 읽기', self.read_all))
        h.addWidget(self._btn('기본값으로', self.defaults_all))
        h.addStretch()
        h.addWidget(self._btn('프리셋 저장', self.save_preset))
        h.addWidget(self._btn('프리셋 불러오기', self.load_preset))
        h.addWidget(self._btn('바뀐 값 복사 (comms 용)', self.copy_diff, '로봇 설정에 영구 반영하려면 comms 로 요청'))
        v.addLayout(h)
        lg = QtWidgets.QLabel('주황 = 아직 로봇에 반영 안 됨, 초록 = 반영됨. 로봇을 재시작하면 기본값으로 돌아간다 '
                              '(영구 반영은 "바뀐 값 복사" → comms/to-robot).')
        lg.setStyleSheet('color:#888')
        lg.setWordWrap(True)
        v.addWidget(lg)
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        inner = QtWidgets.QWidget()
        iv = QtWidgets.QVBoxLayout(inner)
        self.groups = [Group(t, n, tb, note, self) for t, n, tb, note in LIVE_GROUPS]
        self.g_pad = Group('패드 (pad_teleop)', [], PAD, '노트북 — "다시 시작" 또는 다음 켤 때 적용', self, live=False)
        self.g_gest = Group('손짓 (gesture_node)', [], GESTURE, '노트북 — "다시 시작" 또는 다음 켤 때 적용', self, live=False)
        for g in self.groups + [self.g_pad, self.g_gest]:
            iv.addWidget(g)
        iv.addStretch()
        sa.setWidget(inner)
        v.addWidget(sa, 1)
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
        i = self.logsel.findText(cur)
        self.logsel.setCurrentIndex(max(i, 0))
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
            if '/person_follow' in self.nodes:
                self.log('사람 추종 스택 이미 켜짐')
                return
            self.ssh('추종 스택 켜기', 'setsid nohup ~/gen2_ws/logs/run_follow.sh > ~/gen2_ws/logs/run_follow.out 2>&1 '
                     '< /dev/null & echo started')
            self.log('추종 스택 켜는 중 — 10–20 초 뒤 "추종 활성화" 또는 패드 X')
        elif k == 'stack_off':
            self.ssh('추종 스택 끄기', '~/gen2_ws/logs/stop_follow.sh')

    def _poll_robot(self):
        self.ssh('_poll', 'printf "%s %s %s\\n" $(systemctl is-active gen2-bench) $(systemctl is-active gen2-balance) '
                 '$(pgrep -f "[s]lam_toolbox" >/dev/null && echo slam || echo noslam)')

    def _on_ssh(self, what, code, out):
        if what == '_poll':
            self.robot_ok = code == 0
            if self.robot_ok:
                p = (out.split() + ['?'] * 3)[:3]
                self.robot_info = {'bench': p[0], 'balance': p[1], 'slam': p[2] == 'slam'}
                ok = p[0] == 'active' and p[1] == 'active'
                self.l_robot.set('on' if ok else 'warn')
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
            launch('pad_teleop', f'ros2 run gen2_tools pad_teleop --ros-args {ros_args(self.g_pad.values())}')
            if not glob.glob('/dev/input/js*'):
                self.log('게임패드가 안 보임 (블루투스 연결 확인)', err=True)
        elif k == 'gesture':
            launch('gesture', f'{DESK}/venv_vision/bin/python -m gen2_tools.gesture_node --ros-args '
                              f'{ros_args(self.g_gest.values())}')
        elif k == 'slam':
            if self.robot_info.get('slam'):
                self.log('로봇에서 SLAM 이 돌고 있음 — 둘이면 지도·TF 가 싸운다', err=True)
                return
            launch('slam', 'ros2 launch gen2_bringup slam.launch.py')
        elif k == 'rviz':
            launch('rviz', f'rviz2 -d {DESK}/gen2_desktop.rviz')
        elif k == 'hud':
            launch('hud', f'/usr/bin/python3 {DESK}/hud.py')
        elif k == 'dash':   # 터미널 화면 프로그램이라 창 하나
            subprocess.Popen(['gnome-terminal', '--title=상태판', '--', 'bash', '-c', f'{LENV}; wifi -r'],
                             start_new_session=True, stderr=subprocess.DEVNULL)
        self.log(f'{name}: 켬')

    def local_off(self, k):
        name, pat, _ = LOCAL[k]
        if k == 'pad':
            kill('lib/gen2_tools/pad_teleop')
            kill('lib/joy/joy_node')
        elif k == 'slam':
            kill('[a]sync_slam_toolbox_node|[g]en2_bringup slam.launch')
        elif k == 'dash':
            kill('desk_dash_robot.py|wifi -r')
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
        QtCore.QTimer.singleShot(1200, lambda: self.local_on(k))

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
        if '/person_follow' not in self.nodes:
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
            if k == 'pad':
                joy = running('lib/joy/joy_node')
                pad = bool(glob.glob('/dev/input/js*'))
                self.lamp[k].set('on' if on and joy and pad else 'warn' if on or joy else 'off')
                self.note[k].setText(('켜짐' if on else '꺼짐') + (' · 게임패드 연결됨' if pad else ' · 게임패드 없음')
                                     + (' · ⚠ 다른 컴퓨터에서 패드가 돎' if self._other_pc(k) else ''))
            else:
                other = self._other_pc(k)
                self.lamp[k].set('on' if on else 'err' if other else 'off')
                self.note[k].setText('켜짐' if on else '⚠ 다른 컴퓨터에서 돎' if other else '꺼짐')

    def _on_graph(self, names):
        self.nodes = names
        stack = '/person_follow' in names
        self.lamp['stack'].set('on' if stack else 'off')
        parts = [n for n, k in (('사람검출', '/person_detector'), ('다리', '/leg_detector'),
                                ('추종', '/person_follow'), ('Nav2', '/controller_server')) if k in names]
        self.note['stack'].setText(' · '.join(parts) if parts else '꺼짐')
        self.l_ros.set('on' if '/balance_node' in names or any('balance' in n for n in names) else
                       'warn' if names - {'/gcs_gui'} else 'off')
        if stack:
            self.ros.get_params('/person_follow', ['use_nav'])
        else:
            for k in ('follow', 'hold', 'nav'):
                self.lamp[k].set('off')
                self.note[k].setText('추종 스택 꺼짐')
        if '/cmd_mux' in names:
            self.ros.get_params('/cmd_mux', ['lidar_limit'])
        else:
            self.lamp['lidar'].set('off')
            self.note['lidar'].setText('cmd_mux 안 보임')
        self.show_log() if self.logsel.currentIndex() else None

    def _on_params(self, node, vals):
        if node == '/person_follow' and 'use_nav' in vals:
            self.lamp['nav'].set('on' if vals['use_nav'] else 'warn')
            self.note['nav'].setText('켜짐 — 막히면 돌아감' if vals['use_nav'] else '꺼짐 — 직진만')
        if node == '/cmd_mux' and 'lidar_limit' in vals:
            self.lamp['lidar'].set('on' if vals['lidar_limit'] else 'err')
            self.note['lidar'].setText('켜짐' if vals['lidar_limit'] else '⚠ 꺼짐 — 장애물 앞에서 안 멈춤')
        for g in self.groups:
            g.on_params(node, vals)

    def _on_topic(self, t, s):
        if t == 'follow/status':
            self.follow_status = s
            st = s.split()[0] if s else ''
            self.t_follow.setText(f'추종: {s}')
            on = st not in ('', 'IDLE')
            self.lamp['follow'].set('on' if on else 'off')
            self.note['follow'].setText(s or '—')
            hold = st == 'HOLD'
            self.lamp['hold'].set('warn' if hold else 'off')
            self.note['hold'].setText('정지 중' if hold else '—')
        elif t == 'cmd_mux/status':
            self.t_mux.setText(f'명령: {s}')
        elif t == 'controller/state':
            self.t_ros.setText(f'균형: {s}' if s else '균형: 상태 안 들어옴 (/controller/state)')
            if s:
                self.l_ros.set('err' if s.startswith('fault') else 'on')
        elif t == 'gesture/action':
            self.log(f'손짓: {s}')

    def _on_done(self, what, ok, msg):
        if what.startswith('읽기'):
            return
        self.log(f'{what}: {"OK" if ok else "실패"} {msg}', err=not ok)

    # ---------------- 튜닝 동작
    def read_all(self):
        for g in self.groups:
            g.read()

    def defaults_all(self):
        if QtWidgets.QMessageBox.question(self, '기본값', '모든 튜닝값을 기본값으로 돌릴까요?'
                                          + ('\n(바로 적용 켜짐: 로봇에도 보냄)' if self.auto.isChecked() else '')) \
                == QtWidgets.QMessageBox.Yes:
            for g in self.groups + [self.g_pad, self.g_gest]:
                g.defaults()

    def _all_groups(self):
        return {g.title(): g for g in self.groups + [self.g_pad, self.g_gest]}

    def save_preset(self):
        os.makedirs(PRESETS, exist_ok=True)
        f, _ = QtWidgets.QFileDialog.getSaveFileName(self, '프리셋 저장', f'{PRESETS}/{time.strftime("%m%d_%H%M")}.yaml',
                                                     'YAML (*.yaml)')
        if f:
            with open(f, 'w') as fh:
                yaml.safe_dump({t: g.values() for t, g in self._all_groups().items()}, fh, allow_unicode=True)
            self.log(f'프리셋 저장: {f}')

    def load_preset(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, '프리셋 불러오기', PRESETS, 'YAML (*.yaml)')
        if not f:
            return
        with open(f) as fh:
            data = yaml.safe_load(fh) or {}
        for t, g in self._all_groups().items():
            for n, v in (data.get(t) or {}).items():
                if n in g.rows:
                    g.rows[n].set_value(float(v))
                    g.rows[n].touched = True
                    g.pending[n] = float(v)
            if g.live and self.auto.isChecked():
                g.flush()
        self.log(f'프리셋 불러옴: {f}' + (' (로봇에 적용)' if self.auto.isChecked() else ' — "지금 값 모두 적용" 으로 보내기'))

    def copy_diff(self):
        lines = []
        for t, g in self._all_groups().items():
            node = g.nodes[0] if g.nodes else ('pad_teleop' if g is self.g_pad else 'gesture_node')
            for n, r in g.rows.items():
                if abs(r.value() - r.default) > 1e-9:
                    lines.append(f'- {node} `{n}`: {r.default:g} → **{r.value():g}**  ({t})')
        txt = '\n'.join(lines) or '(기본값과 다른 것 없음)'
        QtWidgets.QApplication.clipboard().setText(txt)
        self.log('바뀐 값 복사함:\n' + txt)

    def closeEvent(self, e):
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
    app.aboutToQuit.connect(g.ros.shutdown)
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
