"""관제 GUI 오른쪽 상태판 — 한눈에 볼 값만 큰 글씨로.

Ros.panel 신호 (5 Hz) 의 dict 를 받아 그린다. 값이 1.5 초 넘게 안 들어오면 회색 '—'.
  모드 · 모터 배터리 (6S) · 속도 · 가속도 (바퀴 축 속도의 변화, 0.2 s 평활) · 기울기 · 다리 높이 · 컴퓨터 배터리 (3S) ·
  모터 최고 온도와 이상 (motor_monitor /diagnostics, 1 Hz)
배터리 색: 셀당 3.50 V 아래 주황, 3.30 V 아래 빨강 (sensor_hub.yaml warn_cell_v / error_cell_v 와 같음).
2026-10-14 센서 허브 (PM02 전압·전류) 를 로봇에서 뺐다 → /power/* 없음 = '센서 없음', 컴퓨터 배터리 칸은 숨김.
허브를 다시 달면 (bench.launch hub:=true) 값이 들어오는 대로 다시 보인다.
"""
import math

from PyQt5 import QtCore, QtGui, QtWidgets

WARN_CELL, ERR_CELL = 3.50, 3.30
OK, WARN, ERR, DIM, TXT = '#27ae60', '#e67e22', '#c0392b', '#95a5a6', '#1b2631'
MODE_KO = {'balance': ('균형', OK), 'stand': ('서는 중', '#2980b9'), 'disarmed': ('해제', DIM), 'fault': ('고장', ERR)}


class Card(QtWidgets.QFrame):
    def __init__(self, title, big=30):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setStyleSheet('Card { background:#f8f9f9; border:1px solid #d5dbdb; border-radius:6px; }')
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(10, 4, 10, 6)
        v.setSpacing(0)
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet('color:#566573; font-weight:bold')
        self.value = QtWidgets.QLabel('—')
        f = self.value.font()
        f.setPointSize(big)
        f.setBold(True)
        self.value.setFont(f)
        self.sub = QtWidgets.QLabel('')
        self.sub.setStyleSheet('color:#566573')
        self.sub.setWordWrap(True)
        for w in (self.title, self.value, self.sub):
            v.addWidget(w)

    def set(self, value, sub='', color=TXT):
        self.value.setText(value)
        self.value.setStyleSheet(f'color:{color}')
        self.sub.setText(sub)
        self.sub.setVisible(bool(sub))


class StatusPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedWidth(290)
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(4, 0, 0, 0)
        v.setSpacing(6)
        self.mode = Card('모드', 24)
        self.batt = Card('모터 배터리 (6S)', 30)
        self.speed = Card('속도', 30)
        self.acc = Card('가속도', 26)
        self.tilt = Card('기울기', 16)
        self.leg = Card('다리 높이 (왼 / 오른)', 18)
        self.cbatt = Card('컴퓨터 배터리 (3S)', 18)
        self.temp = Card('모터 (최고 온도 · 이상)', 18)
        for c in (self.mode, self.batt, self.speed, self.acc, self.tilt, self.leg, self.cbatt, self.temp):
            v.addWidget(c)
        v.addStretch(1)

    @staticmethod
    def _batt(card, b, cells):
        if b is None:
            card.set('센서 없음', '허브 뺌 (2026-10-14) — 직접 확인: 6S 셀당 3.5 V 아래면 충전' if cells == 6 else '', DIM)
            return
        volts, pct, present = b
        if not present:
            card.set(f'{volts:.1f} V', '배터리 없음 / 값 이상', DIM)
            return
        cell = volts / cells
        col = ERR if cell < ERR_CELL else WARN if cell < WARN_CELL else OK
        p = '' if pct is None or math.isnan(pct) else f' · {pct * 100:.0f} %'
        card.set(f'{volts:.1f} V', f'셀당 {cell:.2f} V{p}' + (' — 곧 충전' if col != OK else ''), col)

    def update_values(self, d):
        cs = d.get('cs')
        if cs is None:
            for c in (self.mode, self.speed, self.acc, self.tilt, self.leg):
                c.set('—', '균형 상태 안 들어옴 (/controller/state)', DIM)
        else:
            name, col = MODE_KO.get(cs['mode'], (cs['mode'] or '?', TXT))
            self.mode.set(name, cs['fault'], col)
            v = cs['v']
            self.speed.set(f'{v:+.2f} m/s', f'{v * 3.6:+.1f} km/h · 명령 {cs["vx_cmd"]:+.2f} m/s')
            a = cs['acc']
            self.acc.set(f'{a:+.2f} m/s²', '+ = 앞으로 빨라짐 (바퀴 축 속도 변화)',
                         ERR if abs(a) > 3.0 else WARN if abs(a) > 1.5 else TXT)
            p, r = cs['pitch'], cs['roll']
            self.tilt.set(f'피치 {p:+.1f}°  롤 {r:+.1f}°', '+ = 앞 숙임 / 오른쪽 내려감',
                          ERR if max(abs(p), abs(r)) > 20 and cs['mode'] in ('stand', 'balance') else TXT)   # 앉아 있으면 -63° 가 정상
            hl, hr = cs['h']
            self.leg.set(f'{hl * 1000:.0f} / {hr * 1000:.0f} mm', '')
        self._batt(self.batt, d.get('motor'), 6)
        self.cbatt.setVisible(d.get('compute') is not None)
        self._batt(self.cbatt, d.get('compute'), 3)
        t = d.get('temp')
        if t is None:
            self.temp.set('—', '안 들어옴 (/diagnostics motor_monitor)', DIM)
        else:
            tc, who, level, bad = t
            col = ERR if level >= 2 or tc > 75 else WARN if level == 1 or tc > 65 else TXT
            self.temp.set('—' if math.isnan(tc) else f'{tc:.0f} °C', bad or f'{who} · 한계 80 °C', col)
