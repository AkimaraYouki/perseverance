"""로봇 자세 (아두파일럿 HUD 처럼) — roll · pitch · yaw 를 실시간으로. 관제 GUI 아래 줄 + '크게 보기' 창.

위에서 내려다보면 기울기 성분이 안 보이니 축마다 따로 그린다:
  자세계 (인공 수평선, 피치 사다리 + 롤 눈금) · 위에서 (요, 나침반) · 옆에서 (피치) · 뒤에서 (롤) · 10 초 기록.
부호 (REP-103 base_link, imu_cal.py 와 같음): 피치 + = 앞 숙임, 롤 + = 오른쪽 내려감, 요 + = 왼쪽으로 돎 (위에서 반시계).
데이터 (Ros.attitude): 롤·피치 = /controller/state (100 Hz, 중력 방향 — GUI 가 이미 받는 것), 요 = /odom (자이로 적분).
/imu/data 500 Hz 를 rclpy 로 받으면 노트북 한 코어의 45–75 % 를 써서 (2026-10-14 잼) 쓰지 않는다.
요는 balance_node 가 서 있을 때만 적분하고 (해제 중엔 멈춤) 기준이 임의 → '요 0 맞추기' 로 지금 방향을 0 으로.
"""
import math
import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets

SKY, GROUND, LINE = QtGui.QColor('#3d8fd6'), QtGui.QColor('#8a5a2b'), QtGui.QColor('#ffffff')
BODY, WHEEL, ACC = QtGui.QColor('#5d6d7e'), QtGui.QColor('#222222'), QtGui.QColor('#f39c12')
C_ROLL, C_PITCH, C_YAW = QtGui.QColor('#e74c3c'), QtGui.QColor('#27ae60'), QtGui.QColor('#2980b9')
HIST_S = 10.0


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class _Pane(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        self.title = title
        self.att = None          # (roll, pitch, yaw) [deg], None = 데이터 없음
        self.setMinimumSize(220, 220)

    def set_att(self, att):
        self.att = att
        self.update()

    def _frame(self, p):
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#1b2631'))
        p.setPen(QtGui.QColor('#d5dbdb'))
        f = p.font()
        f.setBold(True)
        p.setFont(f)
        p.drawText(self.rect().adjusted(8, 4, -8, -4), QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft, self.title)
        f.setBold(False)
        p.setFont(f)

    def _no_data(self, p):
        p.setPen(QtGui.QColor('#e74c3c'))
        p.drawText(self.rect(), QtCore.Qt.AlignCenter, '자세 안 들어옴\n(/controller/state)')

    def _value(self, p, text, color):
        """아래 큰 숫자. 칸이 좁으면 (GUI 아래 줄) 글씨를 줄이고, 그래도 안 들어가면 괄호 설명을 뺀다."""
        f = p.font()
        f.setBold(True)
        base = f.pointSizeF() * 1.4
        room = self.width() - 12
        for t in (text, text.split('  (')[0]):
            for k in (1.0, 0.85, 0.72):
                f.setPointSizeF(base * k)
                if QtGui.QFontMetrics(f).horizontalAdvance(t) <= room:
                    break
            if QtGui.QFontMetrics(f).horizontalAdvance(t) <= room:
                break
        p.setFont(f)
        p.setPen(color)
        p.drawText(self.rect().adjusted(6, 4, -6, -6), QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, t)


class Horizon(_Pane):
    """인공 수평선: 기체 기호 고정, 하늘·땅이 피치로 오르내리고 롤로 기운다."""

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 6
        c = QtCore.QPointF(w / 2, h / 2)
        p.fillRect(self.rect(), QtGui.QColor('#1b2631'))
        if self.att is None:
            self._frame(p)
            self._no_data(p)
            return
        roll, pitch, _ = self.att
        k = r / 30.0                             # 화면 반지름 = 피치 30°
        clip = QtGui.QPainterPath()
        clip.addEllipse(c, r, r)
        p.setClipPath(clip)
        p.save()
        p.translate(c)
        p.rotate(-roll)                          # 오른쪽 내려가면 (롤 +) 수평선은 반시계로
        p.translate(0, -pitch * k)               # 앞 숙이면 (피치 +) 땅이 올라온다
        big = 4 * r
        p.fillRect(QtCore.QRectF(-big, -big, 2 * big, big), SKY)
        p.fillRect(QtCore.QRectF(-big, 0, 2 * big, big), GROUND)
        p.setPen(QtGui.QPen(LINE, 2))
        p.drawLine(QtCore.QPointF(-big, 0), QtCore.QPointF(big, 0))
        p.setPen(QtGui.QPen(LINE, 1.5))
        for d in range(-60, 61, 5):              # 피치 사다리: 숫자 = 그 선이 가운데 오는 피치 (위 = 젖힘 -, 아래 = 숙임 +)
            if d == 0:
                continue
            y = d * k
            half = r * (0.28 if d % 10 == 0 else 0.14)
            p.drawLine(QtCore.QPointF(-half, y), QtCore.QPointF(half, y))
            if d % 10 == 0:
                p.drawText(QtCore.QRectF(half + 4, y - 8, 40, 16), QtCore.Qt.AlignVCenter, f'{d:+d}')
        p.restore()
        p.setClipping(False)
        p.save()                                 # 롤 눈금 (위쪽 호) + 롤 지침
        p.translate(c)
        p.setPen(QtGui.QPen(LINE, 1.5))
        for d in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            p.save()
            p.rotate(d)
            p.drawLine(QtCore.QPointF(0, -r), QtCore.QPointF(0, -r + (12 if d % 30 == 0 else 7)))
            p.restore()
        p.rotate(-roll)
        p.setBrush(ACC)
        p.setPen(QtCore.Qt.NoPen)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(0, -r + 13), QtCore.QPointF(-7, -r + 25), QtCore.QPointF(7, -r + 25)]))
        p.restore()
        p.setPen(QtGui.QPen(ACC, 4))             # 기체 기호 (고정)
        p.drawLine(QtCore.QPointF(c.x() - r * 0.55, c.y()), QtCore.QPointF(c.x() - r * 0.18, c.y()))
        p.drawLine(QtCore.QPointF(c.x() + r * 0.18, c.y()), QtCore.QPointF(c.x() + r * 0.55, c.y()))
        p.drawEllipse(c, 3, 3)
        p.setPen(QtGui.QPen(QtGui.QColor('#d5dbdb'), 2))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(c, r, r)
        p.setPen(QtGui.QColor('#d5dbdb'))
        p.drawText(self.rect().adjusted(8, 4, -8, -4), QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft, self.title)


class Compass(_Pane):
    """위에서 본 로봇 (요). 눈금은 고정, 로봇이 돈다. 0° = '요 0 맞추기' 한 방향."""

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        self._frame(p)
        if self.att is None:
            self._no_data(p)
            return
        yaw = self.att[2]
        if yaw is None:
            p.setPen(QtGui.QColor('#e67e22'))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, '요 안 들어옴\n(/odom)')
            return
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 26
        c = QtCore.QPointF(w / 2, h / 2 - 6)
        p.setPen(QtGui.QPen(QtGui.QColor('#d5dbdb'), 2))
        p.drawEllipse(c, r, r)
        for d in range(0, 360, 15):
            p.save()
            p.translate(c)
            p.rotate(-d)                         # 요 + = 반시계 → 눈금도 반시계로 +
            p.drawLine(QtCore.QPointF(0, -r), QtCore.QPointF(0, -r + (12 if d % 90 == 0 else 6)))
            p.restore()
            if d % 90 == 0:                      # 숫자는 똑바로
                a = math.radians(d)
                q = QtCore.QPointF(c.x() - (r - 24) * math.sin(a), c.y() - (r - 24) * math.cos(a))
                p.drawText(QtCore.QRectF(q.x() - 20, q.y() - 8, 40, 16), QtCore.Qt.AlignCenter,
                           {0: '0', 90: '+90', 180: '180', 270: '-90'}[d])
        p.save()
        p.translate(c)
        p.rotate(-yaw)
        s = r * 0.5                              # 로봇 윗모습: 몸통 + 양쪽 바퀴, 앞 = 화살표
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(WHEEL)
        p.drawRoundedRect(QtCore.QRectF(-s * 0.95, -s * 0.35, s * 0.25, s * 0.7), 3, 3)
        p.drawRoundedRect(QtCore.QRectF(s * 0.70, -s * 0.35, s * 0.25, s * 0.7), 3, 3)
        p.setBrush(BODY)
        p.drawRoundedRect(QtCore.QRectF(-s * 0.65, -s * 0.45, s * 1.3, s * 0.9), 5, 5)
        p.setBrush(ACC)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(0, -s * 1.25), QtCore.QPointF(-s * 0.3, -s * 0.55),
                                       QtCore.QPointF(s * 0.3, -s * 0.55)]))
        p.restore()
        self._value(p, f'요 {yaw:+.1f}°  ' + ('(왼쪽으로 돎)' if yaw > 0.5 else '(오른쪽으로 돎)' if yaw < -0.5 else ''), C_YAW)


class SideView(_Pane):
    """옆에서 본 로봇 (피치) — 오른쪽 옆에서 봄, 앞 = 화면 오른쪽. 바퀴 축을 중심으로 기운다."""

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        self._frame(p)
        if self.att is None:
            self._no_data(p)
            return
        pitch = self.att[1]
        w, h = self.width(), self.height()
        s = min(w, h) / 220.0
        ground = h - 44
        wr = 22 * s
        axle = QtCore.QPointF(w / 2, ground - wr)
        p.setPen(QtGui.QPen(QtGui.QColor('#7f8c8d'), 2))
        p.drawLine(QtCore.QPointF(10, ground), QtCore.QPointF(w - 10, ground))
        p.setPen(QtGui.QPen(QtGui.QColor('#566573'), 1, QtCore.Qt.DashLine))
        p.drawLine(axle, QtCore.QPointF(axle.x(), axle.y() - 120 * s))      # 수직 기준
        p.save()
        p.translate(axle)
        p.rotate(pitch)                          # 앞 숙임 (+) = 시계방향 (앞이 내려감)
        p.setPen(QtGui.QPen(QtGui.QColor('#aab7b8'), 5 * s))
        p.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(0, -55 * s))         # 다리
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(BODY)
        p.drawRoundedRect(QtCore.QRectF(-30 * s, -110 * s, 60 * s, 55 * s), 6, 6)
        p.setBrush(ACC)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(30 * s, -92 * s), QtCore.QPointF(44 * s, -82.5 * s),
                                       QtCore.QPointF(30 * s, -73 * s)]))     # 앞
        p.restore()
        p.setPen(QtGui.QPen(QtGui.QColor('#aab7b8'), 2))
        p.setBrush(WHEEL)
        p.drawEllipse(axle, wr, wr)
        p.setPen(QtGui.QColor('#aab7b8'))
        p.drawText(QtCore.QRectF(w - 60, ground - 18, 50, 16), QtCore.Qt.AlignRight, '앞 →')
        self._value(p, f'피치 {pitch:+.1f}°  ' + ('(앞 숙임)' if pitch > 0.5 else '(뒤로 젖힘)' if pitch < -0.5 else ''), C_PITCH)


class RearView(_Pane):
    """뒤에서 본 로봇 (롤) — 화면 오른쪽 = 로봇 오른쪽. 바닥 가운데를 중심으로 기운다."""

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        self._frame(p)
        if self.att is None:
            self._no_data(p)
            return
        roll = self.att[0]
        w, h = self.width(), self.height()
        s = min(w, h) / 220.0
        ground = h - 44
        base = QtCore.QPointF(w / 2, ground)
        piv = 56 * s if roll > 0 else -56 * s     # 내려간 쪽 바퀴의 바닥을 축으로 (바퀴가 땅에 안 박히게)
        p.setPen(QtGui.QPen(QtGui.QColor('#7f8c8d'), 2))
        p.drawLine(QtCore.QPointF(10, ground), QtCore.QPointF(w - 10, ground))
        p.setPen(QtGui.QPen(QtGui.QColor('#566573'), 1, QtCore.Qt.DashLine))
        p.drawLine(base, QtCore.QPointF(base.x(), base.y() - 140 * s))
        p.save()
        p.translate(base.x() + piv, base.y())
        p.rotate(roll)                           # 오른쪽 내려감 (+) = 시계방향
        p.translate(-piv, 0)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(WHEEL)
        for x in (-62, 50):
            p.drawRoundedRect(QtCore.QRectF(x * s, -44 * s, 12 * s, 44 * s), 3, 3)
        p.setPen(QtGui.QPen(QtGui.QColor('#aab7b8'), 5 * s))
        p.drawLine(QtCore.QPointF(-44 * s, -22 * s), QtCore.QPointF(-30 * s, -70 * s))
        p.drawLine(QtCore.QPointF(44 * s, -22 * s), QtCore.QPointF(30 * s, -70 * s))
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(BODY)
        p.drawRoundedRect(QtCore.QRectF(-40 * s, -125 * s, 80 * s, 55 * s), 6, 6)
        p.setPen(QtGui.QPen(LINE, 2))
        p.drawLine(QtCore.QPointF(-40 * s, -97 * s), QtCore.QPointF(40 * s, -97 * s))   # 몸 수평선
        p.restore()
        p.setPen(QtGui.QColor('#aab7b8'))
        p.drawText(QtCore.QRectF(10, ground - 18, 50, 16), QtCore.Qt.AlignLeft, '왼')
        p.drawText(QtCore.QRectF(w - 60, ground - 18, 50, 16), QtCore.Qt.AlignRight, '오른')
        self._value(p, f'롤 {roll:+.1f}°  ' + ('(오른쪽 내려감)' if roll > 0.5 else '(왼쪽 내려감)' if roll < -0.5 else ''), C_ROLL)


class History(QtWidgets.QWidget):
    """지난 10 초 롤·피치 (±range) 와 요 변화."""

    def __init__(self):
        super().__init__()
        self.buf = deque()                       # (t, roll, pitch, yaw)
        self.setMinimumSize(120, 120)

    def add(self, t, att):
        self.buf.append((t, *att))
        while self.buf and t - self.buf[0][0] > HIST_S:
            self.buf.popleft()
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#1b2631'))
        w, h = self.width(), self.height()
        L, R, T, B = 44, 10, 20, 18
        pw, ph = w - L - R, h - T - B
        rng = 10.0                               # ±10° 부터, 넘으면 넓힌다
        for _, r_, p_, _y in self.buf:
            rng = max(rng, abs(r_) * 1.1, abs(p_) * 1.1)
        rng = min(90.0, math.ceil(rng / 5) * 5)
        ymap = lambda v: T + ph / 2 - v / rng * ph / 2   # noqa: E731
        p.setPen(QtGui.QColor('#34495e'))
        for v in (-rng, -rng / 2, 0, rng / 2, rng):
            p.drawLine(QtCore.QPointF(L, ymap(v)), QtCore.QPointF(w - R, ymap(v)))
        p.setPen(QtGui.QColor('#aab7b8'))
        for v in (-rng, 0, rng):
            p.drawText(QtCore.QRectF(0, ymap(v) - 8, L - 4, 16), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, f'{v:+.0f}°')
        fm = p.fontMetrics()
        p.drawText(QtCore.QPointF(L, 14), '지난 10 초:')
        x0 = L + fm.horizontalAdvance('지난 10 초:') + 10
        for name, col in (('롤', C_ROLL), ('피치', C_PITCH), ('요 변화', C_YAW)):
            p.setPen(col)
            p.drawText(QtCore.QPointF(x0, 14), name)
            x0 += fm.horizontalAdvance(name) + 14
        if len(self.buf) < 2:
            return
        t1 = self.buf[-1][0]
        xmap = lambda t: L + pw * (1 - (t1 - t) / HIST_S)   # noqa: E731
        y0 = next((e[3] for e in self.buf if e[3] is not None), None)
        for idx, col, f in ((1, C_ROLL, None), (2, C_PITCH, None), (3, C_YAW, lambda v: wrap180(v - y0))):
            path = QtGui.QPainterPath()
            prev = None
            for e in self.buf:
                if e[idx] is None:
                    prev = None
                    continue
                v = e[idx] if f is None else f(e[idx])
                pt = QtCore.QPointF(xmap(e[0]), ymap(max(-rng, min(rng, v))))
                # 처음·빈칸 뒤·요가 ±180° 를 넘어갈 때는 선을 끊는다 (세로선 방지)
                path.moveTo(pt) if prev is None or abs(v - prev) > 90 else path.lineTo(pt)
                prev = v
            p.setPen(QtGui.QPen(col, 1.6))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawPath(path)


class AttitudeView(QtWidgets.QWidget):
    """ros.attitude() 의 최신 (t, roll°, pitch°, yaw° | None, wz | None) 를 30 Hz 로 읽어 그린다.
    보이는 동안만 /odom 구독 (ros.att_on 은 개수를 센다).
    compact = 관제 GUI 아래 한 줄 (자세계 · 위 · 옆 · 뒤 · 기록), 아니면 큰 창 배치. 요 0 기준은 ros.yaw0 하나를 같이 쓴다."""

    def __init__(self, ros, compact=False, parent=None, flags=QtCore.Qt.Widget):
        super().__init__(parent, flags)
        self.ros = ros
        self.on = False
        self.raw_yaw = None
        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0) if compact else None
        self.hz = Horizon('자세계')
        self.top = Compass('위에서 — 요')
        self.side = SideView('옆에서 — 피치')
        self.rear = RearView('뒤에서 — 롤')
        self.hist = History()
        if compact:
            for c, v in enumerate((self.hz, self.top, self.side, self.rear)):
                v.setMinimumSize(100, 120)       # 반쪽 화면 스냅에서도 들어가게
                g.addWidget(v, 0, c)
                g.setColumnStretch(c, 2)
            g.addWidget(self.hist, 0, 4)
            g.setColumnStretch(4, 3)
            g.setRowStretch(0, 1)
            row = 1
        else:
            self.hz.title = '자세계 (앞에서 본 시야)'
            g.addWidget(self.hz, 0, 0)
            g.addWidget(self.top, 0, 1)
            g.addWidget(self.side, 1, 0)
            g.addWidget(self.rear, 1, 1)
            g.addWidget(self.hist, 2, 0, 1, 2)
            g.setRowStretch(0, 3)
            g.setRowStretch(1, 2)
            g.setRowStretch(2, 1)
            row = 3
        h = QtWidgets.QHBoxLayout()
        self.info = QtWidgets.QLabel('—')
        self.info.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)   # 좁으면 잘림 (창 폭을 안 막음)
        self.info.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        h.addWidget(self.info, 1)
        self.buttons = h
        b = QtWidgets.QPushButton('요 0 맞추기')
        b.setToolTip('지금 로봇이 보는 방향을 0° 로 (odom 요는 기준이 임의이고 천천히 흐른다)')
        b.clicked.connect(self.zero_yaw)
        h.addWidget(b)
        g.addLayout(h, row, 0, 1, 5 if compact else 2)
        self.timer = QtCore.QTimer(self, interval=66)        # 15 Hz — 30 Hz 는 노트북 CPU 를 두 배로 씀
        self.timer.timeout.connect(self._tick)

    def zero_yaw(self):
        if self.raw_yaw is not None:
            self.ros.yaw0 = self.raw_yaw

    def _set_on(self, on):
        if on != self.on:
            self.on = on
            self.ros.att_on(on)
            self.timer.start() if on else self.timer.stop()

    def showEvent(self, e):
        self._set_on(True)
        super().showEvent(e)

    def hideEvent(self, e):
        self._set_on(False)
        super().hideEvent(e)

    def closeEvent(self, e):
        self._set_on(False)
        super().closeEvent(e)

    def _tick(self):
        m = self.ros.attitude()
        if m is None or time.monotonic() - m[0] > 0.5:
            for v in (self.hz, self.top, self.side, self.rear):
                v.set_att(None)
            self.info.setText('자세 안 들어옴 (/controller/state) — 로봇 망 (ROBOT) 에 붙었는지, gen2-balance 가 도는지 확인')
            return
        t, roll, pitch, yaw, wz = m
        self.raw_yaw = yaw
        if yaw is not None and getattr(self.ros, 'yaw0', None) is None:
            self.ros.yaw0 = yaw
        rel = None if yaw is None else wrap180(yaw - self.ros.yaw0)
        att = (roll, pitch, rel)
        for v in (self.hz, self.top, self.side, self.rear):
            v.set_att(att)
        self.hist.add(t, att)
        ys = '—' if rel is None else f'{rel:+6.1f}°'
        ws = '' if wz is None else f'   회전 {math.degrees(wz):+6.1f} °/s'
        self.info.setText(f'롤 {roll:+6.1f}°  피치 {pitch:+6.1f}°  요 {ys}{ws}   (요는 서 있을 때만 갱신)')


class AttitudeWindow(AttitudeView):
    """따로 뜨는 큰 자세 창 (관제 GUI 아래 줄의 '크게 보기')."""

    def __init__(self, ros):
        super().__init__(ros, compact=False, flags=QtCore.Qt.Window)
        self.setWindowTitle('C-WANG 로봇 자세 (롤 · 피치 · 요)')
        self.resize(900, 720)
