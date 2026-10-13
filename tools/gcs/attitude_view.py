"""로봇 자세 창 (아두파일럿 HUD 처럼) — /imu/data 의 roll · pitch · yaw 를 실시간으로.

위에서 내려다보면 기울기 성분이 안 보이니 축마다 따로 그린다:
  자세계 (인공 수평선, 피치 사다리 + 롤 눈금) · 위에서 (요, 나침반) · 옆에서 (피치) · 뒤에서 (롤) · 10 초 기록.
부호 (REP-103 base_link, imu_cal.py 와 같음): 피치 + = 앞 숙임, 롤 + = 오른쪽 내려감, 요 + = 왼쪽으로 돎 (위에서 반시계).
요는 IMU 자체 값이라 기준이 임의이고 천천히 흐른다 → '요 0 맞추기' 로 지금 방향을 0 으로.
"""
import math
import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets

SKY, GROUND, LINE = QtGui.QColor('#3d8fd6'), QtGui.QColor('#8a5a2b'), QtGui.QColor('#ffffff')
BODY, WHEEL, ACC = QtGui.QColor('#5d6d7e'), QtGui.QColor('#222222'), QtGui.QColor('#f39c12')
C_ROLL, C_PITCH, C_YAW = QtGui.QColor('#e74c3c'), QtGui.QColor('#27ae60'), QtGui.QColor('#2980b9')
HIST_S = 10.0


def quat_to_rpy(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


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
        p.drawText(self.rect(), QtCore.Qt.AlignCenter, 'IMU 안 들어옴\n(/imu/data)')

    def _value(self, p, text, color):
        f = p.font()
        f.setPointSizeF(f.pointSizeF() * 1.4)
        f.setBold(True)
        p.setFont(f)
        p.setPen(color)
        p.drawText(self.rect().adjusted(8, 4, -8, -6), QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, text)


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
        self.setMinimumHeight(150)

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
        for name, col in (('롤', C_ROLL), ('피치', C_PITCH), ('요 변화 (10 초 전 기준)', C_YAW)):
            p.setPen(col)
            p.drawText(QtCore.QPointF(x0, 14), name)
            x0 += fm.horizontalAdvance(name) + 14
        if len(self.buf) < 2:
            return
        t1 = self.buf[-1][0]
        xmap = lambda t: L + pw * (1 - (t1 - t) / HIST_S)   # noqa: E731
        y0 = self.buf[0][3]
        for idx, col, f in ((1, C_ROLL, None), (2, C_PITCH, None), (3, C_YAW, lambda v: wrap180(v - y0))):
            path = QtGui.QPainterPath()
            for i, e in enumerate(self.buf):
                v = e[idx] if f is None else f(e[idx])
                pt = QtCore.QPointF(xmap(e[0]), ymap(max(-rng, min(rng, v))))
                path.moveTo(pt) if i == 0 else path.lineTo(pt)
            p.setPen(QtGui.QPen(col, 1.6))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawPath(path)


class AttitudeWindow(QtWidgets.QWidget):
    """ros.imu() 로 최신 (t, w, x, y, z, gx, gy, gz) 를 30 Hz 로 읽어 그린다. 창이 열려 있는 동안만 /imu/data 구독."""

    def __init__(self, ros):
        super().__init__(None, QtCore.Qt.Window)
        self.ros = ros
        self.setWindowTitle('C-WANG 로봇 자세 (롤 · 피치 · 요)')
        self.resize(900, 720)
        self.yaw0 = None
        self.raw_yaw = None
        g = QtWidgets.QGridLayout(self)
        self.hz = Horizon('자세계 (앞에서 본 시야)')
        self.top = Compass('위에서 — 요')
        self.side = SideView('옆에서 — 피치')
        self.rear = RearView('뒤에서 — 롤')
        self.hist = History()
        g.addWidget(self.hz, 0, 0)
        g.addWidget(self.top, 0, 1)
        g.addWidget(self.side, 1, 0)
        g.addWidget(self.rear, 1, 1)
        g.addWidget(self.hist, 2, 0, 1, 2)
        g.setRowStretch(0, 3)
        g.setRowStretch(1, 2)
        g.setRowStretch(2, 1)
        h = QtWidgets.QHBoxLayout()
        self.info = QtWidgets.QLabel('—')
        self.info.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        h.addWidget(self.info, 1)
        b = QtWidgets.QPushButton('요 0 맞추기')
        b.setToolTip('지금 로봇이 보는 방향을 0° 로 (IMU 요는 기준이 임의이고 천천히 흐른다)')
        b.clicked.connect(self.zero_yaw)
        h.addWidget(b)
        g.addLayout(h, 3, 0, 1, 2)
        self.timer = QtCore.QTimer(self, interval=33)
        self.timer.timeout.connect(self._tick)

    def zero_yaw(self):
        if self.raw_yaw is not None:
            self.yaw0 = self.raw_yaw

    def showEvent(self, e):
        self.ros.imu_on(True)
        self.timer.start()
        super().showEvent(e)

    def closeEvent(self, e):
        self.timer.stop()
        self.ros.imu_on(False)
        super().closeEvent(e)

    def _tick(self):
        m = self.ros.imu()
        if m is None or time.monotonic() - m[0] > 0.5:
            for v in (self.hz, self.top, self.side, self.rear):
                v.set_att(None)
            self.info.setText('IMU 안 들어옴 — 로봇 망 (ROBOT) 에 붙었는지, gen2-bench (iahrs) 가 도는지 확인')
            return
        t, w, x, y, z, gx, gy, gz = m
        roll, pitch, yaw = quat_to_rpy(w, x, y, z)
        self.raw_yaw = yaw
        if self.yaw0 is None:
            self.yaw0 = yaw
        att = (roll, pitch, wrap180(yaw - self.yaw0))
        for v in (self.hz, self.top, self.side, self.rear):
            v.set_att(att)
        self.hist.add(t, att)
        self.info.setText(f'롤 {roll:+6.1f}°  피치 {pitch:+6.1f}°  요 {att[2]:+6.1f}° (IMU {yaw:+.1f}°)   '
                          f'각속도 x {math.degrees(gx):+6.1f}  y {math.degrees(gy):+6.1f}  z {math.degrees(gz):+6.1f} °/s')
