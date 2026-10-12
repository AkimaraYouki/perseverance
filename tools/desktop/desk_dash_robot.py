#!/usr/bin/env python3
"""로봇 쪽 수집기 (wifi -r 가 ssh 로 띄운다): LCD 와 같은 정보 + 제어 상태를 1 초마다 JSON 한 줄로 stdout.

구독: /diagnostics (전원·모터·센서, LCD 와 같은 출처), /robot/state, /controller/state (100 Hz: 속도·자세·토크), /odom (회전·주행거리).
시스템 정보는 LCD 노드와 같은 gen2_status_display.sysinfo.SysInfo (CPU·온도·RAM·Wi-Fi·SSH).
"""
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from gen2_msgs.msg import ControllerState
from gen2_status_display.sysinfo import SysInfo
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

FRESH = 3.0   # 진단이 이보다 오래되면 '오래됨'


def _f(kv, k):
    try:
        v = float(kv.get(k))
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


class Dash(Node):
    def __init__(self):
        super().__init__('desk_dash')
        self.diag, self.state, self.cs, self.od = {}, None, None, None
        self.prev = None          # (stamp s, v)
        self.acc_f = 0.0          # 바퀴축 가속 [m/s^2] (속도 미분, 저역통과)
        self.win = dict(vmax=0.0, amax=0.0, pmax=0.0)    # 지난 1 초 최대
        self.sys = SysInfo()
        self.create_subscription(DiagnosticArray, 'diagnostics', self._diag, 20)
        self.create_subscription(String, 'robot/state', lambda m: setattr(self, 'state', (time.monotonic(), m.data)), 10)
        self.create_subscription(ControllerState, 'controller/state', self._cs, 50)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'od', (time.monotonic(), m)), 10)
        self.create_timer(1.0, self._tick)

    def _diag(self, m):
        now = time.monotonic()
        for s in m.status:
            self.diag[s.name.split(': ', 1)[-1]] = (now, int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level),
                                                    s.message, {kv.key: kv.value for kv in s.values})

    def _cs(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.prev is not None and t > self.prev[0]:
            a = (m.v - self.prev[1]) / (t - self.prev[0])
            self.acc_f += 0.1 * (a - self.acc_f)          # 100 Hz 에서 ~1.7 Hz 저역통과
        self.prev = (t, m.v)
        w = self.win
        w['vmax'] = max(w['vmax'], abs(m.v)); w['amax'] = max(w['amax'], abs(self.acc_f))
        w['pmax'] = max(w['pmax'], abs(m.pitch))
        self.cs = (time.monotonic(), m)

    def _get(self, name):
        d = self.diag.get(name)
        if d is None:
            return None
        t, lvl, msg, kv = d
        return dict(age=time.monotonic() - t, level=lvl, msg=msg, kv=kv)

    def _tick(self):
        now = time.monotonic()
        out = dict(t=time.time(), sys=self.sys.sample())
        out['state'] = self.state[1] if self.state and now - self.state[0] < 2.0 else None
        for k in ('power motor', 'power compute'):
            d = self._get('sensor_hub: ' + k)
            if d:
                kv = d['kv']
                out[k] = dict(age=d['age'], level=d['level'], msg=d['msg'], v=_f(kv, 'voltage_v'), a=_f(kv, 'current_a'),
                              w=_f(kv, 'power_w'), soc=_f(kv, 'soc_percent'), run=_f(kv, 'runtime_min'),
                              cells=int(_f(kv, 'cells') or 0), cell_v=_f(kv, 'cell_voltage_v'), present=kv.get('present') == 'True')
        out['motors'] = []
        for name in ('leg_l', 'leg_r', 'wheel_l', 'wheel_r'):
            d = self._get('motor: ' + name)
            if d:
                kv = d['kv']
                out['motors'].append(dict(name=name, age=d['age'], level=d['level'], msg=d['msg'], temp=_f(kv, 'temperature_c'),
                                          cur=_f(kv, 'current_a'), pos=_f(kv, 'position_deg'), err=int(_f(kv, 'error_code') or 0)))
        dev = {}
        for key, name in (('can', 'can: bus'), ('imu', 'iahrs'), ('lidar', 'scan'), ('camera', 'stream'), ('hub', 'sensor_hub: link'),
                          ('gnss', 'sensor_hub: gnss'), ('mag', 'sensor_hub: magnetometer')):
            d = self._get(name)
            if d:
                kv = d['kv']
                dev[key] = dict(age=d['age'], level=d['level'], msg=d['msg'],
                                rate=_f(kv, 'frame_rate_hz') or _f(kv, 'rate_hz') or _f(kv, 'sample_rate_hz'),
                                fix=int(_f(kv, 'fix_type') or 0), sv=int(_f(kv, 'num_sv') or 0))
        out['dev'] = dev
        if self.cs and now - self.cs[0] < 1.0:
            m = self.cs[1]
            out['ctrl'] = dict(mode=m.mode, fault=m.fault, v=m.v, v_ref=m.v_ref, vx=m.vx_cmd, wz=m.wz_cmd, acc=self.acc_f,
                               pitch=math.degrees(m.pitch), roll=math.degrees(m.roll), th_bias=math.degrees(m.th_bias),
                               h=list(m.h), wheel_tau=list(m.wheel_tau_cmd), hip_tau=list(m.hip_tau_cmd), lifted=m.lifted,
                               imu_age=m.imu_age_ms, loop_max=m.loop_dt_max_ms, overruns=m.overruns,
                               vmax=self.win['vmax'], amax=self.win['amax'], pmax=math.degrees(self.win['pmax']))
        if self.od and now - self.od[0] < 1.0:
            o = self.od[1]
            q = o.pose.pose.orientation
            out['odom'] = dict(x=o.pose.pose.position.x, y=o.pose.pose.position.y, wz=o.twist.twist.angular.z,
                               yaw=math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        self.win = dict(vmax=0.0, amax=0.0, pmax=0.0)
        print(json.dumps(out, default=str), flush=True)


def main():
    rclpy.init()
    n = Dash()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
