#!/usr/bin/env python3
"""wifi -r: 로봇 상태판 (터미널). 로봇 LCD 와 같은 항목 + 제어 상태 (속도·가속·자세·토크) + 데스크톱/로봇 Wi-Fi 신호 %.

제어 상태·전원·모터·센서는 데스크톱이 직접 구독 (/controller/state, /diagnostics, /odom, /robot/state — env.sh 필요),
로봇에서는 desk_dash_robot.py 가 시스템 정보 (CPU·온도·SSH·Wi-Fi) 만 ssh 로 1 초마다 보낸다 (젯슨 CPU 거의 0, 2026-10-12).
Ctrl+C 로 끝 (로봇 쪽도 같이 끝남).
"""
import json
import math
import os
import threading
import re
import subprocess
import sys
import time

ROBOT = 'parksudo@192.168.50.124'
HERE = os.path.dirname(os.path.abspath(__file__))
ENV = 'source ~/gen2_ws/install/setup.bash; '      # gen2_status_display (SysInfo) 만 — ROS 노드는 안 띄움
G, Y, R, D, B, Z = '\033[32m', '\033[33m', '\033[31m', '\033[2m', '\033[1m', '\033[0m'


def pct(dbm):
    return None if dbm is None else max(0, min(100, round(2 * (dbm + 100))))


def bar(p, n=20):
    k = 0 if p is None else round(p / 100 * n)
    return '#' * k + '.' * (n - k)


def lvl(level, txt):
    return (G if level == 0 else Y if level == 1 else R) + txt + Z


def f(v, spec, miss='--'):
    return miss if v is None else format(v, spec)


def heat(t, warn, bad):
    return D if t is None else R if t >= bad else Y if t >= warn else G


def desktop_wifi():
    out = []
    for dev in sorted(os.listdir('/sys/class/net')):
        if not dev.startswith('wl'):
            continue
        s = subprocess.run(['iw', 'dev', dev, 'link'], capture_output=True, text=True).stdout
        m, ssid = re.search(r'signal:\s*(-?\d+)', s), re.search(r'SSID:\s*(.*)', s)
        out.append((dev, float(m.group(1)) if m else None, ssid.group(1).strip() if ssid else None))
    return out


def wifi_line(name, dbm, ssid):
    if dbm is None:
        return f'{name} {R}연결 안 됨{Z}'
    p = pct(dbm)
    c = G if p >= 66 else Y if p >= 40 else R
    return f'{name} {c}{p:3d}%{Z} [{bar(p)}] {dbm:4.0f} dBm  {ssid or ""}'


LIPO_V = (3.30, 3.50, 3.60, 3.70, 3.75, 3.80, 3.85, 3.95, 4.05, 4.20)   # 센서허브 lipo_percentage 와 같은 곡선 (셀 V -> %)
LIPO_S = (0, 5, 10, 20, 30, 40, 50, 70, 85, 100)


def volt_pct(cell_v):
    """셀 전압 기준 % (0 % = 3.30 V/셀 '닳기 직전', 100 % = 4.20 V/셀 완충). 부하 중이면 조금 낮게 나온다."""
    if cell_v is None:
        return None
    if cell_v <= LIPO_V[0]:
        return 0.0
    for i in range(1, len(LIPO_V)):
        if cell_v <= LIPO_V[i]:
            return LIPO_S[i - 1] + (LIPO_S[i] - LIPO_S[i - 1]) * (cell_v - LIPO_V[i - 1]) / (LIPO_V[i] - LIPO_V[i - 1])
    return 100.0


def battery(name, b):
    if not b:
        return f'  {name:<8} {D}데이터 없음{Z}'
    if not b.get('present'):
        return f'  {name:<8} {Y}연결 안 됨{Z}'
    vp = volt_pct(b.get('cell_v'))
    c = D if vp is None else G if vp >= 40 else Y if vp >= 20 else R
    run = b.get('run')
    rt = f'{int(run // 60)}h{int(run % 60):02d}m' if run else '--'
    socs = f'{c}{B}{f(vp, "3.0f")}%{Z} {D}(센서허브 {f(b.get("soc"), ".0f")}%){Z} '
    return (f'  {name:<8} {B}{f(b.get("v"), ".2f")} V{Z} ({b.get("cells")}S {f(b.get("cell_v"), ".2f")} V/셀) {socs}'
            f'{f(b.get("a"), "5.2f")} A {f(b.get("w"), "5.1f")} W  남은 {rt}  {lvl(b["level"], b["msg"])}')


def render(d, conn_age):
    L = []
    L.append(f'{B}PERSEVERANCE 상태판{Z}  {time.strftime("%H:%M:%S")}  {D}(로봇 데이터 {conn_age:.1f} s 전, Ctrl+C 끝){Z}')
    # 1. 통신
    L.append(f'{B}■ 통신{Z}')
    for dev, dbm, ssid in desktop_wifi():
        L.append('  ' + wifi_line('데스크톱', dbm, f'{ssid} ({dev})'))
    sy = d.get('sys', {}) if d else {}
    if d:
        L.append('  ' + wifi_line('로봇    ', sy.get('wifi_dbm'), sy.get('ssid')) + f'  IP {sy.get("wifi_ip") or "--"}')
        L.append(f'  SSH 접속 {B}{sy.get("ssh", 0)}{Z}   ETH {sy.get("eth_ip") or "--"}')
    if not d:
        L.append(f'  {R}로봇 데이터 없음{Z}')
        return '\n'.join(L)
    # 2. 전원
    L.append(f'{B}■ 전원{Z}')
    L.append(battery('모터', d.get('power motor')))
    L.append(battery('컴퓨터', d.get('power compute')))
    L.append(f'  젯슨 {f(sy.get("jetson_w"), ".1f")} W  모드 {sy.get("power_mode") or "--"}')
    # 3. 컴퓨터
    L.append(f'{B}■ 컴퓨터{Z}')
    tc = sy.get('t_cpu')
    L.append(f'  CPU {f(sy.get("cpu"), "3.0f")}%  GPU {f(sy.get("gpu"), "3.0f")}%  온도 {heat(tc, 70, 85)}{f(tc, ".0f")}°C{Z}  '
             f'RAM {f(sy.get("ram_used"), ".1f")}/{f(sy.get("ram_total"), ".1f")} GB  팬 {f(sy.get("fan_rpm"), ".0f")} rpm')
    # 4. 모터
    L.append(f'{B}■ 모터{Z}')
    for m in d.get('motors', []):
        st = lvl(m['level'], m['msg'] if m['age'] < 3 else '오래됨')
        L.append(f'  {m["name"]:<8} {heat(m["temp"], 60, 75)}{f(m["temp"], "3.0f")}°C{Z}  {f(m["cur"], "+6.2f")} A  '
                 f'{f(m["pos"], "+8.1f")}°  {st}' + (f'  {R}오류 {m["err"]}{Z}' if m['err'] else ''))
    # 5. 센서
    L.append(f'{B}■ 센서{Z}')
    names = dict(imu='IMU', lidar='라이다', camera='카메라', hub='센서허브', can='CAN', gnss='GNSS', mag='지자기')
    row = []
    for k in ('imu', 'lidar', 'camera', 'hub', 'can', 'gnss', 'mag'):
        x = d.get('dev', {}).get(k)
        row.append(f'{names[k]} ' + (lvl(x['level'], x['msg'] if x['age'] < 5 else '오래됨') if x else f'{D}없음{Z}'))
    L.append('  ' + '   '.join(row[:4]))
    L.append('  ' + '   '.join(row[4:]))
    # 6. 주행
    L.append(f'{B}■ 주행 / 자세{Z}')
    c = d.get('ctrl')
    if not c:
        L.append(f'  {D}제어 상태 없음 (balance_node 꺼짐?){Z}')
    else:
        mode = c['mode'] + (f' — {c["fault"]}' if c.get('fault') else '') + (' (들림)' if c.get('lifted') else '')
        mc = G if c['mode'] == 'balance' else Y if c['mode'] in ('stand', 'disarmed') else R
        L.append(f'  모드 {mc}{B}{mode}{Z}')
        L.append(f'  속도 {B}{c["v"]:+5.2f} m/s{Z} ({c["v"] * 3.6:+4.1f} km/h)  기준 {c["v_ref"]:+5.2f}  명령 vx {c["vx"]:+5.2f}  '
                 f'| 1 s 최대 {c["vmax"]:.2f} m/s')
        o = d.get('odom') or {}
        L.append(f'  가속 {B}{c["acc"]:+5.2f} m/s²{Z}  (1 s 최대 {c["amax"]:.2f})   회전 {f(o.get("wz"), "+5.2f")} rad/s  명령 wz {c["wz"]:+5.2f}')
        L.append(f'  pitch {c["pitch"]:+5.1f}° (1 s 최대 {c["pmax"]:.1f})  roll {c["roll"]:+5.1f}°  균형점 보정 {c["th_bias"]:+5.2f}°')
        L.append(f'  다리 L {c["h"][0] * 1000:5.1f} R {c["h"][1] * 1000:5.1f} mm   고관절 토크 {c["hip_tau"][0]:+5.2f} {c["hip_tau"][1]:+5.2f}  '
                 f'바퀴 토크 {c["wheel_tau"][0]:+5.2f} {c["wheel_tau"][1]:+5.2f} N·m')
        L.append(f'  위치 x {f(o.get("x"), "+6.2f")} y {f(o.get("y"), "+6.2f")} m  방향 {f(o.get("yaw"), "+6.1f")}°   '
                 f'{D}IMU 나이 {c["imu_age"]:.1f} ms  루프 최대 {c["loop_max"]:.1f} ms  넘침 {c["overruns"]}{Z}')
    return '\n'.join(L)


class Collector:
    """데스크톱에서 로봇 토픽을 직접 구독해 상태판용 dict 를 만든다 (예전 로봇 쪽 수집기와 같은 항목·계산)."""

    def __init__(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from gen2_msgs.msg import ControllerState
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile
        from std_msgs.msg import String
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C·종료 신호는 이 스크립트가 처리 (rclpy 가 삼키지 않게)
        self.rclpy = rclpy
        self.node = rclpy.create_node('desk_dash')
        self.diag, self.state, self.cs, self.od = {}, None, None, None
        self.prev, self.acc_f = None, 0.0
        self.win = dict(vmax=0.0, amax=0.0, pmax=0.0)
        self.lock = threading.Lock()
        q1 = QoSProfile(depth=1)
        self.node.create_subscription(DiagnosticArray, 'diagnostics', self._diag, 20)
        self.node.create_subscription(String, 'robot/state', lambda m: setattr(self, 'state', (time.monotonic(), m.data)), 5)
        self.node.create_subscription(ControllerState, 'controller/state', self._cs, q1)
        self.node.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'od', (time.monotonic(), m)), q1)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _diag(self, m):
        now = time.monotonic()
        with self.lock:
            for s in m.status:
                lv = int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level)
                self.diag[s.name.split(': ', 1)[-1]] = (now, lv, s.message, {kv.key: kv.value for kv in s.values})

    def _cs(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.prev is not None and t > self.prev[0]:
            self.acc_f += 0.1 * ((m.v - self.prev[1]) / (t - self.prev[0]) - self.acc_f)
        self.prev = (t, m.v)
        w = self.win
        w['vmax'] = max(w['vmax'], abs(m.v)); w['amax'] = max(w['amax'], abs(self.acc_f)); w['pmax'] = max(w['pmax'], abs(m.pitch))
        self.cs = (time.monotonic(), m)

    def _get(self, name):
        with self.lock:
            d = self.diag.get(name)
        return None if d is None else dict(age=time.monotonic() - d[0], level=d[1], msg=d[2], kv=d[3])

    def snapshot(self):
        def fv(kv, k):
            try:
                v = float(kv.get(k)); return v if math.isfinite(v) else None
            except (TypeError, ValueError):
                return None
        now = time.monotonic()
        out = dict(state=self.state[1] if self.state and now - self.state[0] < 2.0 else None)
        for k in ('power motor', 'power compute'):
            d = self._get('sensor_hub: ' + k)
            if d:
                kv = d['kv']
                out[k] = dict(age=d['age'], level=d['level'], msg=d['msg'], v=fv(kv, 'voltage_v'), a=fv(kv, 'current_a'),
                              w=fv(kv, 'power_w'), soc=fv(kv, 'soc_percent'), run=fv(kv, 'runtime_min'),
                              cells=int(fv(kv, 'cells') or 0), cell_v=fv(kv, 'cell_voltage_v'), present=kv.get('present') == 'True')
        out['motors'] = []
        for name in ('leg_l', 'leg_r', 'wheel_l', 'wheel_r'):
            d = self._get('motor: ' + name)
            if d:
                kv = d['kv']
                out['motors'].append(dict(name=name, age=d['age'], level=d['level'], msg=d['msg'], temp=fv(kv, 'temperature_c'),
                                          cur=fv(kv, 'current_a'), pos=fv(kv, 'position_deg'), err=int(fv(kv, 'error_code') or 0)))
        dev = {}
        for key, name in (('can', 'can: bus'), ('imu', 'iahrs'), ('lidar', 'scan'), ('camera', 'stream'), ('hub', 'sensor_hub: link'),
                          ('gnss', 'sensor_hub: gnss'), ('mag', 'sensor_hub: magnetometer')):
            d = self._get(name)
            if d:
                dev[key] = dict(age=d['age'], level=d['level'], msg=d['msg'])
        out['dev'] = dev
        if self.cs and now - self.cs[0] < 1.0:
            m = self.cs[1]
            out['ctrl'] = dict(mode=m.mode, fault=m.fault, v=m.v, v_ref=m.v_ref, vx=m.vx_cmd, wz=m.wz_cmd, acc=self.acc_f,
                               pitch=math.degrees(m.pitch), roll=math.degrees(m.roll), th_bias=math.degrees(m.th_bias),
                               h=list(m.h), wheel_tau=list(m.wheel_tau_cmd), hip_tau=list(m.hip_tau_cmd), lifted=m.lifted,
                               imu_age=m.imu_age_ms, loop_max=m.loop_dt_max_ms, overruns=m.overruns,
                               vmax=self.win['vmax'], amax=self.win['amax'], pmax=math.degrees(self.win['pmax']))
        if self.od and now - self.od[0] < 1.0:
            o = self.od[1]; q = o.pose.pose.orientation
            out['odom'] = dict(x=o.pose.pose.position.x, y=o.pose.pose.position.y, wz=o.twist.twist.angular.z,
                               yaw=math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        self.win = dict(vmax=0.0, amax=0.0, pmax=0.0)
        return out


def main():
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    col = Collector()
    subprocess.run(['scp', '-q', '-o', 'BatchMode=yes', os.path.join(HERE, 'desk_dash_robot.py'), f'{ROBOT}:.cache/desk_dash_robot.py'], check=True)
    p = subprocess.Popen(['ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=2', ROBOT,
                         ENV + 'exec python3 -u ~/.cache/desk_dash_robot.py'],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.PIPE, text=True, bufsize=1)
    os.set_blocking(p.stdout.fileno(), False)
    sysd, sys_t, buf = None, None, ''
    try:
        while True:
            try:
                chunk = p.stdout.read() or ''
            except (BlockingIOError, TypeError):
                chunk = ''
            buf += chunk
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if line.startswith('{'):
                    try:
                        sysd, sys_t = json.loads(line).get('sys'), time.time()
                    except json.JSONDecodeError:
                        pass
            d = col.snapshot()
            age = time.time() - sys_t if sys_t else 99.0
            d['sys'] = sysd if age < 5 else {}
            alive = bool(d.get('ctrl') or d.get('dev') or d['sys'])
            sys.stdout.write('\033[H\033[2J' + render(d if alive else None, age) + '\n')
            sys.stdout.flush()
            if p.poll() is not None and not d.get('dev'):
                print(f'{R}로봇 연결 끊김{Z}')
                return
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        p.terminate()
        sys.stdout.flush()
        os._exit(0)          # rclpy spin 스레드를 기다리지 않고 바로 끝 (C++ terminate 경고 없이)


if __name__ == '__main__':
    main()
