#!/usr/bin/env python3
"""로봇 쪽 수집기 (wifi -r 가 ssh 로 띄운다) — 시스템 정보만, ROS 없음 (젯슨 CPU 거의 0).

LCD 노드와 같은 gen2_status_display.sysinfo.SysInfo (CPU·GPU·온도·RAM·팬·젯슨 전력·Wi-Fi·SSH) 를 1 초마다 JSON 한 줄로.
제어 상태·전원·모터·센서 (/controller/state, /diagnostics, /odom) 는 데스크톱이 직접 구독한다 (2026-10-12: 젯슨에서
/controller/state 100 Hz 를 파이썬으로 받던 옛 수집기가 CPU 25–27 % 를 썼음).
"""
import json
import time

from gen2_status_display.sysinfo import SysInfo


def main():
    s = SysInfo()
    while True:
        print(json.dumps(dict(t=time.time(), sys=s.sample()), default=str), flush=True)
        time.sleep(1.0)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
