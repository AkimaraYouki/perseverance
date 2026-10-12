from: desktop
re: 2026-10-12_robot-roles.md
status: 반영 완료

1. **gesture_node 데스크톱**: ~/gen2_desk/venv_vision (python3.10 + system-site-packages, mediapipe 1.1.0, numpy 1.26 / opencv-contrib 4.10 로 고정 —
   numpy 2 는 시스템 matplotlib 과 충돌), 모델 ~/gen2_ws/models/gesture_recognizer.task. `cw gesture` (process_hz 15) 로 실행 확인.
2. **SLAM 데스크톱**: gen2_bringup 데스크톱 빌드, `cw slam start` = 데스크톱 `ros2 launch gen2_bringup slam.launch.py`.
   로봇에서 slam_toolbox 가 돌고 있으면 시작을 거부 (지도·TF 충돌 방지). 지도 저장은 데스크톱 ~/maps.
3. **대시보드 수집기**: 젯슨 쪽은 이제 ROS 없이 SysInfo 만 1 Hz (CPU 26 % → 1 % 확인). /controller/state·/diagnostics·/odom 은 데스크톱이 직접 구독.
   옛 수집기 (pid 6401, SIGTERM 무시) 는 kill -9 로 정리함.
4. 데스크톱 RViz (tools/desktop/gen2_desktop.rviz): 전역·지역 코스트맵, /plan, /footprint, /people/legs 표시 추가. HUD (`cw hud`) 는 YOLO-pose 키포인트·대상·다리·상태.
5. `cw up` = 추종 스택 (로봇 run_follow.sh) + 패드 + 손짓 + RViz + 상태판 + HUD.
참고 (10-12 23:29 상태판): 컴퓨터 배터리 10.8 V (셀 3.61 V, 11 %), MAXN_SUPER, 모터 피드백 stale·disarmed·pitch −62° (넘어져 있는 상태였음).
