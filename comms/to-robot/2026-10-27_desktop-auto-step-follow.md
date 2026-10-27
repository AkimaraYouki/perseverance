from: desktop
re: 사용자 2026-10-27 "카메라로 턱 감지해서 턱 모드 들어가는 기능", "0.3 되면 둘이 비교해서 더 잘 되는 쪽으로 노드 수정 요청"
status: 요청 — desktop/camera-follow (5b947a3 이후) 의 노드 갱신 + 시험. 자동 턱은 기본 꺼짐

## 결정: 턱 감지 모드 접근 속도는 지금 그대로 0.14 (balance.yaml retry_approach_v)
시뮬 (실기 보정 플랜트, 2 단 칸 1 m, 2048 판): 접근 0.14 성공 6 cm 82.4 % / 8 cm 74.4 %, 0.3 → 69.0 / 72.4 %, 0.4 → 7.8 / 15.4 % (넘어짐 84–92 %).
Isaac 자동 턱 0.3 다섯 판: 셋 성공 (~23 s), 둘 실패. → 부딪히는 속도는 올리지 않고, 카메라로 느린 구간만 줄임 (0.3 m/s 감속 뒤 0.5 m 앞에서만 0.14).

## 바뀐 것 (데스크톱이 10-27 로봇 설치본에 이미 넣은 것: tracking_core·person_tracker·person_follow_cam (롤 -2.5°), camera_cal.yaml
## — 백업 logs/backup_20261027). 이번 커밋에서 더 바뀐 것:
- step_watch.py (새): 바닥 위 긴 가로 모서리 중 위아래 표면 색이 다른 것 (= 턱 발치, 바닥 이음매·격자는 제외) → 광선.
- auto_step.py (새): FOLLOW 0.7 → 1.5 m 앞 SLOW 0.3 → 0.5 m 앞 (화면에서 사라지면 주행 거리로 기억) STEP = balance/step_mode true,
  wz 0, 전진 0.3 → 착지 뒤에도 턱 감지 모드 유지 (2 단, 사용자 지시) → 마지막 착지 뒤 1.3 m 막힘 없으면 false → FOLLOW.
  카메라가 놓쳐도: 0.15 m/s 이상 명령인데 0.3 s 멈춤 + 라이다 1 m 통로 비어 있음 → STEP (상자는 라이다에 보여 Nav2 회피).
- person_tracker: step/rays (JSON) 2 프레임마다. person_follow_cam: auto_step 파라미터 (기본 false, 균형 중 켜고 끄기 가능),
  balance/step_phase 2→0 = 발사로 봄 (점프 단계 토픽이 없어서 1.2 s 를 공중으로 가정), follow/status 끝에 ' [AUTO <상태> <거리>]'.
- person_follow.py: _ALWAYS 클래스 속성만 (동작 같음).

## 할 일
1. gen2_tools 4 파일 설치 (step_watch, auto_step, person_tracker, person_follow_cam, person_follow) — colcon build 또는 설치본 복사.
2. (권장) ControllerState 에 int8 jump_phase (wb_core 단계) 추가 → 데스크톱이 1.2 s 가정 대신 씀.
3. 시험: 추종 켠 뒤 `ros2 param set /person_follow auto_step true` (또는 GCS 버튼) →
   ① 평지만 추종: [AUTO SLOW] 오탐 빈도 (바닥 이음매·그림자) ② 6 cm 한 단 ③ 대회 2 단. person_tracker.log 의 FPS 가 30 유지되는지.
   bag: /step/rays /follow/status /cmd_vel/auto /balance/step_phase /odom /people/target.
