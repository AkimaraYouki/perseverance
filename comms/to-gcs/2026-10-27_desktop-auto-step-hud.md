from: desktop
re: 사용자 2026-10-27 — 카메라 자동 턱 + HUD 왜곡·기울기 보정 → GCS UI 수정 요청
status: 요청 (설치 없음, desktop/camera-follow 의 파일 복사)

1. HUD (tools/desktop/hud.py → ~/gen2_desk/hud.py) + tools/robot/camera_cal.yaml → ~/gen2_desk/camera_cal.yaml:
   - 체커보드 렌즈 보정으로 광각 왜곡을 펴서 표시 (사람 상자·관절도 옮겨 그림). Tab/u = 원본/보정, [ ] roll, , . pitch, 0 초기화,
     Enter/w = 저장 (~/.config/cw/camera_mount.json, 한글 입력 상태에서도 Enter 로). 데스크톱 값: roll -2.5°, pitch 0.
   - follow/status 끝 ' [AUTO SLOW 1.2m]' 같은 자동 턱 상태가 상단 띠에 그대로 보임.
2. cw-gui '기능 켜기/끄기' 탭에 줄 추가 요청: '자동 턱 (카메라)' 켜기/끄기 →
   ros.set_params('/person_follow', {'auto_step': True/False}) (균형 중에도 됨), 상태 칸에 follow/status 의 [AUTO ...] 부분 표시.
   기본 꺼짐. 파라미터 튜닝 탭에 auto_slow_dist_m (1.5), auto_arm_dist_m (0.5), auto_slow_v (0.3), auto_done_travel_m (1.3) 은 시작 때만 (재시작 필요).
