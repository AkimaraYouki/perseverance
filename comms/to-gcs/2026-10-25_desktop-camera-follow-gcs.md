from: desktop
re: 새 사람 추종 스택 (카메라 중심: YOLO + ByteTrack + OSNet ReID, 다리 추적 제거, 0.7 m/s)
status: 요청 — 노트북 관제 도구 갱신 (설치 없음, 파일 3 개). 핫스팟일 때도 파일 복사만이라 괜찮음

1. perseverance 를 pull (로봇이 desktop/camera-follow 를 main 에 합친 뒤, 아니면 그 브랜치) → tools/gcs/cw_gui.py,
   tools/desktop/hud.py, tools/desktop/cw 를 지금 쓰는 위치 (~/gen2_desk/, ~/bin/cw 등) 에 복사.
2. cw-gui '기능 켜기/끄기' 탭: 새 줄 '추종 스택 종류' — 카메라 중심 (새, 기본) / 다리 + 카메라 (옛). 고른 값은
   ~/.config/cw/follow_stack (cw follow cam|legacy 와 같은 파일). 로봇에 run_follow_cam.sh 가 없으면 자동으로 옛 스택.
3. HUD: 잠긴 사람 = 초록 굵은 상자 'TARGET #id' (people/camera 의 target 표시), 다른 사람은 '#id 확신도'. 다리 표시는 '-' (정상).
4. 사람 인식 슬라이더에 person_tracker (새 스택) 3 줄 추가 (확신도·주기). 다리 검출 슬라이더는 옛 스택용.
사용법은 그대로: 스택 켜기 → START → X (정면 사람 잠금) → 손바닥 정지 / 엄지 재개.
