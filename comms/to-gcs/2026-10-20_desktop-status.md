from: desktop
status: 정보 — 10-15 이후 진행 상황

- 로봇: USB 인식 오류·부팅 느림으로 분해 중. 재조립 계획 = IMU 젯슨 UART 직결, 라이다 USB (납땜·고정), GPS·전압/전류는 ESP32 허브 (CAN 후보). 새 부품 질량 반영 예정.
  → 재조립 전까지 GUI 의 배터리·GPS 는 "센서 없음" 그대로, IMU 장치 경로가 /dev/ttyTHS1 로 바뀔 수 있음.
- 점프: CMA-11 + 고관절 상한 12 N·m 실기 성공 ("내 실수 빼면 안 넘어짐"). 15 N·m 대안 묶음 있음 (턱 8–10 cm 용).
- Fold (폰) 를 관제 PC 로 쓰는 이식 작업이 따로 진행됨 (폰 Termux:X11 + Ubuntu/ROS Jazzy, Xbox 패드 → /joy → 원본 pad_teleop).
  데스크톱 오프라인 점검: 단위 테스트 4/4, 입력 손실·화면 이탈 시 /joy 중단, heartbeat 는 원본 0.5 s 타임아웃.
  실기 DDS 수신·SSH·구동은 로봇 재조립 뒤 검증. **노트북·데스크톱·폰 중 한 곳만 관제** (패드 heartbeat·SLAM 중복 금지) 는 그대로.
