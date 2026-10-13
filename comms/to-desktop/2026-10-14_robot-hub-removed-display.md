from: robot
re: 사용자 "허브랑 관련된 기능 다 뺌 (GPS, 지자기, 전원 전류센서) … 디스플레이 레이아웃 변경" (2026-10-14)
status: 정보 (반영 완료, 커밋)

## 하드웨어
- ESP32 센서 허브 (GPS NEO-M8N, IST8310 지자기, Holybro RGB/safety LED, PM02 ×2 배터리 전압·전류) **로봇에서 제거**.
  경위: 새 ESP32 로 바꾸고 하네스에 납땜한 뒤 두 보드 모두 USB 인식 실패 (error -71/-110, 붙어도 데이터 0, esptool 응답 없음)
  → 하네스 전원/배선 문제로 추정. 젯슨 USB 허브 **3번 포트 (1-2.3) 도 불량** (아침 IMU 끊김도 이 포트) — 쓰지 말 것.
- 지금 USB: 1번 라이다, 2번 IMU, 4번 (비움 / 옛 허브 꽂혀 있으면 무시됨).
- 펌웨어 쪽 (참고, 지금은 안 씀): firmware/biped_sensor_hub 에 ENABLE_GPS/MAG/RGB = false 스위치, 옛 보드 (…8F:3C) 에 한 번 올라감.

## 소프트웨어 변경
- bench.launch: `hub:=false` 기본 (sensor_hub 노드 안 뜸). 다시 쓰려면 hub:=true.
- **/power/compute, /power/motor, gps/*, 지자기 토픽 없음.** 배터리 % · 전압 · 저전압 경고도 없음 → **배터리는 직접 확인** (모터 배터리 6S 3.5 V/셀).
- LCD: 2. SENSORS = IMU / LIDAR / CAM / DDS, 4. NET & POWER = Wi-Fi · SSH · **PC ~W (추정: 1.08 × VDD_IN + 3.93, Rigol 보정)** · BATT "no sensor" · 가동 시간.
  헤더 상태 = balance_node controller/state 모드 (DISARMED / STAND / BALANCE / FAULT …) — 예전 /robot/state 는 아무도 안 내서 늘 NO CTRL 이었다.
- health_logger: 배터리 칸은 비어 있음 (나머지 그대로).
- 데스크톱 대시보드·GCS GUI 에서 배터리/GPS 표시는 "센서 없음" 으로 바꿔 주세요.
