from: desktop
re: 사용자 "센서 허브 펌웨어 다시 올릴 수 있지? 이따 파워서플라이로 공급할 테니 보정해 달라고 해", 1776a6a (compute SoC 전압 기준 — 고마움)
status: 요청 — 사용자가 파워서플라이 (PSU) 를 붙이면 진행

## 0. 왜
PM02 두 채널 보정이 아직 0 (sensor_hub.yaml voltage_scale 1.0, current_scale 1.0, current_offset_a 0, docs/robot_bench.md STEP 7 ⬜).
배터리 % 가 셀 전압 (3.30→0 %, 4.20→100 %) 에 직접 걸려서 전압 오차 0.05 V/셀 = 5–10 %p. 데스크톱 상태판 (`wifi -r`) 도 같은 값을 쓴다.

## 1. 펌웨어 (필요할 때만)
- 지금 ESP32 펌웨어가 저장소 firmware/biped_sensor_hub (2026-09-24 판) 와 같은지 먼저 확인. 같으면 다시 올리지 말 것.
- 올릴 때: `sudo systemctl stop gen2-bench` → arduino-cli compile/upload (docs/robot_bench.md 그대로, /dev/ttyACM0) → 재시작.
  원본 백업 firmware/backup/esp32c3_original_full_flash.bin. 올린 뒤 100 Hz·CRC 0·drop 0 확인 (gen2_bench_check.py).

## 2. 전압 보정 (PSU = 기준, 사용자가 연결)
배터리 자리에 PSU, **PSU 표시값을 멀티미터로 한 번 확인** (PSU 표시가 ±1 % 넘게 틀린 경우가 많음).
| 채널 | 점 (3 개 이상) |
|---|---|
| compute (3S) | 9.90 V (0 %), 11.10 V, 12.60 V (100 %) |
| motor (6S) | 19.8 V (0 %), 22.2 V, 25.2 V (100 %) |
각 점에서 raw_V 를 2 s 평균 → 최소제곱 V_true = scale·raw (+ offset 이 0.02 V 넘게 필요하면 offset 도 파라미터로).
합격: 모든 점에서 셀당 ±0.01 V (팩 compute ±0.03 V, motor ±0.06 V).

## 3. 전류 보정
- 무부하 0 점: 부하 분리 상태 `z compute` / `z motor` → current_offset_a.
- 기울기: PSU 전류 표시를 기준으로 2–3 부하점 (compute: 젯슨 대기 / CPU 부하 / 카메라·SLAM 켬, motor: 다리 서기 전류). ESP32 ADC 가 1.5 A 아래를 못 읽는 건 알고 있으니
  1.5 A 이상 점이 있어야 scale 을 정할 수 있음 — 없으면 offset 만 넣고 scale 은 그대로 두고 보고.
- **주의 (PSU 로 모터 채널)**: PSU 는 회생 전류를 못 받는다 — 바퀴를 세게 감속하면 버스 전압이 튀어 PSU 과전압 보호·드라이브 오류가 날 수 있음.
  모터 채널 보정 중엔 균형·주행 하지 말고 서기 (다리만) 까지만. PSU 전류 제한은 compute 3 A, motor 5 A 정도.

## 4. 결과
sensor_hub.yaml 에 값 넣고 gen2-bench 재시작 → 실제 배터리로 돌아와 멀티미터와 한 번 더 비교 (1 % 안). 점별 원값·보정값 표를 comms 로 보내 달라.
