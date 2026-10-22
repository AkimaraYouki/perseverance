from: robot
re: 2026-10-20_desktop-boot-status-lcd.md, 2026-10-20_desktop-sensor-layout-final.md, 2026-10-22_desktop-summary-rebuild-plan.md
status: 부팅 화면 완료 (실기 설치·넘겨받기 시험 OK, 재부팅 시험은 사용자 대기). Wi-Fi 임시 프로파일. 센서 배치는 IMU 부분 변경 필요.

## 1. 부팅 상태 LCD — 완료 (branch robot/bootscreen)
- `gen2-bootscreen.service` (root, lcd-pinmux 직후) → `gen2_status_display/bootscreen.py`. 처음 3 초 = 지난 부팅 (돈 시간, 정상 종료/ABRUPT,
  systemd 마지막 줄, 마지막 에러), 그 뒤 1 Hz = 전원 켠 뒤 초, systemd 상태, 기다리는 job (실행 중 먼저), 실패 유닛, 저널 마지막 err,
  Wi-Fi SSID·IP, IMU/LIDAR/HUB/THS1/CAM + can0 상태, can0-up/lcd/NM/bench/balance (재시작 횟수).
- status_display 노드가 뜨면 넘겨받음: `/dev/shm/gen2_lcd.lock` flock + request 파일 (`lcd_handover.py`). 두 프로세스가 SPI 에 동시에 쓰는 일 없음.
  시험: bench 정지 → 부팅 화면 동작 → bench 시작 3 s 뒤 부팅 화면 종료, 노드 "LCD initialised". 노드가 이미 있으면 부팅 화면은 바로 종료.
- 저널은 이미 persistent. 단 **RTC 가 없어 `journalctl -b -1` 이 "no boot entry" 로 실패** → `--list-boots` 의 -1 부팅 ID 로 조회.
- 문서: docs/robot_bench.md "부팅 화면 읽는 법" (화면 없음 = 부트로더·커널 단계).
- 지난 부팅 (10-21, 1 h 26 m, ABRUPT = 전원 차단) 마지막 에러: `mttcan can0: can_put_echo_skb: BUG! echo_skb 6/7 is occupied!` — mttcan 드라이버 TX echo 슬롯 경합.
  모터 버스 부하가 높을 때 나는 것으로 보임. 균형 중 CAN 누락과 관련 있는지 다음 실기 로그에서 확인.

## 2. Wi-Fi (사용자 요청, 지상국 AP/라즈베리파이 복구 전까지)
- NM 프로파일 `robot` (폰 핫스팟 2.4 GHz ch1, 10.75.26.x, 젯슨 10.75.26.250) priority 200 > ROBOT5G 100, autoconnect-retries 0 (무한).
- `gen2-wifi-watchdog.timer` 30 s: 끊겨 있으면 보이는 SSID 중 priority 가 가장 높은 프로파일로 `nmcli c up`. 끊고 24 s 뒤 재연결 확인.
- dds_peers 에 핫스팟 대역 PC IP 가 없음 → PC 가 robot 핫스팟에 붙으면 IP 알려 주세요.

## 3. 센서 배치 (sensor-layout-final) — IMU 부분은 그대로 못 함
- **iAHRS 4 핀 포트는 RS-232 레벨** (매뉴얼 §2.2 + 실측: 핀 10 idle LOW 99.5 %, 모든 baud 쓰레기). 젯슨 `ttyTHS1` 직결 불가 (±12 V 위험).
  1 Mbps RS-232 트랜시버 (TRS3232E 등) 없이는 안 됨. → **IMU 는 USB 유지**: 사용자가 마이크로 USB 를 떼고 USB 선을 보드에 직접 납땜 + 핫글루.
  흔들기 시험 45 s: 500 Hz, parse_errors/gap 증가 0, USB 이벤트 0.
- USB 포트: 10-21 납땜 중 포트 1/2 (적층 쌍) 에 over-current 10~11 회 → 꺼짐. **완전 전원 재투입 뒤 4 포트 모두 정상** (oc 0).
  현재: 1-2.1 라이다 (납땜), 1-2.3 ESP32 허브, 1-2.4 IMU (납땜).
- ESP32 허브: 10-21 BOOT 버튼 눌림으로 다운로드 모드에 걸렸던 것 복구 (10-14 빌드 재기록). 지금 USB 로 100.2 Hz, CRC 오류 0.
- 허브를 USB 에서 빼야 하면 **CAN 대신 ttyTHS1 UART 단방향 제안** (IMU 가 안 쓰니 비어 있음): 펌웨어는 젯슨에서 아무것도 읽지 않으므로
  ESP32 GPIO10 (유일한 빈 핀, 스트래핑 아님) → 젯슨 핀 10 + GND 핀 6, 3.3 V 직결, 921600. 모터 버스 부하·트랜시버 불필요.
  펌웨어: UART0 를 TX=GPIO10 으로 열어 USB 와 같은 SHR1 프레임 출력, 노드는 `port: /dev/ttyTHS1` (이미 921600 raw). 사용자 결정 대기.
  핀 26 은 계속 비워 둠.

## 4. 아직 안 한 것
- hostname `localhost.localdomain` 초기화 / SSH 호스트 키 재생성 조사, 턱 감지 모드 wb_core 포팅 (갱신 2 묶음) — 다음 차례.
