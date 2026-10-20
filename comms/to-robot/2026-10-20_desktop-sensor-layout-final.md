from: desktop
re: 2026-10-20_desktop-status-rebuild.md (재조립 계획)
status: 확정 (사용자 OK) — 센서 연결 구성. USB 는 라이다 하나만. 재조립 때 적용·확인 부탁.

| 장치 | 연결 | 비고 |
|---|---|---|
| IMU iAHRS RB-SDA-v1 | **젯슨 UART 직결** 40 핀 8 (TX) / 10 (RX) / 6 (GND), `/dev/ttyTHS1` | 921600 baud, 500 Hz 그대로 |
| RPLIDAR C1 | **유일한 USB** | 선 직접 납땜 (라이다 4 선 → 변환 보드 패드, USB 선 → 보드 USB 패드), 젯슨 쪽 USB-A 만, ㄱ자 + 핫글루 + 케이블 고정. 3 번 포트 (1-2.3) 금지 |
| 카메라 IMX219 | CSI | 변화 없음 |
| GPS · 전압/전류 | **ESP32 허브 → 내장 CAN (can0, 모터 버스)** | 사용자: can0 = 보드에 납땜한 내장 CAN |
| LCD ST7789 | SPI1 (그대로) | 핀 15 RST, 18 BL, 19 MOSI, 22 DC, 23 SCK, 24 CS0 (system/sbin/lcd_pinmux.sh) → **8/10 과 안 겹침** |

## IMU UART
- 확인: iAHRS TTL 레벨이 3.3 V 인지 (5 V 면 레벨 변환). RS-232 레벨이면 안 됨. TX↔RX 교차, GND 공통.
- 연결 전 루프백: 핀 8–10 잠깐 이어 921600 로 쓴 글자가 돌아오는지. `/dev/ttyTHS1` 권한 (dialout), 콘솔 (nvgetty 등) 이 안 잡고 있는지.
- 설정: `src/gen2_sensors/config/iahrs.yaml` port `/dev/gen2_imu` → `/dev/ttyTHS1` (또는 udev 91-gen2-serial.rules 의 gen2_imu 링크를 ttyTHS1 로).
- 검증: 예전 USB 측정과 비교 — 500 lines/s, RX 간격 평균 2.000 ms / sd 0.19 / 최대 3.55 ms, 10 s 동안 count gap 0. 그리고 착지·넘어짐 충격 뒤에도 끊김 없는지.

## 허브 on CAN (모터 버스)
- ESP32 TWAI + 3.3 V 트랜시버 (SN65HVD230 등), 짧은 스터브, 종단 저항은 버스 양 끝만 (허브에 120 Ω 넣지 말 것).
- ID 는 모터보다 낮은 우선순위 (예: 0x700 대), 전송 ≤ 50 Hz, 프레임 몇 개만. 허브 고장 시 버스 방해 대비: 허브는 bus-off 면 조용히 (자동 복구 간격 길게).
- 붙이기 전·후 버스 사용률 비교 (`canbusload can0@1000000` 또는 candump 프레임 수). 모터 500 Hz 주고받기 지연·누락 그대로인지.
- balance_node 쪽은 허브 프레임을 무시하면 되고, 허브 노드 (gen2_sensor_hub) 가 can0 에서 읽어 /power, gps 토픽으로.

## 그 밖
- 착지 때 USB 끊김 원인 (기계 vs 전원) 은 라이다 하나 남아도 확인: 끊길 때 `dmesg` (disconnect 만 / over-current·전압).
- 새 부품 질량이 정해지면 데스크톱이 시뮬 모델·balance_tables 갱신.
