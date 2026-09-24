# SHR1 센서허브 패킷 테스트

`biped_sensor_hub_esp32_c3_gpio8_fixed.ino` (ESP32-C3 Super Mini) 가 USB CDC 로
쏘는 텔레메트리 프레임을 호스트에서 검증하는 도구.

## 프레임 규격 (펌웨어 struct 와 1:1 대조 완료)

| 항목 | 값 |
|---|---|
| 크기 | **136 byte** packed (`static_assert` 와 일치) |
| magic | `0x31524853` = `"SHR1"` LE = `53 48 52 31` |
| version | 1 |
| CRC | CRC32-IEEE (poly `0xEDB88320`, init/final `0xFFFFFFFF`) — **앞 132 byte**, crc 필드 제외. `zlib.crc32` 와 동일 |
| 전송률 | 100 Hz, 921600 baud (13.3 KiB/s ≈ 링크 용량의 15 %) |
| 포트 | ESP32-C3 내장 USB JTAG/serial → `/dev/ttyACM*` (VID:PID `303a:1001`) |

필드 오프셋 표는 `--selftest` 가 출력한다. 펌웨어 struct 를 고치면
`--selftest` 가 먼저 깨지므로 회귀 검사로 쓸 것.

## 사용법

```bash
python3 shr1_packet_test.py --selftest       # 보드 없이 파서/CRC/재동기 검증
python3 shr1_packet_test.py --loopback 3     # pty 가상보드로 수신경로 전체 검증
python3 shr1_packet_test.py --loopback 5 --noise   # 손상/유실 주입 → 검출력 확인

python3 shr1_packet_test.py --once           # 실보드 한 프레임 전체 디코드
python3 shr1_packet_test.py -d 60            # 60 s 측정 + 판정 리포트 (포트 자동탐지)
python3 shr1_packet_test.py --stall          # 호스트 정지 내성 (버퍼 여유) 측정

python3 shr1_packet_test.py -d 30 --record cap.bin   # 원시 바이트 녹화
python3 shr1_packet_test.py --replay cap.bin         # 녹화 재생 (젯슨 캡처 분석용)
python3 shr1_packet_test.py -d 30 --csv log.csv      # 필드 CSV
python3 shr1_packet_test.py --json | jq .            # 프레임당 JSON 한 줄
```

## 2026-09-24 데스크톱 실측 결과

10 s 측정, `/dev/ttyACM1`:

- 1002 프레임, **CRC 오류 0 / 헤더 오류 0 / 재동기 0 / seq 누락 0 / 보드 드랍 0**
- 99.98 Hz, ESP 송신 간격 평균 10.00 ms (min 6.21 / max 13.80, σ 0.23 ms)
- 종합 **PASS** — 링크·프레이밍·CRC 계층은 문제 없음

## 확인된 이슈 3건

### 1. 호스트가 1 s 이상 멈추면 USB 링크가 물린다 (가장 중요)

| 정지 | 보드 드랍 | seq 누락 | tty 잔량 |
|---|---|---|---|
| 10–200 ms | 0 | 0 | ≤ 2720 B | 
| 500 ms | 12 | 12 | 4095 B (포화) |
| 1000 ms | — | — | 장치가 USB 에서 탈락, **자동 복귀 안 함** |

커널 tty 버퍼가 4096 B = 30 프레임 ≈ 300 ms 분. 그 이상 안 읽으면 유실되고,
1 s 를 넘기면 CDC 가 죽는다.

→ 젯슨 쪽 대응: 소비 스레드를 **200 ms 안쪽 주기로 반드시 read**,
그리고 포트 소멸/재열거를 감지해 재오픈하는 워치독 필수.
(펌웨어에도 USB TX 스톨 워치독을 넣는 게 안전)

### 2. GPS 가 한 바이트도 안 들어온다

`FLAG_GPS_PACKET_SEEN` 이 전 구간 0, 동시에 `gps_checksum_error_count` 도 0.
깨진 UBX 조차 없다는 뜻 → 보율 불일치(그러면 보통 동기 실패만 남음)보다는
**배선 미연결 / GPS 모듈 무전원** 쪽. 확인 순서:

1. Holybro TX → ESP GPIO20(RX), Holybro RX → ESP GPIO21(TX) 결선
2. GPS 모듈 전원
3. 모듈이 38400 이 아닌 9600 으로 출고됐는지 (`GPS_UART_BAUD`)

### 3. PM1(compute 레일) 전압이 유효 범위 경계에서 떨림

`FLAG_PM1_VALID` 는 `5.0 V < compute_v < 20.0 V` 일 때만 선다.
실측 17.41 V 로 20 V 상한 근처라 프레임의 74.8 % 에서 플래그가 떨어졌다.
compute 전류도 24.25 A / 422 W 로 비현실적 → **PM02D 입력이 떠 있는(floating) 상태**로 보인다.
PM02D 를 실제 레일에 연결한 뒤 재측정할 것. 떨림이 남으면
`PM1_V_INPUT_RATIO` / `PM02_VOLTAGE_DIVIDER` 교정이 필요하다.

(motor 레일은 12.46 V / 0.10 A 로 `PM2_VALID` 안정적으로 유지)
