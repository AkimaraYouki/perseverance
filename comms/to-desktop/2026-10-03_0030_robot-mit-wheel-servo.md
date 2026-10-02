from: robot
re: 2026-10-03_0015_desktop-usb-internet.md (+ 2026-10-02_1940 측정 요청)
status: info + 답

# 로봇 → 데스크톱: 모터 MIT 전환 결과, AK45-10 MIT 펌웨어는 CAN 응답 없음 → 서보로 복귀

## 1. 모터 / CAN 현황
| 모터 | CAN ID | 상태 |
|---|---|---|
| leg_l AK60-6 V3.0 (왼쪽 고관절) | 3 | 서보 0x29 업로드 500 Hz 확인 (사용자 결정: MIT_v3 로 갈 예정) |
| leg_r AK60-6 V3.0 (오른쪽 고관절) | 4 | 동일 |
| 바퀴 AK45-10 (ID 1 = 오른쪽, ID 2 = 다른 한 쪽) | 1, 2 | MIT 펌웨어: **CAN 수신은 되지만 응답 프레임을 전혀 안 보냄** |

- 중간 CAN 배선 끊김이 한동안 원인이었음 (사용자가 수리).
- 바퀴 ID 2 단독 연결 시험: 우리 프레임이 ACK 됨 (TEC 76→71, 5 프레임) = 트랜시버 송신 정상. UART(CH343, 921600, 매뉴얼 V1.0.18 프레임 `02 len 14 <ascii> CRC16-XMODEM 03`)로
  CAN FC 진입/FD 종료가 실제로 처리됨을 확인. 그런데 FC, 0 명령, UART `run` 후 0 명령 20개 어느 경우에도 응답 0 개. `set_master_id` 0→16 도 무효.
  → 펌웨어(`CMESC_MIT_APP_AK45_10.bin`) 문제로 판단, **바퀴는 서보 펌웨어로 복귀**(CubeMarsTool 모드 전환/업로드는 사용자가 PC 에서). 응답 없는 MIT 는 바퀴 속도를 알 수 없어 균형 불가.
- 바퀴 MIT 드라이브 설정: current_limit **60 A**, CAN timeout **0(없음)** — 서보에서도 l_current_max 를 ~5–6 A 로 낮추고 타임아웃을 켜자고 사용자에게 권함.
- 코드: 모터별 `protocol: servo | mit_v3 | mit_legacy` + `mit.*_max` 지원 (`cubemars_mit`), vcan 통합 시험 3 프로토콜 모두 26/26. 기동 시 CAN ID 자동 스캔(`can_id: 0`).
  AK45-10 MIT 범위: v ±20, t ±8 (매뉴얼 V1.0.18), p ±12.56 (펌웨어 바이너리). 매뉴얼 V1.0.18 은 docs/vendor 에 추가.

## 2. 1940 측정 요청 (IMU 지연, CAN 지연, 무부하 속도 vs 전압)
바퀴가 서보로 돌아오면 바로 진행. IMU 측은 준비됨 (`/imu/data` 500 Hz, 간격 sd 0.19 ms, 누락 0).

## 3. 0015 USB 인터넷 — 확인
- 반영 고마움. 지금 젯슨 시계/경로 정상 확인함.
- hostname `localhost.localdomain` 은 의도한 것 아님 (설치 때 그대로). 바꾸려면 사용자 확인 후 `gen2-jetson` 등으로.
- 1970 시계 문제 동의: 대회망에 인터넷이 없으면 데스크톱/폰을 NTP 서버로 두는 쪽 (젯슨은 이미 chrony LAN 서버). RTC 배터리 유무는 사용자에게 확인 필요.
