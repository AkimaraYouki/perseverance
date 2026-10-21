from: desktop
status: 정리 — 10-20 이후 데스크톱 작업 전부 + 재조립 뒤 순서 (사용자 요청 "지금까지 내용 정리해서 보내")

## 1. 재조립 (사용자 진행 중: 새 부품, 필라멘트 제조사 바뀜 → 부품당 ±5 g, 합 50–60 g)
- **센서 연결 확정** (2026-10-20_desktop-sensor-layout-final.md): IMU → 젯슨 UART `/dev/ttyTHS1` (핀 8/10/6, 921600, 3.3 V TTL 확인·루프백 먼저),
  라이다 → 유일한 USB (선 직접 납땜·고정, 3 번 포트 금지), GPS·전압/전류 → ESP32 허브 → 내장 can0, LCD 그대로 (핀 15/18/19/22/23/24), **핀 26 비워 둘 것** (나중 SC16IS752).
- **질량**: 사용자 방침 = 먼저 지금 파라미터로 시험, 이상 있을 때만 재최적화. 근거: 시뮬 평가가 이미 몸통 질량 ±15 %·무게중심 ±2 cm 무작위 (60 g = 1.4 %),
  LQR 게인 변화 예상 ~1 % → balance_gains_robot (kt083 ×1.2) 그대로. 조립 뒤 **th_trim 재보정**만. 부품 질량은 저울로 기록해 두면 필요할 때 데스크톱이 반영.
- 재최적화 신호: 점프 높이·넘어지는 모양이 시뮬과 다름, th_trim 2° 넘게 바뀜, 넘어짐 잦음 → 질량·step 로그를 데스크톱으로.

## 2. 점프 — 턱 감지 모드 (요청 정리, 아직 미반영분 포함)
- 사양·최종값: 2026-10-20_desktop-step-detect-mode.md (**갱신 2** 의 묶음이 최종). 고관절 명령 상한 **15 N·m** 전제 (2026-10-15_2000_desktop-jump-hip15.md).
- wb_core 포팅 필요: jump_retry 로직 + retry_approach_v / retry_v / retry_settle_s / retry_stall_frac / **retry_trig_k** (점프 지점 = trigger + k × (v − retry_v)).
  기준 wbctrl = desktop/jump-rl `6b64ca0`. 골든은 그 wbctrl 로 턱 감지 케이스 (막힘 → 후진 → 재가속 → 점프) 를 만들어 맞춤.
- 패드 켜기/끄기 (2026-10-20_desktop-step-mode-pad-toggle.md): 십자키 ↑ 켜기 / ↓ 끄기, Y = 즉시 수동 점프 그대로, 서비스 `balance/step_mode` (SetBool), LCD "STEP".
- 시뮬 (실기 보정 플랜트: 고관절 실효 0.3–0.4, w0 29.3, 바퀴 토크 0.83, 각 2048 판, 턱 앞 착지는 자동 재시도 최대 3 회):
  **대회 2 단 계단 6 cm 97.3 %, 8 cm 98.5 %, 단 10 cm 99.8 %, 단 12 cm 98.3 %** (넘어짐 0.2–2.4 %).
  수동 점프 (Y, 누르는 위치 22–40 cm 무작위, 재시도 없음): 8 cm 60 %, 10 cm 53 % → 턱·계단은 턱 감지 모드 권장.
- 시뮬은 실기보다 낙관적이었음 (평지 시뮬 83 % vs 실기 75 %). 몇 값이 탐색 경계에 붙음 (trigger 0.27, trig_k 0.4, retry_v 0.3 = 사용자 상한).

## 3. 재조립 뒤 실기 순서 (제안)
1. 센서·USB 확인: IMU 500 Hz 간격 (예전 평균 2.000 ms / 최대 3.55 ms 와 비교), can0 버스 사용률 (허브 붙이기 전·후), USB 끊김은 dmesg 로 기계/전원 구분
2. 균형 + th_trim 재보정
3. 턱 감지 모드: 점프 끈 채 상자 벽 → 막힘 → 후진 0.30 m → 재가속 0.3 m/s 동작만 (재가속 실제 속도 넘침 확인 — 시뮬 녹화에서 0.42 m/s 까지)
4. 점프: 평지 → 6 cm → 8 cm → 2 단 계단. step 로그 (jump_phase, 이륙 gy, EXTRACT hip_cur, 착지 h, 바퀴 ω) 를 데스크톱으로
5. Fold 휴대폰 지상국 실기 연결 (comms/to-gcs/2026-10-20_desktop-fold-phone-gcs-status.md 순서)

## 4. 그 밖 요청 (미반영)
- 부팅 상태·에러 LCD 표시 + 저널 보존 (2026-10-20_desktop-boot-status-lcd.md)
- **젯슨 이상 (2026-10-21 데스크톱이 USB-C 로 확인)**: hostname 이 `localhost.localdomain` 으로 초기화, SSH 호스트 키 재생성 (파일 날짜 1970-01-01,
  ED25519 `SHA256:uuu2HJF+ftbInUCZoL7zfaTRfLIlI+Y8GGZoNWY96Ww`, 예전 `DeRKRu…`). 부팅 불량과 관련 있는지 (파일시스템 점검, /etc 복구) 확인 부탁.
- **ROBOT Wi-Fi**: 2026-10-21 데스크톱(ROBOT 2.4G)·젯슨(ROBOT5G) 둘 다 AP 접속·인증은 되는데 라즈베리파이 192.168.50.1 · AP 192.168.50.2 무응답 (ARP 도 없음)
  → AP 유선 쪽 (라즈베리파이 전원/eth0, PoE 인젝터 LAN 케이블) 문제. 사용자 표: 젯슨 고정 IP 192.168.50.10 (지금 문서·도구는 .124) — 바꾸면 알려 주세요, 데스크톱·노트북·폰 설정 같이 바꿈.
- 젯슨 인터넷은 데스크톱 USB-C 공유로 정상 (2026-10-21 확인) → git pull 은 그 경로로.

## 데스크톱 쪽 산출물
- 시뮬 코드: branch desktop/jump-rl `6b64ca0` (wbctrl, jump_rl --retry/--p_stairs/--cma_space, plot_jump_trace.py)
- 바탕화면: 점프궤적/ (최적화 전·후 궤적 그림), ascento_jump_plateau_{80,100,120}mm*.mp4, ascento_jump_stairs2_{60,80}mm*.mp4
