from: robot
re: 2026-10-20_desktop-step-detect-mode.md (갱신 2), 2026-10-22_desktop-summary-rebuild-plan.md §2
status: 턱 감지 모드 이식 완료 (골든 일치), 실기 시험 전. 점프 묶음 (CMA-11 15 N·m + 갱신 2) 적용. IMU 끊김 원인 보고.

## 1. 턱 감지 모드 (wb_core jump_retry) — branch robot/hub-refit 6762233
- wbctrl desktop/jump-rl 6b64ca0 의 jump_retry 를 wb_core 로: 접근·settle·stall_frac·막힘·후진·재가속·retry_trig_k·fire_frac·
  land_hold·retry_vmax·재막힘·놓으면 취소. 점프 상태기계는 그대로 (요청만 만듦).
- 골든: `GOLDEN_RETRY=1 WBCTRL_DIR=<6b64ca0 wbctrl> python3 test/gen_golden.py` -> golden_retry.csv (5200 스텝, 턱 코스:
  막힘 3, 후진 3, 점프 2, 접근 중·재주행 중 놓기 각 1; fire_frac 0.5 / land_hold 0.5 / vmax 0.4 도 켜서 덮음).
  test_wb_core: rt_phase 매 스텝 비교, 최대 차 1.3e-15. 기존 골든 4 개 그대로 통과. 전환 시 상태 초기화 시험 추가.
- balance_node: tune.jump_retry (시작은 끔) + retry_* / trigger, 서비스 `balance/step_mode` (SetBool, 다음 제어 스텝에 적용),
  `balance/step_mode_active` (Bool, transient_local), 단계·발사 로그, step 로그 끝에 `rt_phase` 열 추가.
  vcan0 / 도메인 88 에서 서비스·토픽 확인. 실기 install 반영, disarmed 에서 재시작 확인 (jump_retry false).
- 값 (balance.yaml): 갱신 2 최종 — approach 0.14, settle 0.5, stall_frac 0.5, block 0.15, back 0.3004 @ 0.3, retry_v 0.3,
  trig_k 0.4, contact 0.07, trigger 0.27. fire_frac / land_hold / vmax = 0.
- 패드: 십자키 ↑ 켜기 / ↓ 끄기 (pad_teleop dpad_y_axis 7, dpad_up_sign 1 — 노트북 패드로 /joy 확인 필요). LCD 머리줄 'STEP'.

## 2. 점프 묶음 적용 (사용자 OK, 2026-10-23)
- balance.yaml tune 17 개 = balance_tune_jump_cma11_hip15.yaml + 갱신 2 (extract_pitch 6.7183, h_land 0.2051, land_kp 22.8847,
  land_kd 0.5, spin_match_k 0.6944). hip_pd_tau_max 12 -> 15. 옛 값은 [ ] 주석.

## 3. "균형이 혼자 꺼짐" = IMU stale (참고: 시뮬 지연 모델)
- iAHRS 가 **움직일 때만, 정확히 8 s 마다 12–166 ms 출력을 멈춤** (shm 추적: 읽는 스레드는 read 대기 = CPU 문제 아님,
  샘플 유실). 손으로 흔들어도 (균형 꺼짐) 같음. FFT/진동 꺼져 있음, zv=0 · 250 Hz 시험도 그대로. IMU 공급 4.4 V (사양 4.2–12).
- 대응: imu_timeout_s 0.05 -> 0.2 (마지막 샘플 유지). 시뮬에 "8 s 마다 30–170 ms IMU 정지" 를 넣어 보면 실기와 가까울 것.

## 4. 그 밖
- pad_teleop: START 때 높이 기본값 (일어설 때 H_MAX 0.2325 로 펴지던 것). 노트북은 pull 뒤 재시작 필요.
- leg_l 영점: 10-22 hip_cli zero 가 60° 틀린 상태에서 저장됨 -> -60° 보정 (offset -0.474730, stops -13.1/+59.5).
