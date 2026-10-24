from: robot
re: 2026-10-24_0920_desktop-opt1-landing-bundle.md
status: 적용 완료 — 최적1 켜짐 (사용자 "최적 1 적용해줘", 2026-10-24 10:43, disarmed 에서). 실기 시험 대기.

- 스위치: `gen2_bringup/lib/gen2_bringup/gen2_tune.sh opt1 | default` → `~/.ros/gen2_tune` 에 이름 기록 + gen2-balance 재시작
  (standing 이면 거부). balance.launch.py 가 `config/balance_tune_<이름>.yaml` 을 맨 마지막 params 파일로 얹음 (`$GEN2_TUNE` 도 됨).
  launch 로그 `tune overlay: OPT1 (...)`, LCD 머리줄에 `OPT1` 테두리 배지 (STEP 옆).
- 확인: 실행 중 /balance 의 13 키 모두 balance_tune_opt1.yaml 값과 같음 (retract_lean 12, extract_pitch 12, t_tuck 0.0467, air_kp 23.551,
  air_kd 0.5984, air_pitch −13.5451, land_pitch 7.7672, land_kp 21.7418, land_kd 1.1027, h_land 0.1827, spin_match_k 0.4171,
  air_w_max 17.4808, retry_trig_k 0.1397). 나머지는 balance.yaml 그대로 (hip_pd_tau_max 15, imu_timeout_s 0.2, jump_retry 시작 false).
- 시험 기록: 점프 감시기가 점프마다 이륙 5 s 전 … 3 s 뒤 200 Hz 를 따로 저장 중 → 시험 끝나면 metrics CSV 와 함께 comms 로.
- 되돌리기: `gen2_tune.sh default` (사용자가 "기본으로" 하면).
- 최적2 오면 `balance_tune_opt2.yaml` 로 넣으면 같은 방식으로 `gen2_tune.sh opt2`.
