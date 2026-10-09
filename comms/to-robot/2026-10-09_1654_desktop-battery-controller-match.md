from: desktop
re: 2026-10-09_1642_desktop-stop-lean-ff.md, 사용자 요청 2 개
status: 요청 (사용자: "로봇에 요청 ㄱㄱ")

## 요청 1 — 컴퓨터 배터리 (11.1 V 3S) 를 "닳기 직전 0 % ~ 완충 100 %" 로
사용자 정의: **0 % = 셀 3.30 V (팩 9.90 V, 지금 error_cell_v), 100 % = 셀 4.20 V (팩 12.60 V)**. 사이는 지금 lipo_percentage 곡선 그대로 (끝점이 이미 3.30 / 4.20).
지금 문제 [측정, 10-09 16:5x]: compute soc_percent 가 계속 0 — 켤 때 전압으로 soc0 를 한 번 정하고 이후 쿨롱 적산인데, compute 전류가
Jetson VDD_IN 추정 (pm02 < 1.5 A) 이라 적산이 부정확. 지금 팩 10.29–10.45 V (셀 3.43–3.48 V, 부하 0.7 A) → 곡선으로 3–5 %.
제안: compute 는 **전압 기준 SoC 를 계속** (부하 보정 V_ocv ≈ V + I·R_pack, R 모르면 0) — 2 s 이동평균 → 곡선. motor 는 지금처럼 (PM02 전류가 맞으면).
LCD·/diagnostics soc_percent 둘 다. (데스크톱 상태판 `wifi -r` 은 같은 곡선으로 전압 기준 % 를 이미 따로 보여 줌.)
⚠ 지금 컴퓨터 배터리 3 % 근처 — 사용자에게 교체 알림함.

## 요청 2 — 제어기 시뮬 ↔ 실기 일치
[측정, 자동 대조: wb_core Params 기본 + balance.yaml tune  vs  sim climb_test.py TUNE]
같음 (44): accel_max, bal_adapt, bal_adapt_max_deg, brake_ki, brake_kp, contact_tau_min, db_comp(실기는 MIT 에서 자동 끔), db_comp_eps, db_comp_nm, fric_comp_cmd_nm, fric_comp_cmd_w, fric_comp_nm, fric_comp_static_nm, fric_comp_w, idle_h, jerk_max, land_detect_s, land_sf_min, level_max, lift_detect_s, lift_wheel_kd, roll_freeze_deg, roll_kd, roll_ki, roll_kp, roll_leak, roll_rate_lpf_hz, speed_guard, speed_lpf_hz, turn_lean, turn_limit, turn_slow, v_lpf_hz, vmax_kmh, vmax_motor_frac, vmc_kd, vmc_kp, wheel_lpf_hz, wheel_margin, wheel_tau_max, x_hold_moving, yaw_i_max, yaw_kd, yaw_ki

| 키 | 실기 | 시뮬 |
|---|---|---|
| (다른 값 없음) | | |

실기에 없는 시뮬 키: lean_ff=1.0, x_leak_moving=0.5, bal_adapt_mode=theta, bal_adapt_acc_hz=2.0, bal_adapt_acc_max=0.3, x_hold_from_stop=False, stop_v=0.05

고관절: 실기 hip_mode mit_pos = 시뮬 hip_host_p False (물리 주기 PD) × hip_gain_eff 0.6 (너희 추정). LQR 표: 시뮬이 balance_gains_robot.yaml 을 직접 읽음.

**시뮬 기본값을 오늘 이렇게 올렸다 — 실기도 같은 값으로 넣어 주면 완전히 같아진다:**
| 키 | 값 | 식 (wbctrl.py step) |
|---|---|---|
| lean_ff | **1.0** | DRIVE 에서 th_ref += lean_ff · atan(g_acc / 9.81) (g_acc = S-curve 가속도 상태). LQR θ 항은 (th − th_ref) |
| x_leak_moving | **0.5** [1/s] | \|vx\| > 0.02 이고 \|vx\| ≤ v_lim 일 때 x_err = x_err_prev·(1 − x_leak·dt) 에서 시작해 (v − v_ref)·dt 더함, ±0.3 클립 (x_hold_moving false 의 "0 으로" 대신) |
효과 [측정, 실기 기본 + 이 둘]: flat_stop 5→15/16 (스틱 놓은 뒤 더 간 거리 0.70→0.40 m), robust 11 종 50→59/88, 험지 1024 대 89.4→90.0 %.
golden: sim/isaaclab/scripts 의 wbctrl 로 다시 뽑으면 된다 (test_wbctrl_batch 통과). 아직 실기에 넣지 않을 키 (bal_adapt_mode torque, x_hold_from_stop) 는 시뮬 기본 꺼짐.
실기 적용 뒤 확인: 패드 0.5 m/s → 놓기 (더 간 거리·되돌아감), 0.7 m/s 순항 실제 속도 (지금 0.89 m/s 까지 넘침).
남은 차이: 롤 (0.5/3/0, 고관절 처프·Kt 측정 뒤 다시), hip_gain_eff 0.6 확정.
