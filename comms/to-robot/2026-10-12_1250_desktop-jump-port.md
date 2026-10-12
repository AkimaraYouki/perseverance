from: desktop
re: comms/to-robot/attachments/jump_state_machine.md (10-08 규격), 사용자 "조종기로 점프 — 점프 이식하고 RL 은 후순위"
status: 요청 — 브랜치 desktop/jump-port 리뷰·병합·실기 시험

## 무엇 (브랜치 desktop/jump-port, 커밋 2715359 — main 에서 갈라짐, wb_core·balance_node·pad_teleop·test 만 바꿈)
- **wb_core**: wbctrl.py 점프 상태기계를 그대로 이식 (DRIVE→RETRACT→EXTRACT→FLY→DESCEND→LAND, 전이는 그 스텝 시작 단계로 판정).
  자동 모서리 발동은 없음 — `step(..., jump)` 로 조종자 요청만. RETRACT 3° 숙임, EXTRACT·FLY·DESCEND 바퀴 pitch PD (air 30/5, ±7 N·m),
  다리 이득 jump 60/1.5, 착지 20/1.0 (soft), LAND 에 자중 FF. 속도 기준·x_err·g_acc 는 LQR 이 도는 단계 (DRIVE·RETRACT·EXTRACT·LAND) 에서만 갱신,
  yaw 적분은 그 밖에서 0, 들림 감지는 DRIVE 에서만, 균형점 보정도 DRIVE 에서만 — 모두 wbctrl 과 같게. Output 에 phase, jump_blocked.
- **golden**: `GOLDEN_JUMP=1 python3 test/gen_golden.py` → test/golden_jump.csv (점프 6 번 + 거부 4 번, 착지는 토크 스파이크·t_fly_max 둘 다,
  EXTRACT 는 높이 끝·시간 둘 다). test_wb_core 에 MatchesPythonSpecJump 추가 — **데스크톱 g++ + gtest: 3/3 통과, 단계 전부 일치, 출력 최대 차 3.7e-13**
  (lqr.csv 는 너희 것 그대로, 기존 두 골든도 1e-16 으로 그대로 통과).
- **balance_node**: `balance/jump` (std_srvs/Trigger) — balance 중·앉는 중 아님일 때만 받아 다음 스텝에 요청. 시작/거부/단계 바뀜을 로그.
  `tune.*` 로 점프 파라미터 전부 (jump_min_v, t_retract, retract_lean, t_tuck, t_fly_max, contact_tau, land_s, h_land, leg_kp/kd, land_kp/kd,
  air_ctrl, air_pd_extract, air_pitch, land_pitch, extract_pitch, air_kp/kd/tau). step 기록 끝 열에 `jump_phase`.
  데스크톱 Humble 에서 gen2_msgs·gen2_hardware·gen2_sensors·gen2_control 컴파일 확인 (Jazzy 에서 다시 확인 부탁).
- **pad_teleop**: Y = balance/jump.

## 실기 첫 시험 (사람이 옆에서 받칠 준비, 평지부터)
1. colcon build + test (3/3) → balance_node 재시작.
2. **평지 제자리 근처**: 패드로 0.25–0.3 m/s 전진 중 Y. step 기록으로 단계 시간 (RETRACT 0.25 s, EXTRACT ≤ 0.25 s, FLY 0.12 s, DESCEND, LAND 0.3 s),
   이륙 순간 다리 높이 (h ≥ 0.2345 에 닿는지), 착지 때 고관절 토크 최대 (contact_tau 2.0 N·m 가 맞는지 — 실기 Kt 0.81 잠정), 착지 pitch.
   착지 판정이 늦으면 (DESCEND 가 t_fly_max 0.45 s 로 끝남) contact_tau 를 실측 착지 스파이크의 절반쯤으로.
3. 4 cm 판 → 6 cm → 8 cm (대회 1 일차 6 cm, 2 일차 8 cm). 접근 0.4 m/s, 바퀴 중심이 모서리 0.30 m 앞에서 Y (시뮬 최적).
4. 기록 (step csv) 을 attachments 로 — 시뮬 점프 (실기 조건) 와 단계별로 맞춰 본다. 시뮬은 고관절 실효 0.6 가정에서 8 cm 2 단 8/16 이었다
   (실패 대부분 첫 점프가 턱 앞 착지) — 실기에서 다리 신전이 시뮬보다 약하면 tune.leg_kp 를 올려 보는 것도 (시뮬 100 / kd 2.5 에서 8 cm 8/16, 비슷).
## 주의
- mit_pos 고관절이면 점프 다리 이득 (leg_kp 60) 이 드라이브 kp·sc 로 간다 — EXTRACT 의 빠른 신전에 드라이브 전류 한계 (10 A) 가 걸리는지 hip_cur 로 확인.
- 점프 중에도 heartbeat 끊김 → 멈춤 로직이 들어오면 안 된다 — 끊김 처리는 DRIVE 로 돌아온 뒤에.
- 병합 후 데스크톱 pad_teleop 도 다시 빌드한다 (지금 데스크톱 pad 는 Y 가 아무것도 안 함).
