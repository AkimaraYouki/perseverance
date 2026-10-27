from: gcs
re: 2026-10-28_gcs-drift-after-turns-bias-walk.md — 사용자 "로봇 와이파이가 없어 안의 에이전트가 못 돎, GCS 가 처리"
status: 정보 — 로봇에 적용함 (PR: gcs/balance-drift → main), 로봇 에이전트는 다음 세션에 확인 부탁

## 바뀐 것 (branch gcs/balance-drift = origin/main + 2 커밋)
1. 73c3209 balance.yaml: tune.x_turn_wz 0 → 0.3, tune.bal_adapt 0.3 → 0.15, tune.bal_adapt_max_deg 3 → 1.5 (각 줄 주석에 이전 값).
2. 8f1b24b balance_node: tune.retry_approach_v (턱 감지 접근 속도) 를 live_params 에 (0.1..0.4, disarmed 만, vmax_kmh 와 같은 방식).

## 로봇에 한 일 (disarmed, 시계 1970)
- 설치 전: install / ~/gen2_ws/src / origin/main 의 balance.yaml 셋이 같음 확인. gen2_control C++ 소스 = origin/main (md5).
  ~/gen2_ws/src 와 다른 것: launch/balance.launch.py (src 가 옛것), opt1/roll1 overlay 없음 — install 은 main 쪽.
- 백업: ~/gen2_ws/logs/backup_gcs_balance_drift/ (balance.yaml.install, .src).
- ~/gen2_ws/src/gen2_control/config/balance.yaml 도 같은 내용으로 바꿈 (src 에서 다시 빌드해도 유지 — 단 src 의 launch 는 여전히 옛것).
- git bundle → worktree `.claude/worktrees/balance-drift` → `colcon build --build-base build_gcs` (기존 build/ 안 건드림, 71 s, 소스 touch 2026-10-29).
  test_wb_core 6/6 PASS. gen2-balance 재시작 (sudo -n).
- 확인: /balance live_params = [tune.vmax_kmh, tune.retry_approach_v]; 0.25 OK, 0.6 거절; 0.14 로 되돌림.
  x_turn_wz 0.3, bal_adapt 0.15, bal_adapt_max_deg 1.5.
- GCS GUI: 속도 묶음에 '턱 감지 모드 접근 속도' 0.1–0.4 (기본 0.14).

## 추가 (같은 날): tune.jump_max_v 0.6 → 0.4 (사용자 "0.4 로 되돌려")
- 0.6 은 640124a (10-15 08:38, 로봇 에이전트, "(user)"). 오늘 점프 3 회 모두 tilt fault (웅크림 0.16 s 에 pitch +20–25°, FLY +33–42° /
  한 번은 착지 −20°), 이륙 0.37–0.58 m/s. 묶음이 0.2–0.4 에서 튜닝·평가된 값이라 되돌림.
- branch gcs/balance-drift 261e100. 로봇 install·src balance.yaml 교체 (이전 md5 15b51cb9… 확인, 백업 balance.yaml.before_jumpmax04).
  gen2-balance 재시작은 disarmed 될 때 (GCS 가 대기 중).
