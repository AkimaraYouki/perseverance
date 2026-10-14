from: robot
re: 2026-10-14_0100_desktop-kt083-params.md (요청: wb_core 에 spin_match_k / air_w_max 포팅 — 골든 포함)
status: 완료 (커밋). 실기 점프는 아직 꺼짐 (jump_max_v 0) — 사용자 OK 뒤 켬

- wb_core: Params spin_match_k, air_w_max (기본 0 = 끔). 공중 PD 블록을 wbctrl (jump-rl 2e03b80) 과 같은 순서로:
  u = air_kp·(pitch − ref) + air_kd·gy → 바퀴마다 uw = u; DESCEND 이고 k > 0 이면 uw = k·(vf/R − ω_abs[k]);
  air_w_max > 0 이고 sgn(uw)·ω_joint[k] > air_w_max 면 uw = 0; act = clip(uw / air_tau). air_ctrl 꺼짐이면 0.
  vf 는 wb_core 의 vf_ (뜬 동안 이륙 직전 유지, wbctrl self.vf 와 같은 시점에 갱신).
- 골든: gen_golden.py 에 GOLDEN_SPIN=1 (+ WBCTRL_DIR = `git show origin/desktop/jump-rl:sim/isaaclab/scripts/wbctrl.py` 사본) →
  golden_jump_spin.csv (spin_match_k 0.25, air_w_max 5: DESCEND 143 단계 중 136, FLY 93 단계에서 바퀴 명령이 바뀜 = 둘 다 실제로 탐).
  test_wb_core: 4/4 (새 케이스 max |diff| 5.6e-16). 참고: jump-rl wbctrl 로 기존 golden_jump 를 다시 뽑으면 a2/a3 가 3.7e-13 차이
  (np.select → np.where 반올림) — 기존 파일은 그대로 둠. jump-rl 의 wbctrl 이 main 에 합쳐지면 WBCTRL_DIR 없이 된다.
- balance.yaml: tune.spin_match_k 0.251, air_w_max 0 (CMA-5 그대로). itest 21/21.
- 실기 순서 (사용자 OK 뒤): jump_max_v 0.4 → 평지 0.3 m/s 접근 + Y × 3 → step 기록 보냄
  (이륙 pitch, 공중 바퀴 ω, DESCEND 에서 ω_abs·R vs vf, 접지 순간 미끄럼, 착지 다리 h, 고관절 전류).
