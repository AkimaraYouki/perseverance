---
from: gcs
date: 2026-10-29
topic: step-detect false "blocked -> back off" after landing — wb_core landing settle installed on the robot
---
User: "한번 뛰고 뒤로 갈려고 함 / 앞으로 가고 갑자기 뒤로 밀려남" (manual D-pad step mode and follow + auto step, falls).

Robot step log (manual, 196 s jump): landing overshoot v 0.44-0.85 vs approach 0.14 -> x_err +0.19 m (moving catch-up,
x_leak_moving 0.5) + slope integrator negative (v_ki 0.8) -> v_ref 0.03-0.04, v 0.04 < stall 0.5 x 0.14 -> blocked ~1 s
after landing -> back off at -0.96 m/s -> re-run -> jump refused 0.64 m/s. Same pattern earlier (follow + auto step, fell).

Fix (branch gcs/land-reset dc0b354, on top of gcs/balance-drift): tune.retry_land_settle_s (core default 0 = golden
unchanged, 6/6 wb_core tests pass; balance.yaml 1.0). For that time after a landing in step-detect mode: no block test,
no 'moved' credit, x_err and v_i held 0, v_ref = ramped command. Installed + gen2-balance restarted (plywood overlay kept).
NOTE robot build: build_gcs/gen2_control cache points at the balance-drift worktree; this one was built with
--build-base build_gcs_lr from .claude/worktrees/land-reset. Please merge into your wbctrl spec / sim if you keep it.

Also on the robot (gen2_tools, gcs/auto-step-safe-exit 56d6e35): auto step STEP ending without a jump -> SLOW (not
FOLLOW 0.7; 3 falls), false-alarm margin 0.35 -> 0.8 m (auto_false_m), forward 0 for 0.8 s after each landing
(auto_land_stop_s).
