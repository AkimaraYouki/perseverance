# Robot: first balance runs with AK45-10 MIT wheels (low-gain LQR, trim 1.75, roll off)

All 15 s balance + sit, user steadying by hand. Logs: logs/2026-10-08_balance1{0..4}_*.csv (100 Hz ControllerState).

| run | change | travel | pitch sd | wheel torque >8 Hz (fwd / yaw) |
|---|---|---|---|---|
| 10 | first MIT run | +0.03 m | 1.4° | 0.016 / 0.007 N·m (quiet) |
| 12 | same params (after leg_r re-zero) | +2.5 m | 5.7° | 0.21 / 0.35 — 15 Hz fwd, 12 Hz yaw chatter, user felt Z shaking |
| 13 | fric_comp_nm 0.11 -> 0 | +0.48 m | 1.7° | 0.09 / 0.27 — 13 Hz yaw left |
| 14 | + yaw_kd 0.5 -> 0.15 | +0.11 m | 1.4° | 0.03 / 0.005 — quiet |

- MIT passes small torques (no 0.5 A deadband), so the velocity-sign Coulomb comp (0.11·tanh(w/0.5)) and yaw_kd 0.5
  now reach the wheel; with gear backlash (18') they chatter at 12–15 Hz. The servo deadband used to hide this.
- New robot defaults (balance.yaml tune): fric_comp_nm 0.0, yaw_kd 0.15 (new param tune.yaw_kd). db_comp auto-off for MIT wheels.
- Please check in sim: does yaw_kd 0.15 still hold heading well enough, and is a friction comp with a dead zone (e.g. only
  above |w| > 2 rad/s) worth it? Next on the robot: try the default LQR (Q θ̇ 5, R 1) now that the wheel path is faster.

Other events today:
- leg_r: link slipped ~30.5° on the motor output during a sit (leg blocked, hip pushed 9.9 A). End stops re-measured,
  offset −0.401426 -> 0.131772 rad. New guard: hip |I| > 5 A while sitting = blocked -> fault (sit_sat_current_a).
- Wheel directions flipped for the MIT firmware (L −1, R +1).
