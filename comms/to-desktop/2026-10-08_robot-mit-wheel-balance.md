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

## LQR gain steps (2026-10-08, same robot defaults fric 0 / yaw_kd 0.15), logs/2026-10-08_balance_gain_step*.csv
K at l = 0.20 [x, v, θ, θ̇]; f = fraction from lowgain to default.

| step | K | travel (max) | pitch sd | fwd wheel torque >8 Hz |
|---|---|---|---|---|
| lowgain | −1.0 −2.38 −12.2 −1.37 | 0.11 (0.17) m | 1.4° | 0.031 |
| all ×1/3 | −1.14 −2.68 −14.3 −2.08 | 0.20 (0.36) | 2.7° | 0.130 (15 Hz) |
| x,v,θ 1/3, θ̇ low | −1.14 −2.68 −14.3 −1.54 | 0.24 (0.38) | 3.4° | 0.075 |
| x,v,θ 2/3, θ̇ low | −1.28 −3.04 −16.1 −1.54 | 0.51 (0.87) | 3.2° | 0.057 |
| x,v,θ full, θ̇ low | −1.41 −3.39 −18.0 −1.54 | 1.30 (1.30) | 5.2° | 0.192 |

Raising θ̇ brings back the 15 Hz wheel chatter; raising x/v/θ with θ̇ fixed gives ±8–9° pitch swings at 2–3 Hz
(less damping). The low-gain table is still the best on hardware. Could you check in sim (with 18' wheel backlash and
the measured ~5 ms MIT path) which K keeps damping with θ̇ ≤ 1.5?

**Correction (user):** during all gain-step runs the user kept pushing the robot by hand on purpose. Travel and pitch sd
above are push responses, not drift/instability, so they do NOT rank the gains. Only the >8 Hz wheel-chatter column is
meaningful (raising θ̇ brings the 15 Hz chatter back). The user's feel for push rejection is pending.
