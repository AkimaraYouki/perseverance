# Robot: AK45-10 wheel ID 2 (left) bench test in legacy MIT — works, no deadband

Wheel lifted (free spinning). Raw CAN, legacy MIT frames (P ±12.56, V ±20, T ±8, KP 500, KD 5), 500 Hz.

| test | result |
|---|---|
| torque ±0.05 / ±0.07 N·m | no motion (static friction) |
| torque ±0.10 N·m, 0.5 s | spins: +5.6 / −6.5 rad/s after 0.5 s |
| torque ±0.20 N·m, 0.15 s | ±8.2–8.5 rad/s; kd 0.3 brake stops it at once |
| velocity ±3 rad/s (kd 0.3) | tracks 2.8–2.9 rad/s, needs ≈0.08 N·m |
| position ramp +1 rad and back (kp 5, kd 0.2) | max error 0.061 rad, final +0.015 rad |

- Breakaway 0.07–0.10 N·m = wheel/gearbox static friction, not a drive deadband
  (servo-mode current had a 0.5 A deadband; MIT passes small torques). Matches wb_core friction comp 0.11.
- Torque feedback follows the command (0.10 → 0.093–0.101 N·m reported). err 0, temp byte 72 (≈32 °C if −40 offset).
- Two runs hit my 15 rad/s over-speed stop (0.2 N·m for 0.5 s free; an unramped kp 5 step of 1 rad on the free wheel) — test design, not the drive.

Wheel ID 1 (right): no MIT reply and no servo upload on the bus — the user is changing hardware now.
balance_node now drives mit_legacy wheels (commit d2dd555): MIT enter on arm (refuses if no fresh reply within 100 ms),
torque frames t = tau, zero + MIT exit on disarm/fault, deadband compensation off. Not run on the robot yet.
