# Gen2 wheeled-biped — ROS 2 workspace

Jetson Orin Nano 8GB · Ubuntu 24.04.4 · JetPack L4T R39.2.1 (kernel 6.8.12-1021-tegra) · **ROS 2 Jazzy** · CycloneDDS

## Architecture (current state)

```
ESP32-C3 hub ──USB CDC 100 Hz──> gen2_sensor_hub (C++) ──> /power/*, /gps/*, /diagnostics
                                   (GNSS, compass, power, diagnostics only — never in a control loop)

AK45-10 ──CAN 1 Mbit/s──> gen2_hardware::MotorBus (C++, RX thread → seqlock slots)
                              ├─ motor_monitor_node   read-only → /motors/state, /diagnostics
                              ├─ motor_test_node      guarded tests (services, heartbeat dead-man)
                              └─ motor_cli            interactive bench tool
                          future: real-time balance process (IMU + CAN, C++, no DDS in the loop)

gen2_status_display (Python, 1 Hz, /diagnostics only) ──SPI──> ST7789 2" LCD
```

| Package | Role |
|---|---|
| `gen2_msgs` | `GnssPvt`, `MotorState(Array)`, `MotorTestStatus`, `MotorTest.srv` |
| `gen2_sensor_hub` | HostFrame v1 driver: CRC32, sync, seq-drop / ESP-reset detection, reconnect, iTOW de-dup, PM02 calibration, SoC + runtime estimate |
| `gen2_hardware` | SocketCAN, CubeMars AK **servo-mode** codec (manual V1.0.15 / AK3.0 V3.2.0), MotorBus, MotorTester, monitor/test nodes, CLI |
| `gen2_status_display` | B&W LCD "C-WANG": boot animation, then 4 sectors (1 PC · 2 sensors · 3 motors · 4 network & power). Errors blink 2 Hz (inverted), warnings 1 Hz, OK steady; only changed regions are sent; `rotate_180` option |
| `gen2_camera` | IMX219 → nvjpegenc → `/camera/image_raw/compressed` (Best Effort, depth 1), auto-restart, latency probe (desktop draft + robot measurements) |
| `gen2_tools` | `hub_cli`, `motor_test_gui`, desktop shortcuts |
| `gen2_bringup` | profiles, CycloneDDS config, systemd units, `gen2_bench_check.py` |

Vendor documents used as the source of truth are in `docs/vendor/` (CubeMars manuals, AK45-10
McParams/AppParams, iAHRS manual, ESP32 firmware).

## Build

```bash
cd ~/gen2_ws && source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose     # unit tests (parser, codec, config)
```

## Boot services

| Unit | Purpose |
|---|---|
| `can0-up.service` | can0 1 Mbit/s, restart-ms 100 |
| `lcd-pinmux.service` | 40-pin pinmux for the LCD (SPI1 + DC/RST/BL) |
| `chrony` | Jetson serves NTP to the LAN / phone hotspot (`system/chrony/`) for camera latency measurement |
| `gen2-bench.service` | **bench profile** at boot: sensor hub + read-only motor monitor + LCD + camera |

`journalctl -u gen2-bench -f` for logs. Motors are never commanded by anything that starts at boot.

## Profiles

| Profile | Command | Status |
|---|---|---|
| bench | `ros2 launch gen2_bringup bench.launch.py` (runs at boot) | ✅ |
| motor_test | `ros2 launch gen2_hardware motor_test.launch.py` (GUI) or `motor_cli` | ✅ |
| wheels_up / balance_test / slam / navigation / field | — | not yet (needs IMU, 3 motors, LiDAR, URDF) |

## Desktop tools

`bash src/gen2_tools/scripts/install_desktop_shortcuts.sh` creates: **Gen2 Motor Test UI**,
**Gen2 Motor Test CLI**, **Gen2 Sensor Hub Test**, **Gen2 Bench Check**.

Motor test (UI and CLI share the C++ `MotorTester`): modes **current**, **velocity**, **position**
(relative joint move with the drive's position-speed loop, speed limit, holds until the end).
Limits = motor capability (user request): |I| ≤ 5 A, |ω| ≤ 20 rad/s, move ≤ 720°, ≤ 10 s; position
tests refused near the ±3200° feedback wrap. Explicit "robot lifted" confirmation, 100 Hz stream, abort on STOP / Space / Esc /
Enter / Ctrl+C, stale feedback, drive fault, over-temperature, over-speed, **GUI heartbeat lost
(0.5 s)**; always ends with 0 A. The drive itself stops after 1 s without commands
(AppParams `timeout_msec 1000`). **A hardware E-stop is still required.**

Integration test without hardware (vcan + fake drive, 14 checks):
```bash
sudo modprobe vcan; sudo ip link add vcan0 type vcan; sudo ip link set vcan0 up
python3 src/gen2_hardware/test/test_motor_test_node.py src/gen2_hardware/config/motors.yaml
```

## Logging (sim ↔ real comparison)

```bash
ros2 run gen2_tools record_bag.sh <experiment>      # mcap in ~/bags/<date>_<experiment>; keep STILL 5 s
ros2 run gen2_tools bag_to_csv ~/bags/<bag> [--start S --end S --still 5]
```
Records motors (state + commands + tests), IMU (`/imu/data`, `/imu/data_raw`, `/imu/raw`),
`/controller/state`, diagnostics (CAN stats), power, GNSS, TF, scan, `/cmd_vel`, `/joy`. Topics
that appear later are picked up automatically. CSV columns follow `sim/isaaclab/scripts/play_log.py`
(`q_`, `qd_`, `cur_`, `tau_`, `kt_` per motor; `roll pitch yaw angvel_* acc_*`); `trim.txt` holds
the still-window means (static lean trim). Do not commit bags; put CSV excerpts in `logs/`.

## Desktop ↔ robot agents

`comms/` is a mailbox between the desktop (Isaac Sim) agent and the Jetson agent — see
`comms/README.md`. `sim/` and `tools/shr1/` belong to the desktop side.

## Test sequence (pass/fail)

| Step | How | Pass criteria | Status |
|---|---|---|---|
| 1 ROS env | `ros2 doctor --report`, `colcon build`, `colcon test` | builds, 0 test failures | ✅ |
| 2 CAN only | `ip -d link show can0` | UP, 1 Mbit/s, ERROR-ACTIVE, `rx_error_frames 0` in diagnostics | ✅ |
| 3 motor state read-only | bench + `ros2 topic echo /motors/state` | each motor fresh (`stale false`), ~50 Hz feedback, `tx_frames 0` | ✅ 1/3 motors (AK45-10 id 69) |
| 3b position scale | UI "Hand-rotation scale check" or `motor_cli` → `r` | 1 output turn ⇒ raw Δ = `raw_deg_per_output_rev`; set `verified.position_scale` | ⬜ do it |
| 4 single motor low power | UI / CLI current test 0.2–0.5 A, 1 s, wheel lifted | moves smoothly, result `done`, no drive fault | ⬜ |
| 5 ID / direction / velocity | UI velocity test 1 rad/s | "mean(2nd half)" ≈ command (±10 %), + command = + joint motion; set `verified.*` | ⬜ |
| 6 IMU | iAHRS RB-SDA-v1 (in delivery) | — | ⬜ |
| 7 ESP32 hub | `Gen2 Sensor Hub Test`, `gen2_bench_check.py` | 100 Hz, CRC 0, drops 0, PM02 within 1 % of multimeter | ✅ link / ⬜ PM02 calibration |
| 8–17 | odometry, EKF, LiDAR/TF, SLAM, balance, Nav2, GNSS | — | ⬜ |

`ros2 run gen2_bringup gen2_bench_check.py` automates steps 2, 3 and 7 (link).

### PM02 calibration (step 7)
In **Gen2 Sensor Hub Test**: `v motor 24.95` (multimeter reading) → `voltage_scale`;
disconnect the load, `z motor` → `current_offset_a`. Same for `compute`. Put values in
`src/gen2_sensor_hub/config/sensor_hub.yaml`, rebuild, `sudo systemctl restart gen2-bench`.

### Battery runtime estimate
Initial SoC from the resting voltage (2 s after connect), then coulomb counting; runtime =
energy above a 20 % reserve ÷ 30 s average power. Reset only when a pack is disconnected ≥ 1 s.
Accuracy depends on the PM02 current calibration.

## Known issues / open items
- GNSS: fitted module is a NEO-M8N (MON-VER) running at 230400 UBX. ESP32 firmware in
  `firmware/biped_sensor_hub/` (flashed 2026-09-24): 230400 + auto-baud, current pins ADC_0db,
  GPS UART byte counter. Build/flash (stop gen2-bench first):
  `arduino-cli compile -b esp32:esp32:esp32c3:CDCOnBoot=cdc --output-dir build . &&
   arduino-cli upload -b esp32:esp32:esp32c3:CDCOnBoot=cdc -p /dev/ttyACM0 --input-dir build .`
  Original flash backup: `firmware/backup/esp32c3_original_full_flash.bin`.
- AK45-10 datasheet: Kt 0.127 N·m/A rotor side → 1.27 N·m/A output (config), 14 pole pairs,
  rated 2.1 A / peak 5 A. The drive's own current limit (AppParams `l_current_max` 35 A) is far
  above the motor's peak: lower it to ≈5 A in the CubeMars tool. Confirm Kt with a torque arm.
- Measured: wheel outer-to-outer 228 mm, wheel thickness 30 mm → track width b ≈ 198 mm
  (contact assumed at wheel mid-plane).
- RPLIDAR C1 not delivered yet; its USB–UART adapter is a CP2102 (`10c4:ea60`, serial `0001`).
- Robot has **4 actuators**: wheels AK45-10 ×2, legs AK60-6 V3.0 ×2 (L = +y, R = -y).
  Drives that appear on the bus but are not in `motors.yaml` are **discovered automatically**
  (read-only): `/motors/state` entry `id_<N>?` with `configured: false` and raw values, a WARN
  diagnostic, and a row `#N?` on the LCD. Add them to `motors.yaml` to get scaling and commands.
  Only one AK45-10 (id 69) is on the bus so far; add the others to `motors.yaml` with their IDs.
  AK60-6 V3.0 supports disable (mode 15) and has Kt 0.5994 N·m/A in the V3.2.0 manual table.
- Feedback upload is 50 Hz (`send_can_status_rate_hz`); balance control needs 500–1000 Hz
  uploads (AK 3.0: up to 2000 Hz) — change in CubeMars tool before step 12.
- IMX219 on CAM1 works after reseating the ribbon cable (an earlier boot failed with I2C -121).
- DDS (CycloneDDS 0.10): `gen2_dds.sh` writes `~/.ros/gen2_cyclonedds.xml` at start-up from the
  interfaces that have IPv4: **wired** `enP8p1s0` (campus LAN, priority 20, multicast off → peers only,
  so other ROS users on the LAN neither see nor command the robot), **Wi-Fi** `wlP1p1s0` (priority 10),
  else **local** (lo only). Peers: `src/gen2_bringup/config/dds_peers.txt` (+ own wired IP). A loopback
  entry next to a real interface became Cyclone's primary and broke off-board traffic; Cyclone does not
  fail over when an address disappears (tested) → the NetworkManager hook
  `system/networkmanager/90-gen2-dds` restarts gen2-bench when a wired/Wi-Fi address changes.
  New terminals get the same config via `~/.bashrc`.
