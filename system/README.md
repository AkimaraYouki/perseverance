# Jetson system configuration (copies of files installed outside the workspace)

| File here | Installed at | Purpose |
|---|---|---|
| `systemd/can0-up.service` | `/etc/systemd/system/` | can0 at 1 Mbit/s, restart-ms 100 |
| `systemd/lcd-pinmux.service` + `sbin/lcd_pinmux.sh` | `/etc/systemd/system/`, `/usr/local/sbin/` | 40-pin pinmux for the ST7789 LCD (SPI1 SFIO, DC/RST/BL tristate off) |
| `systemd/gen2-bench.service` | `/etc/systemd/system/` | bench profile at boot (see `src/gen2_bringup/systemd/`) |
| `systemd/wifi-diag.service` + `scripts/wifi_diag.sh` | `/etc/systemd/system/`, `~/lcd_test/` | boot-time Wi-Fi log → `~/wifi_diag.log` |
| `systemd/post-boot-check.service` | `/etc/systemd/system/` | one-shot post-reboot check (disables itself) |
| `udev/90-ax210-btusb.rules` | `/etc/udev/rules.d/` | load btusb for AX210 Bluetooth (NVIDIA rule blocks it) |
| `scripts/st7789_test.py` | `~/lcd_test/` | standalone LCD wiring test |
| `scripts/can_scope.sh` | `~/can_test/` | repeating CAN frames for oscilloscope checks |

Install / restore:
```bash
sudo cp system/systemd/*.service /etc/systemd/system/
sudo install -m 755 system/sbin/lcd_pinmux.sh /usr/local/sbin/
sudo cp system/udev/90-ax210-btusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload
sudo systemctl daemon-reload
sudo systemctl enable --now can0-up lcd-pinmux gen2-bench
```

Not stored here (rebuild on the Jetson if the kernel changes):
- Intel AX210 driver: iwlwifi/iwlmvm built from Ubuntu `linux-source-6.8.0` against the NVIDIA
  6.8.12-tegra headers, installed to `/lib/modules/$(uname -r)/updates/iwlwifi/`.
- Decompressed firmware in `/lib/firmware` (`iwlwifi-ty-a0-gf-a0-{83,84,86}.ucode`, `.pnvm`,
  `intel/ibt-0041-0041.{sfi,ddc}`) — the NVIDIA kernel cannot load `.zst` firmware.
- Jetson-IO overlay `Camera IMX219-C` (CAM1) in `/boot/extlinux/extlinux.conf`.
