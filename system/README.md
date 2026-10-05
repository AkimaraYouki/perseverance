# Jetson system configuration (copies of files installed outside the workspace)

| File here | Installed at | Purpose |
|---|---|---|
| `systemd/can0-up.service` | `/etc/systemd/system/` | can0 at 1 Mbit/s, restart-ms 100 |
| `systemd/lcd-pinmux.service` + `sbin/lcd_pinmux.sh` | `/etc/systemd/system/`, `/usr/local/sbin/` | 40-pin pinmux for the ST7789 LCD (SPI1 SFIO, DC/RST/BL tristate off) |
| `systemd/gen2-bench.service` | `/etc/systemd/system/` | bench profile at boot (see `src/gen2_bringup/systemd/`) |
| `systemd/wifi-diag.service` + `scripts/wifi_diag.sh` | `/etc/systemd/system/`, `~/lcd_test/` | boot-time Wi-Fi log → `~/wifi_diag.log` |
| `systemd/post-boot-check.service` | `/etc/systemd/system/` | one-shot post-reboot check (disables itself) |
| `chrony/gen2-lan-server.conf` | `/etc/chrony/conf.d/` | Jetson serves NTP to the LAN / phone hotspot (camera latency clock sync) |
| `networkmanager/zz-gen2-wifi-powersave-off.conf` | `/etc/NetworkManager/conf.d/` | Wi-Fi power save off for all connections (latency tails). Name must sort after `default-wifi-powersave-on.conf` |
| `networkmanager/90-gen2-dds` | `/etc/NetworkManager/dispatcher.d/` (root, 755) | restart gen2-bench when the Wi-Fi IPv4 address appears/changes/disappears (DDS mode re-selection) |
| `sysctl/99-gen2-arp.conf` | `/etc/sysctl.d/` | answer ARP only for addresses on the receiving interface (the Jetson answered for l4tbr0's 192.168.55.1 on the campus LAN) |
| `usb-device-mode/nv-l4t-usb-device-mode-config.sh` | `/opt/nvidia/l4t-usb-device-mode/` | USB device-mode network moved 192.168.55.x → **192.168.66.x** (campus LAN uses 192.168.55.x). Default route via the host PC (192.168.66.100) **metric 100** (was 32766): plugged into the desktop, the Jetson uses the desktop's internet. Package file: re-apply after `nvidia-l4t-usb-service` upgrades |
| `sysctl/99-gen2-linkdown.conf` | `/etc/sysctl.d/` | ignore routes on interfaces without carrier — USB cable pulled → the l4tbr0 default route is skipped and Wi-Fi takes over |
| `networkmanager/91-gen2-wired-peers` + `gen2/wired_peers.conf` | `/etc/NetworkManager/dispatcher.d/`, `/etc/gen2/` | desk debugging: when the wired port comes up, pin each peer (on-link /32 route + permanent ARP) — the campus LAN has an ARP spoofer |
| `udev/90-ax210-btusb.rules` | `/etc/udev/rules.d/` | load btusb for AX210 Bluetooth (NVIDIA rule blocks it) |
| `scripts/st7789_test.py` | `~/lcd_test/` | standalone LCD wiring test |
| `scripts/can_scope.sh` | `~/can_test/` | repeating CAN frames for oscilloscope checks |

Install / restore:
```bash
sudo cp system/systemd/*.service /etc/systemd/system/
sudo install -m 755 system/sbin/lcd_pinmux.sh /usr/local/sbin/
sudo cp system/udev/90-ax210-btusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload
sudo install -m 755 system/networkmanager/90-gen2-dds /etc/NetworkManager/dispatcher.d/
sudo rm -f /etc/NetworkManager/conf.d/99-gen2-wifi-powersave-off.conf   # old name, lost to the package default
sudo cp system/networkmanager/zz-gen2-wifi-powersave-off.conf /etc/NetworkManager/conf.d/ && sudo nmcli general reload conf
NetworkManager --print-config | grep -A1 powersave   # must print wifi.powersave=2
sudo iw dev wlP1p1s0 set power_save off   # now, without reconnecting
sudo cp system/chrony/gen2-lan-server.conf /etc/chrony/conf.d/ && sudo systemctl restart chrony
sudo cp system/sysctl/99-gen2-arp.conf system/sysctl/99-gen2-linkdown.conf /etc/sysctl.d/ && sudo sysctl --system
sudo cp system/usb-device-mode/nv-l4t-usb-device-mode-config.sh /opt/nvidia/l4t-usb-device-mode/
sudo install -m 755 system/networkmanager/91-gen2-wired-peers /etc/NetworkManager/dispatcher.d/
sudo mkdir -p /etc/gen2 && sudo cp system/gen2/wired_peers.conf /etc/gen2/
sudo systemctl daemon-reload
sudo systemctl enable --now can0-up lcd-pinmux gen2-bench
```

Desktop PC side (not the Jetson) — share the desktop's internet over USB-C (2026-10-03):
`desktop/90-jetson-usb-share` → desktop `/etc/NetworkManager/dispatcher.d/` (root, 755). When an interface gets a
192.168.66.x address it turns on IPv4 forwarding and NATs 192.168.66.0/24 (the desktop's `/etc/sysctl.conf` keeps
`ip_forward=0` otherwise). Why: the campus guest Wi-Fi (`kumoh-guest`) is a captive portal — HTTPS gets a self-signed
certificate — so on it the Jetson has no internet and chrony cannot sync (clock stays at 1970 → every TLS check fails).

Not stored here (rebuild on the Jetson if the kernel changes):
- Intel AX210 driver: iwlwifi/iwlmvm built from Ubuntu `linux-source-6.8.0` against the NVIDIA
  6.8.12-tegra headers, installed to `/lib/modules/$(uname -r)/updates/iwlwifi/`.
- Decompressed firmware in `/lib/firmware` (`iwlwifi-ty-a0-gf-a0-{83,84,86}.ucode`, `.pnvm`,
  `intel/ibt-0041-0041.{sfi,ddc}`) — the NVIDIA kernel cannot load `.zst` firmware.
- Jetson-IO overlay `Camera IMX219-C` (CAM1) in `/boot/extlinux/extlinux.conf`.

Wired network (campus LAN, 2026-09-26): `Wired connection 1` = DHCP client with `ipv4.never-default yes`
(Wi-Fi keeps the default route). **Never set it to "Shared to other computers"** on the campus LAN —
that runs a DHCP server for the whole segment. Current lease: 192.168.54.23/24, gateway 192.168.54.1.
The campus LAN has an **ARP spoofer** (`2c:f0:5d:8f:e4:12` answers for the gateway and for other hosts;
40–60 % loss). With the peer pinned (91-gen2-wired-peers): 0 % loss, 0.68 ms to the desktop PC.
Wired is for desk debugging only; competition/outdoor uses Wi-Fi only (dedicated TP-Link AP).
