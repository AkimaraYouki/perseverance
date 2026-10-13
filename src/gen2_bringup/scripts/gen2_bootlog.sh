#!/bin/bash
# Boot report (gen2-bootlog.service, every boot, GEN2_BOOTLOG_DELAY s after start, default 90):
#   ~/gen2_ws/logs/boot/<date>_boot<idx>.txt
# - how the PREVIOUS boot ended (clean shutdown / reboot vs. power loss or hang: its last journal lines)
# - boot history (the clock starts in 1970 on every boot: no RTC, NTP sets it once Wi-Fi is up)
# - this boot: kernel errors, failed units, gen2 services and their errors, boot timing, network, DDS mode,
#   CAN (state, error counters, bus-off), USB / serial / camera devices, motors (motors/state), batteries,
#   Jetson power mode, temperatures, throttling, disk, and the start of the health events log.
# A boot that never reaches this point leaves no report: the next report shows it as "previous boot ended abruptly"
# or as a gap in the boot history.
set -u
sleep "${GEN2_BOOTLOG_DELAY:-90}"
U=parksudo
H=/home/$U
OUT_DIR=$H/gen2_ws/logs/boot
mkdir -p "$OUT_DIR"
IDX=$(journalctl --list-boots --no-pager 2>/dev/null | wc -l)
OUT="$OUT_DIR/$(date +%Y-%m-%d_%H%M%S)_boot$IDX.txt"
exec > "$OUT" 2>&1
sec() { echo; echo "=================== $*"; }

echo "C-WANG boot report  $(date '+%F %T')  boot id $(cat /proc/sys/kernel/random/boot_id)"
echo "uptime $(cut -d' ' -f1 /proc/uptime) s, kernel $(uname -r), clock synced: $(timedatectl show -p NTPSynchronized --value 2>/dev/null)"

sec "previous boot: how it ended"
NOISE="ddsi_udp_conn_write|scan_gate.*passed .* scans|not multicast-capable"
PREV=$(journalctl -b -1 --no-pager -o short-iso 2>/dev/null | tail -n 3000 | grep -vE "$NOISE" | tail -n 40)
echo "last journal entry of the previous boot: $(journalctl -b -1 --no-pager -o short-iso -n 1 2>/dev/null | cut -c1-25)"
echo "DDS write-failure spam in the previous boot: $(journalctl -b -1 --no-pager 2>/dev/null | grep -c ddsi_udp_conn_write) lines (unreachable dds_peers entries)"
if echo "$PREV" | grep -qE "systemd-shutdown|Journal stopped|Reached target.*(Power-Off|Reboot|Shutdown)|Shutting down"; then
  echo "-> CLEAN shutdown / reboot"
else
  echo "-> ABRUPT end (power loss, brown-out, hang or hard reset) — last lines below"
fi
echo "$PREV" | tail -n 25 | cut -c1-220
echo "-- previous boot errors (priority err, last 20)"
journalctl -b -1 -p err --no-pager -o short-iso 2>/dev/null | tail -n 20 | cut -c1-220
echo "-- pstore (kernel panic / oops records)"
ls -la /sys/fs/pstore/ 2>/dev/null; for f in /sys/fs/pstore/*; do [ -f "$f" ] && { echo "## $f"; tail -n 30 "$f"; }; done

sec "boot history (last 10; 1970 start = before NTP)"
journalctl --list-boots --no-pager 2>/dev/null | tail -n 10
last -x -n 12 reboot shutdown 2>/dev/null | head -n 12

sec "this boot: kernel errors / warnings of interest"
journalctl -k -b -p warning --no-pager 2>/dev/null | grep -viE "audit|apparmor" | tail -n 40 | cut -c1-220
echo "-- undervoltage / throttling / thermal / usb / can / mmc / nvme"
dmesg 2>/dev/null | grep -iE "under.?volt|throttl|overcurrent|thermal|over.?temp|usb .*(disconnect|error|reset)|can0|mcp|m_can|bus-off|nvme|mmc.*error|I/O error|imx219|rplidar|ttyUSB|ttyACM|EXT4-fs error" | tail -n 40 | cut -c1-200

sec "systemd: failed units / gen2 services"
systemctl --failed --no-legend --no-pager
for s in can0-up lcd-pinmux gen2-bench gen2-balance NetworkManager; do
  printf '%-16s %-10s since %s  restarts %s\n' "$s" "$(systemctl is-active $s)" \
    "$(systemctl show -p ActiveEnterTimestamp --value $s)" "$(systemctl show -p NRestarts --value $s)"
done
for s in gen2-bench gen2-balance; do
  echo "-- $s errors this boot"
  journalctl -u $s -b --no-pager 2>/dev/null | grep -iE "error|fatal|traceback|died|exception|FAULT|refused|not multicast-capable" | tail -n 15 | cut -c1-220
done

sec "boot timing"
systemd-analyze 2>/dev/null | head -n 2
systemd-analyze blame --no-pager 2>/dev/null | head -n 12
systemd-analyze critical-chain gen2-balance.service --no-pager 2>/dev/null | head -n 15

sec "network / DDS"
ip -br addr
iw dev wlP1p1s0 link 2>/dev/null | grep -E "SSID|signal|freq|bitrate"
nmcli -t -f NAME,DEVICE,STATE con show --active 2>/dev/null
for f in $H/.ros/gen2_dds_mode.*; do echo "$(basename $f): $(cat $f)"; done
journalctl -t gen2-dds -b --no-pager 2>/dev/null | tail -n 10

sec "CAN"
ip -d -s link show can0 2>&1 | head -n 20

sec "devices"
lsusb 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* /dev/video* /dev/i2c-* 2>/dev/null | awk '{print $NF}' | tr '\n' ' '; echo
ls /dev/shm/ 2>/dev/null | tr '\n' ' '; echo

sec "power / thermal"
nvpmodel -q 2>/dev/null | head -n 2
for h in /sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*; do
  for k in 1 2 3; do l=$(cat $h/in${k}_label 2>/dev/null) && echo "$l: $(cat $h/in${k}_input 2>/dev/null) mV  $(cat $h/curr${k}_input 2>/dev/null) mA"; done
done
for z in /sys/class/thermal/thermal_zone*; do printf '%s=%s ' "$(cat $z/type)" "$(( $(cat $z/temp) / 1000 ))"; done; echo
df -h / | tail -n 1
free -m | head -n 2

sec "ROS: motors, batteries, controller (as user $U)"
sudo -u $U bash -c '
source /opt/ros/jazzy/setup.bash; source '"$H"'/gen2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$('"$H"'/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh)
echo "nodes: $(timeout 10 ros2 node list 2>/dev/null | tr "\n" " ")"
timeout 6 ros2 topic echo --once /motors/state 2>/dev/null | grep -E "^  name|stale|temperature_c|error_text|age_s" | paste - - - - - | sed "s/  */ /g"
for b in compute motor; do echo "$b: $(timeout 5 ros2 topic echo --once /power/$b 2>/dev/null | grep -E "^(voltage|percentage|present)" | tr "\n" " ")"; done
timeout 5 ros2 topic echo --once /controller/state 2>/dev/null | grep -E "^(mode|fault|imu_age_ms|overruns)" | tr "\n" " "; echo
'

sec "health events so far (logs/health, newest)"
EV=$(ls -t $H/gen2_ws/logs/health/*_events.log 2>/dev/null | head -n 1)
[ -n "$EV" ] && { echo "$EV"; head -n 40 "$EV"; }
chown -R $U: "$OUT_DIR"
