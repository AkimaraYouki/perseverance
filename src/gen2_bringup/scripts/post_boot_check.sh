#!/bin/bash
# One-shot post-reboot verification. Log: /home/parksudo/boot_check.log, LCD: ~/boot_lcd.png
exec > /home/parksudo/boot_check.log 2>&1
sleep 45
echo "== boot: $(uptime -s)   check: $(date '+%F %T')"
for u in can0-up lcd-pinmux gen2-bench NetworkManager; do echo "service $u: $(systemctl is-active $u)"; done
echo "== can0"; ip -d link show can0 | sed -n '3,4p'
echo "== wifi"; lspci -nn | grep -i network; ip -br addr | grep wl; ping -c2 -W3 8.8.8.8 | tail -1
echo "== camera"; ls /dev/video* 2>&1; dmesg | grep -iE 'imx219' | tail -3
echo "== gen2-bench journal (errors)"; journalctl -u gen2-bench -b --no-pager | grep -iE 'error|died|traceback|fatal' | tail -10 | cut -c1-200
echo "== bench check"
sudo -u parksudo bash -c 'source /opt/ros/jazzy/setup.bash; source /home/parksudo/gen2_ws/install/setup.bash; export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///home/parksudo/gen2_ws/src/gen2_bringup/config/cyclonedds.xml; ros2 run gen2_bringup gen2_bench_check.py 2>&1 | grep -v multicast'
echo "exit=$?"
cp /tmp/gen2_lcd.png /home/parksudo/boot_lcd.png 2>/dev/null && chown parksudo: /home/parksudo/boot_lcd.png && echo "LCD snapshot saved"
systemctl disable post-boot-check.service
