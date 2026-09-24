#!/bin/bash
# Boot-time WiFi diagnostics for AX210 swap. Log: /home/parksudo/wifi_diag.log
exec > /home/parksudo/wifi_diag.log 2>&1
sleep 25
echo "== boot: $(uptime -s)"
echo "== PCIe"; lspci -nn
echo "== USB (AX210 BT = 8087:0032)"; lsusb
echo "== modules"; lsmod | grep -E 'iwl|mac80211|cfg80211|rtl8822|btintel|btusb'
echo "== iwlwifi module file: $(modinfo -n iwlwifi 2>&1)"
echo "== dmesg"; dmesg | grep -iE 'iwl|8086|0001:0|Phy link|link up|firmware|btintel' | tail -60
echo "== links"; ip -br link
echo "== rfkill"; rfkill list
echo "== nmcli"; nmcli dev status; nmcli -f SSID,SIGNAL,SECURITY dev wifi list 2>&1 | head -15
