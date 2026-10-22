#!/bin/bash
# Gen2: keep Wi-Fi up. If wlP1p1s0 has no active connection, bring up the preferred profile.
# Preferred profile = highest autoconnect-priority wifi profile (currently "robot", 200).
IF=wlP1p1s0
state=$(nmcli -g GENERAL.STATE d show "$IF" 2>/dev/null)
case "$state" in
  100*) exit 0 ;;                               # connected
  30*|20*) : ;;                                 # disconnected / unavailable -> try
  *) exit 0 ;;                                  # connecting etc.: leave NM alone
esac
nmcli radio wifi on
for c in $(nmcli -t -f NAME,TYPE,AUTOCONNECT-PRIORITY c show | awk -F: '$2=="802-11-wireless"{print $3":"$1}' | sort -t: -k1,1nr | cut -d: -f2-); do
  if nmcli -t -f SSID d wifi list ifname "$IF" --rescan auto 2>/dev/null | grep -qx "$(nmcli -g 802-11-wireless.ssid c show "$c")"; then
    logger -t gen2-wifi-watchdog "wlan $state -> nmcli c up $c"
    nmcli -w 30 c up "$c" ifname "$IF" && exit 0
  fi
done
logger -t gen2-wifi-watchdog "wlan $state, no saved SSID visible"
