#!/bin/bash
# Write the CycloneDDS config for the current network state and print its path.
#   wired (enP8p1s0) with IPv4 -> added, priority 20, multicast OFF (campus LAN: discovery only
#                                 through dds_peers.txt, so other ROS users on the LAN do not see
#                                 or command the robot)
#   Wi-Fi (wlP1p1s0) with IPv4 -> added, priority 10, multicast for SPDP discovery only (when the
#                                 wired interface is present Cyclone turns multicast off: peers only)
#   neither                    -> local mode (lo only)
# A loopback entry next to a real interface became Cyclone 0.10's primary interface and broke all
# off-board traffic, and Cyclone does not fail over when an address disappears — so the choice is
# made here at start-up and the NetworkManager hook restarts gen2-bench on address changes.
# Env: GEN2_DDS_IFACE (Wi-Fi, default wlP1p1s0), GEN2_DDS_WIRED (default enP8p1s0),
#      GEN2_DDS_OUT (default ~/.ros/gen2_cyclonedds.xml),
#      GEN2_DDS_PEERS (default ~/gen2_ws/src/gen2_bringup/config/dds_peers.txt, falls back to share/)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SHARE="$HERE/../../share/gen2_bringup/config"
WIFI=${GEN2_DDS_IFACE:-wlP1p1s0}
WIRED=${GEN2_DDS_WIRED:-enP8p1s0}
OUT=${GEN2_DDS_OUT:-$HOME/.ros/gen2_cyclonedds.xml}
PEERS=${GEN2_DDS_PEERS:-$HOME/gen2_ws/src/gen2_bringup/config/dds_peers.txt}
[ -f "$PEERS" ] || PEERS="$SHARE/dds_peers.txt"
mkdir -p "$(dirname "$OUT")"
TMP="$OUT.$$"

ipv4_of() { ip -4 -o addr show dev "$1" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1; }
WIRED_IP=$(ipv4_of "$WIRED")
WIFI_IP=$(ipv4_of "$WIFI")

IFACES=""
MODE=""
if [ -n "$WIRED_IP" ]; then
  IFACES="$IFACES        <NetworkInterface name=\"$WIRED\" priority=\"20\" multicast=\"false\"/>\n"
  MODE="wired"
fi
if [ -n "$WIFI_IP" ]; then
  IFACES="$IFACES        <NetworkInterface name=\"$WIFI\" priority=\"10\"/>\n"
  MODE="${MODE:+$MODE+}wifi"
fi

if [ -z "$MODE" ]; then
  MODE=local
  cp "$SHARE/cyclonedds_local.xml" "$TMP"
else
  P=""
  if [ -f "$PEERS" ]; then
    while read -r addr _; do
      case "$addr" in ''|\#*) continue ;; esac
      P="$P        <Peer address=\"$addr\"/>\n"
    done < "$PEERS"
  fi
  # With the wired interface present it is the primary one and its multicast is off, which makes
  # Cyclone disable multicast entirely -> local participants find each other through our own IP.
  if [ -n "$WIRED_IP" ]; then
    P="$P        <Peer address=\"$WIRED_IP\"/>\n"
  fi
  [ -n "$P" ] && P="      <Peers>\n$P      </Peers>"
  sed -e "s|@INTERFACES@|$IFACES|" -e "s|@PEERS@|$P|" "$SHARE/cyclonedds_wifi.xml.in" > "$TMP"
fi
mv -f "$TMP" "$OUT"
echo "gen2_dds: $MODE mode (wired ${WIRED_IP:--}, wifi ${WIFI_IP:--}) -> $OUT" >&2
echo "$OUT"
