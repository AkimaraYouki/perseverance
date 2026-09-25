#!/bin/bash
# Write the CycloneDDS config for the current network state and print its path.
#   Wi-Fi interface has IPv4 -> Wi-Fi mode (wifi only + peers)   else -> local mode (lo only)
# Env: GEN2_DDS_IFACE (default wlP1p1s0), GEN2_DDS_OUT (default ~/.ros/gen2_cyclonedds.xml),
#      GEN2_DDS_PEERS (default ~/gen2_ws/src/gen2_bringup/config/dds_peers.txt, falls back to share/)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SHARE="$HERE/../../share/gen2_bringup/config"
IFACE=${GEN2_DDS_IFACE:-wlP1p1s0}
OUT=${GEN2_DDS_OUT:-$HOME/.ros/gen2_cyclonedds.xml}
PEERS=${GEN2_DDS_PEERS:-$HOME/gen2_ws/src/gen2_bringup/config/dds_peers.txt}
[ -f "$PEERS" ] || PEERS="$SHARE/dds_peers.txt"
mkdir -p "$(dirname "$OUT")"
TMP="$OUT.$$"
if ip -4 -o addr show dev "$IFACE" 2>/dev/null | grep -q ' inet '; then
  MODE=wifi
  P=""
  if [ -f "$PEERS" ]; then
    while read -r addr _; do
      case "$addr" in ''|\#*) continue ;; esac
      P="$P        <Peer address=\"$addr\"/>\n"
    done < "$PEERS"
  fi
  [ -n "$P" ] && P="      <Peers>\n$P      </Peers>"
  sed -e "s|@IFACE@|$IFACE|" -e "s|@PEERS@|$P|" "$SHARE/cyclonedds_wifi.xml.in" > "$TMP"
else
  MODE=local
  cp "$SHARE/cyclonedds_local.xml" "$TMP"
fi
mv -f "$TMP" "$OUT"
echo "gen2_dds: $MODE mode ($IFACE) -> $OUT" >&2
echo "$OUT"
