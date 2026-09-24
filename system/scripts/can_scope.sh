#!/bin/bash
# Repeating CAN frames on can0 for oscilloscope measurement.
# Usage: can_scope.sh [loopback|normal] [bitrate] [interval_ms]
#   loopback : no other node needed (no ACK required)
#   normal   : real bus with another node that ACKs
MODE=${1:-loopback}; BITRATE=${2:-500000}; INTERVAL=${3:-10}
[ "$MODE" = loopback ] && LB=on || LB=off
sudo ip link set can0 down
sudo ip link set can0 type can bitrate $BITRATE restart-ms 100 loopback $LB
sudo ip link set can0 up
echo "can0: $BITRATE bit/s, loopback=$LB, 1 frame / ${INTERVAL} ms  (Ctrl+C to stop)"
echo "frame: ID 0x555, data 55 AA 55 AA 00 FF 00 FF"
trap 'echo; ip -d -s link show can0 | sed -n "3,4p;/RX:/,+3p"; sudo systemctl restart can0-up.service; echo "restored can0 (normal mode)"; exit' INT TERM
while :; do cansend can0 555#55AA55AA00FF00FF || sleep 0.1; sleep $(awk "BEGIN{print $INTERVAL/1000}"); done
