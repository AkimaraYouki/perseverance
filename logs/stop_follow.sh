#!/bin/bash
# Stops the whole follow stack: the process group in follow.pgid (written by run_follow*.sh, children included) and any
# other run_follow*.sh group still running (GCS 2026-10-29: a stack whose pgid file was gone kept running next to a new
# one -> two /person_follow, two trackers, 11 frames/s).
L=$HOME/gen2_ws/logs
G=$(cat $L/follow.pgid 2>/dev/null)
ALL=$( { echo "$G"; ps -eo pgid=,args= | awk '$2 ~ /bash$/ && $3 ~ /run_follow(_cam)?\.sh$/ {print $1}'; } | grep -v '^ *$' | sort -u)
[ -z "$ALL" ] && { echo "no follow stack"; rm -f $L/follow.pgid $L/follow.pids; exit 0; }
for g in $ALL; do kill -INT -- -$g 2>/dev/null; done
sleep 3
for g in $ALL; do kill -TERM -- -$g 2>/dev/null; done
rm -f $L/follow.pgid $L/follow.pids
echo "stopped pgid" $ALL
