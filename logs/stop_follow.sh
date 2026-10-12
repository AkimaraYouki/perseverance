#!/bin/bash
# Stops the whole follow stack (process group written by run_follow.sh, children included).
L=$HOME/gen2_ws/logs
[ -f $L/follow.pgid ] || { echo "no follow.pgid"; exit 0; }
G=$(cat $L/follow.pgid)
kill -INT -- -$G 2>/dev/null; sleep 3; kill -TERM -- -$G 2>/dev/null; rm -f $L/follow.pgid $L/follow.pids
echo "stopped pgid $G"
