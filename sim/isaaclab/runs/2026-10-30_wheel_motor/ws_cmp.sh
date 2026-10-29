#!/bin/bash
# 2026-10-30 바퀴 모터 비교 2: 최고 속도 (평지 1.5 / 2.5 m/s 명령, 험지 1.0 m/s) + 외란 (서 있는 채 앞 밀기 0.6~1.5 m/s)
D=$HOME/.claude/jobs/fcc5fea5/tmp/kt; IL=$HOME/Desktop/IsaacLab/isaaclab.sh; cd $HOME/wheeled_biped_isaaclab
run() { m=$1; n=$2; shift 2
  if [ $m = ak40 ]; then E="WB_WHEEL_PEAK_NM=4.1 WB_WHEEL_NOLOAD_RAD_S=45.55"; M="--wheel_tau_max 4.1 --wheel_cmd_max_nm 4.1"; else E=""; M=""; fi
  for t in 1 2; do env $E WB_WHEEL_STATS=$D/cmp_${m}_$n.npz timeout 2400 env -u PYTHONPATH $IL -p scripts/_isaaclab_launch.py scripts/harsh_eval.py "$@" $M < /dev/null > $D/cmp_${m}_$n.log 2>&1; grep -q "바퀴 통계" $D/cmp_${m}_$n.log && break; done
  echo "$m $n | $(grep -E '^  합계' $D/cmp_${m}_$n.log | sed 's/  */ /g') | $(grep -oE '도달 속도.*' $D/cmp_${m}_$n.log) | $(grep -A2 '바퀴 통계' $D/cmp_${m}_$n.log | sed -n 2p | sed 's/  */ /g')" >> $D/cmp.txt; }
for m in ak45 ak40; do
  run $m top15 --terrains flat --no_push --vx 1.5 --vx_slow 1.5 --vmax_kmh 12 --num_envs 2048 --seconds 12 --yaw0
  run $m top25 --terrains flat --no_push --vx 2.5 --vx_slow 2.5 --vmax_kmh 12 --num_envs 2048 --seconds 12 --yaw0
  run $m rough10 --vx 1.0 --vx_slow 0.4 --vmax_kmh 12 --num_envs 4096 --seconds 20
  for pv in 0.6 0.9 1.2 1.5; do run $m push$pv --terrains flat --vx 0.0 --vx_slow 0.0 --push_v $pv --num_envs 2048 --seconds 20; done
done
echo DONE >> $D/cmp.txt
