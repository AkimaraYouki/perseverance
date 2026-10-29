#!/bin/bash
# 2026-10-28 대회: 합판 턱 미끄러움, 2 단 둘째 단 실패·앞으로 고꾸라짐 -> 기본 묶음을 미끄러운 턱 (box_mu) 에서 재현
D=$HOME/.claude/jobs/fcc5fea5/tmp/kt; IL=$HOME/Desktop/IsaacLab/isaaclab.sh; cd $HOME/wheeled_biped_isaaclab
P="$(cat $D/jump5_args.txt | sed 's/--accel_max 0.8 --jerk_max 3.0/--accel_max 1.2 --jerk_max 6/') --hip dc --hip_w0 29.3 --hip_cap_nm 15 --v_ki 0.8 --v_boost_max 0.4"
B="$(cat $D/cma11_15_best_args.txt)"
R0="--retry 1 --jump_retry 1 --retry_back_v 0.3 --edge_min 0.6 --edge_max 1.0 --v_min 0.3 --v_max 0.3 $(cat $D/final_stairs_args.txt) --max_tries 3 --after_land_s 1.5 --rv_scale 0.6,1.0 --tread 1.0"
for mu in 1.0 0.7 0.5; do for h in 0.06 0.08; do
  timeout 2400 env -u PYTHONPATH $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 2048 --eval base --eval_episodes 2048 --p_flat 0 --hip_eff_min 0.5 --hip_eff_max 0.6 --h $h --p_stairs 1 --box_mu $mu --trace_envs 64 --dump $D/ply_${mu}_$h.npy $P $B $R0 < /dev/null > $D/ply_${mu}_$h.log 2>&1
  echo "mu $mu 2단 $h: $(grep '점프 평가' $D/ply_${mu}_$h.log | sed 's/.*fell/fell/; s/ | 바퀴.*//') | $(grep -oE '넘어짐 중 앞으로 [0-9]+%' $D/ply_${mu}_$h.log) | $(python3 $D/phase_stats.py $D/ply_${mu}_${h}_trace.npy 2>/dev/null | grep -oE 'LAND 시작 pitch [-0-9.]+|착지뒤 최소 v [-0-9.]+' | tr '\n' ' ')" >> $D/ply.txt
done; done
echo DONE >> $D/ply.txt
