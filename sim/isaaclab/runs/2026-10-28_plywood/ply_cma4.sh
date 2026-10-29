#!/bin/bash
# 2026-10-28 대회 밤: 합판 (box_mu 0.7) 2 단 8 cm (2 일차 8 cm 만) 에서 '수직 착지' 최적화 -> 검증 (mu 0.5/0.7/1.0 x 6/8 cm, 기본 vs 새 값)
D=$HOME/.claude/jobs/fcc5fea5/tmp/kt; IL=$HOME/Desktop/IsaacLab/isaaclab.sh; cd $HOME/wheeled_biped_isaaclab
P="$(cat $D/jump5_args.txt | sed 's/--accel_max 0.8 --jerk_max 3.0/--accel_max 1.2 --jerk_max 6/') --hip dc --hip_w0 29.3 --hip_cap_nm 15 --v_ki 0.8 --v_boost_max 0.4"
B="$(cat $D/cma11_15_best_args.txt)"
R0="--retry 1 --jump_retry 1 --retry_back_v 0.3 --edge_min 0.6 --edge_max 1.0 --v_min 0.3 --v_max 0.3 $(cat $D/final_stairs_args.txt) --max_tries 3 --after_land_s 1.5 --rv_scale 0.6,1.0 --tread 1.0"
for t in 1 2; do env -u PYTHONPATH $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 2048 --cma 18 --cma_pop 14 --cma_sigma 0.05 --cma_eps 3 --cma_space plywood3 --cma_w_fell 2.0 --cma_w_lpitch 1.0 --cma_w_back 0.5 --run cma_plywood4 --course s0.08:1.0 --box_mu 0.7 --p_flat 0 --hip_eff_min 0.45 --hip_eff_max 0.65 $P $B $R0 $(cat $D/plywood_best_args.txt) < /dev/null > $D/cma_plywood4.log 2>&1 && break; done
BP=$(python3 -c "import json,glob; d=json.load(open(sorted(glob.glob('logs/jump_cma/*cma_plywood4/best.json'))[-1]))['best']; print(' '.join(f'--{k} {v}' for k,v in d.items()))"); echo "$BP" > $D/plywood4_best_args.txt; cp $D/plywood4_best_args.txt $D/plywood_best_args.txt; echo "== best: $BP" >> $D/ply_cma4.txt
for mu in 0.5 0.7 1.0; do for h in 0.08; do
  for t in 1 2; do timeout 2400 env -u PYTHONPATH $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 2048 --eval base --eval_episodes 2048 --p_flat 0 --hip_eff_min 0.5 --hip_eff_max 0.6 --h $h --p_stairs 1 --box_mu $mu --trace_envs 64 --dump $D/p4_${mu}_$h.npy $P $B $R0 $(cat $D/plywood_best_args.txt) < /dev/null > $D/p4_${mu}_$h.log 2>&1; grep -q "점프 평가" $D/p4_${mu}_$h.log && break; done
  echo "새값 mu $mu 2단 $h: $(grep "점프 평가" $D/p4_${mu}_$h.log | sed 's/.*fell/fell/; s/ | 바퀴.*//') | $(grep -oE "넘어짐 중 앞으로 [0-9]+%" $D/p4_${mu}_$h.log) | $(python3 $D/phase_stats.py $D/p4_${mu}_${h}_trace.npy 2>/dev/null | grep -oE 'LAND 시작 pitch [-0-9.]+|착지뒤 최소 v [-0-9.]+' | tr '\n' ' ')" >> $D/ply_cma4.txt
done; done
echo DONE >> $D/ply_cma4.txt
