#!/bin/bash
# 바퀴 모터 선정 통계 (2026-10-30): 험지 0.4 / 0.7 m/s + 계단 점프 (합판 v4, 8 cm 2 단 / 10 cm 단)
D=$HOME/.claude/jobs/fcc5fea5/tmp/kt; IL=$HOME/Desktop/IsaacLab/isaaclab.sh; cd $HOME/wheeled_biped_isaaclab
M="--wheel_tau_max 4.1 --wheel_cmd_max_nm 4.1 --db_comp_nm 0.28"   # AK40-10: 피크 4.1 N·m (7.3 A), 0.5 A 문턱 = 0.28 N·m
run() { n=$1; shift; for t in 1 2; do WB_WHEEL_PEAK_NM=4.1 WB_WHEEL_NOLOAD_RAD_S=45.55 WB_WHEEL_STATS=$D/ws40_$n.npz timeout 2400 env -u PYTHONPATH $IL -p scripts/_isaaclab_launch.py "$@" < /dev/null > $D/ws40_$n.log 2>&1; grep -q "바퀴 통계" $D/ws40_$n.log && break; done; echo "== $n" >> $D/ws40.txt; grep -A6 "바퀴 통계" $D/ws40_$n.log >> $D/ws40.txt; grep -E "^\[가혹 평가\]|점프 평가\]" $D/ws40_$n.log | cut -c1-200 >> $D/ws40.txt; }
run harsh04 scripts/harsh_eval.py --num_envs 4096 --seconds 20 $M
run harsh07 scripts/harsh_eval.py --num_envs 4096 --seconds 20 --vx 0.7 --vx_slow 0.4 $M
P="$(cat $D/jump5_args.txt | sed 's/--accel_max 0.8 --jerk_max 3.0/--accel_max 1.2 --jerk_max 6/') --hip dc --hip_w0 29.3 --hip_cap_nm 15 --v_ki 0.8 --v_boost_max 0.4"
B="$(cat $D/cma11_15_best_args.txt)"
R0="--retry 1 --jump_retry 1 --retry_back_v 0.3 --edge_min 0.6 --edge_max 1.0 --v_min 0.3 --v_max 0.3 $(cat $D/final_stairs_args.txt) --max_tries 3 --after_land_s 1.5 --rv_scale 0.6,1.0 --tread 1.0"
run jump8 scripts/jump_rl.py --num_envs 2048 --eval base --eval_episodes 2048 --p_flat 0 --hip_eff_min 0.5 --hip_eff_max 0.6 --h 0.08 --p_stairs 1 --box_mu 1.0 $P $B $R0 $(cat $D/plywood4_best_args.txt) $M
run jump10 scripts/jump_rl.py --num_envs 2048 --eval base --eval_episodes 2048 --p_flat 0 --hip_eff_min 0.5 --hip_eff_max 0.6 --h 0.10 --p_stairs 0 $P $B $R0 $M
echo DONE >> $D/ws40.txt
