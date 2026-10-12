#!/bin/bash
# 밤새 점프 RL: jr3flat 기다림 → 평가 → jr4 (다리 ±12 cm) → 평가 → jr5 (턱) → 평가 → ONNX
cd ~/wheeled_biped_isaaclab
N=~/.claude/jobs/fcc5fea5/tmp/night; S=$N/summary.txt
LR=logs/rsl_rl/wheeled_biped_jump
IL=~/Desktop/IsaacLab/isaaclab.sh
FLAT="--p_flat 1.0 --v_min 0.85 --v_max 0.97 --edge_min 2.0 --edge_max 2.5"
STEP="--p_flat 0.0 --v_min 0.3 --v_max 0.6 --edge_min 1.5 --edge_max 2.0"
log() { echo "$(date +%H:%M) $*" >> $S; }
last_model() { ls -t $(ls -td $LR/*_$1 | head -1)/model_*.pt 2>/dev/null | head -1; }
wait_train() {  # run iters
  until grep -qE "Learning iteration $(( $2 - 1 ))/|학습 끝|Traceback" $N/$1.log ~/.claude/jobs/fcc5fea5/tmp/$1.log 2>/dev/null; do sleep 20; done
  sleep 40
  for p in $(pgrep -f -- "--run $1( |$)"); do kill -9 $p; done; sleep 5
}
train() {  # run iters resume_model args...
  run=$1; it=$2; rs=$3; shift 3
  log "학습 $run 시작 ($it iter, 이어서: ${rs:-없음}) $*"
  $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 2048 --iters $it --run $run ${rs:+--resume $rs} "$@" < /dev/null > $N/$run.log 2>&1 &
  wait_train $run $it
  log "학습 $run 끝: $(last_model $run)"
}
evaluate() {  # 이름 모델 args...
  nm=$1; m=$2; shift 2
  timeout 1200 $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 512 --eval $m --eval_episodes 1024 "$@" < /dev/null > $N/eval_$nm.log 2>&1
  r=$(grep "점프 평가" $N/eval_$nm.log | tail -1); log "평가 $nm: ${r:-실패 (eval_$nm.log)}"
  for p in $(pgrep -f "eval_episodes 1024"); do kill -9 $p; done; sleep 5
}
succ() { grep "점프 평가" $N/eval_$1.log | grep -oE "success [0-9.]+" | grep -oE "[0-9.]+" || echo 0; }

: > $S
log "밤 작업 시작"
wait_train jr3flat 1500
M3=$(last_model jr3flat)
evaluate base_flat base $FLAT
evaluate jr3_flat $M3 $FLAT
train jr4wide 1000 $M3 $FLAT --res_leg 0.12
M4=$(last_model jr4wide)
evaluate jr4_flat $M4 $FLAT --res_leg 0.12
if awk "BEGIN{exit !($(succ jr4_flat) >= $(succ jr3_flat))}"; then BEST=$M4; RL=0.12; else BEST=$M3; RL=0.04; fi
log "평지 최고: $BEST (res_leg $RL)"
train jr5step 1500 $BEST --p_flat 0.3 --h_min 0.04 --h_max 0.10 --v_min 0.3 --v_max 0.97 --edge_min 1.5 --edge_max 2.5 --res_leg $RL
M5=$(last_model jr5step)
evaluate base_h6 base $STEP --h 0.06
evaluate base_h8 base $STEP --h 0.08
evaluate jr5_h6 $M5 $STEP --h 0.06 --res_leg $RL
evaluate jr5_h8 $M5 $STEP --h 0.08 --res_leg $RL
evaluate jr5_flat $M5 $FLAT --res_leg $RL
timeout 900 $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 16 --eval $BEST --export $N/onnx_flat --res_leg $RL < /dev/null > $N/export_flat.log 2>&1
timeout 900 $IL -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 16 --eval $M5 --export $N/onnx_step --res_leg $RL < /dev/null > $N/export_step.log 2>&1
log "ONNX: $(ls $N/onnx_*/policy.onnx 2>/dev/null | tr '\n' ' ')"
log "밤 작업 끝"
