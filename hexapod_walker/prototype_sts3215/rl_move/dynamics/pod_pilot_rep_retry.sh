#!/bin/sh
# pod_pilot_rep_retry.sh — ONE pre-registered retry of the dynrep pilot
# replication's encoder stage (08-13: dyn_v2pod_obs seed 0 FAILED G1 at
# k=1 ONLY, model_mse 0.1718 vs linear 0.1673 (~2.7%); every other
# horizon passed. Fork this retry decides: PASS => the seed-0 miss was
# training variance, PPO cohort proceeds on the s1 encoder; FAIL =>
# two encoder seeds lose to the linear baseline at k=1 on the SAME pod
# dataset => the dataset drift (noslip actor falling back to tripod,
# collect.py pod-side limitation) is the prime suspect and the fix is
# operator-side (push noslip_gait.py or revise the recipe) — do NOT
# train a third seed.)
# Reuses the existing v2pod dataset; encoder artifacts *_s1 so the
# failed seed-0 gate record stays intact.
#
#   nohup sh rl_move/dynamics/pod_pilot_rep_retry.sh > .../pod_pilot_rep_retry.log &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
DATA=rl_move/dynamics/datasets/v2pod
ENC_NAME=dyn_v2pod_obs_s1
ENC=rl_move/dynamics/models/${ENC_NAME}.pt
LOG=rl_move/dynamics/logs
mkdir -p "$LOG"
echo "== pod_pilot_rep_retry start $(date -u +%FT%TZ) host=$(hostname)"

# encoder, v2 recipe, ONE variable vs the failed stage: --seed 1
OMP_NUM_THREADS=20 $PY -m rl_move.dynamics.train \
    --data "$DATA" --name "$ENC_NAME" --input-set obs \
    --steps 40000 --lr-final-frac 0.05 --seed 1

# gates G1+G2 (obs input set): PPO only on PASS
$PY -m rl_move.dynamics.eval_model --ckpt "$ENC" --data "$DATA" \
    --dump-latents 2>&1 | tee "$LOG/${ENC_NAME}_gate.txt"
grep -q "GATE G1 .*PASS" "$LOG/${ENC_NAME}_gate.txt" || {
    echo "POD_PILOT_REP_RETRY_G1_FAIL: second encoder seed also lost" \
         "to baselines; dataset-drift suspect, operator call next" \
         "(DYNREP.md hard gate; PPO not wired)"; exit 3; }

# A/B/C pilot cohort, seeds 1..3 in parallel (unchanged from
# pod_pilot_rep.sh stage 4 except ENC points at the s1 encoder)
for SEED in 1 2 3; do
    (
        set -e
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task hold --seed "$SEED" \
                --steps 150000 --encoder "$ENC" --anchor-data "$DATA" \
                --name "pilot_hold_${C}_s${SEED}"
        done
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task lower --seed "$SEED" \
                --steps 150000 --encoder "$ENC" --anchor-data "$DATA" \
                --init-from "rl_move/dynamics/models/ppo_pilot_hold_${C}_s${SEED}.zip" \
                --name "pilot_lower_${C}_s${SEED}"
        done
        echo "PILOT_COHORT_DONE seed=$SEED"
    ) > "$LOG/pilot_rep_s${SEED}.log" 2>&1 &
done
wait

n_done=$(grep -l "PILOT_COHORT_DONE" "$LOG"/pilot_rep_s?.log | wc -l)
echo "cohorts done: $n_done/3"
$PY -m rl_move.dynamics.analyze_pilot --seeds 1 2 3 2>&1 \
    | tee "$LOG/pilot_rep_summary.txt"
echo "POD_PILOT_REP_DONE $(date -u +%FT%TZ)"
