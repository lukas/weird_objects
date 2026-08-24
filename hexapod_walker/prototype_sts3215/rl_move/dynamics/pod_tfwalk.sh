#!/bin/sh
# pod_tfwalk.sh — Transformer-encoder walking/heading A/B/C transfer
# arm (operator order 20260815T221231Z, filed via the MCP operator
# lane: "launch matched real walking/heading PPO A/B/C using THIS
# checkpoint" = cw-dynrep-tf-state2-recovered1, the first
# G1/G1.1+G3-passing TRANSFORMER dynrep encoder — no substitution of
# dyn_scale_M_h16_large or any older encoder).
#
# Design: matched to the GRU futurewalk walking benchmark
# (dynrep-futurewalk-C-s5, wandb ino3k1mk): task walk (commanded
# velocity/heading tracking, the campaign walk goal mode), 1M steps,
# n_envs 8 (trainer default), dr_scale 0.3 (default), eval every 10k
# on rise,hold,walk + --eval-heldout dynamics-mismatch suites,
# term_penalty 30 (default), seed 5. ONE condition + ONE seed per pod
# (the 08-15 OOM lesson: never multiply encoder+dataset loads on one
# pod). Conditions:
#   A  scratch     MlpPolicy on raw stacked obs, no encoder
#   B  frozen      pretrained TF encoder -> frozen z -> PPO heads
#   C  anchored    encoder fine-tunes at 0.1x LR + 4x256 dynamics-loss
#                  anchor batches per rollout on the ORIGINAL
#                  v5_mjx_fresh pretraining corpus (condition C is the
#                  only one that needs the dataset on-pod)
#
# Usage (per pod):
#   COND=A SEED=5 nohup sh rl_move/dynamics/pod_tfwalk.sh \
#       > rl_move/dynamics/logs/pod_tfwalk_A.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
COND=${COND:?set COND=A|B|C}
SEED=${SEED:-5}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
WALK_STEPS=${WALK_STEPS:-1000000}
EVAL_EVERY=${EVAL_EVERY:-10000}
COHORT_NAME=${COHORT_NAME:-tfwalk-metrics1}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/${COHORT_NAME}_manifest.jsonl"
NAME="dynrep-${COHORT_NAME}-${COND}-s${SEED}"
mkdir -p "$LOG"
echo "== pod_tfwalk start $(date -u +%FT%TZ) host=$(hostname)" \
     "cond=$COND seed=$SEED steps=$WALK_STEPS enc=$ENC"

# Preconditions. A needs no encoder file, but the cohort's premise is
# the G1/G1.1 gate on the TF encoder, so every pod checks the gate
# record (DYNREP.md hard gate: no PPO before G1 passes).
grep -Eq "GATE G1(\.1)? .*PASS" "$LOG/$(basename "$ENC" .pt)_gate.txt" \
    2>/dev/null || {
    echo "POD_TFWALK_ABORT: no G1/G1.1 PASS on record for $ENC"; exit 3; }
if [ "$COND" != "A" ]; then
    [ -f "$ENC" ] || { echo "POD_TFWALK_ABORT: encoder $ENC missing"; exit 3; }
fi
if [ "$COND" = "C" ]; then
    [ -d "$DATA" ] || { echo "POD_TFWALK_ABORT: anchor dataset $DATA missing"; exit 3; }
fi
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *train_ppo_transfer*)
        echo "POD_TFWALK_ABORT: train_ppo_transfer already running on $(hostname)"
        exit 4;;
    esac
done

printf '{"event":"start","cohort":"%s","host":"%s","utc":"%s","condition":"%s","seed":%s,"encoder":"%s","data":"%s","walk_steps":%s}\n' \
    "$COHORT_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$COND" "$SEED" "$ENC" "$DATA" "$WALK_STEPS" \
    >> "$MANIFEST"
printf '{"event":"phase_start","cohort":"%s","seed":%s,"condition":"%s","task":"walk","utc":"%s"}\n' \
    "$COHORT_NAME" "$SEED" "$COND" "$(date -u +%FT%TZ)" >> "$MANIFEST"
OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
    --condition "$COND" --task walk --seed "$SEED" \
    --steps "$WALK_STEPS" --eval-every "$EVAL_EVERY" \
    --eval-tasks rise,hold,walk --eval-heldout \
    --encoder "$ENC" --anchor-data "$DATA" \
    --device cuda \
    --name "$NAME"
printf '{"event":"phase_done","cohort":"%s","seed":%s,"condition":"%s","task":"walk","utc":"%s"}\n' \
    "$COHORT_NAME" "$SEED" "$COND" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_TFWALK_DONE $(date -u +%FT%TZ)"
printf '{"event":"done","cohort":"%s","utc":"%s","seeds_done":1,"condition":"%s"}\n' \
    "$COHORT_NAME" "$(date -u +%FT%TZ)" "$COND" >> "$MANIFEST"
