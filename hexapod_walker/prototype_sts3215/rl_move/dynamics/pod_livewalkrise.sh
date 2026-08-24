#!/bin/sh
# pod_livewalkrise.sh — cw-dynrep-livewalkrise1 (condition F, operator
# order fb_20260817T210422_9df9c7 arm A): command-rich LIVE CUDA
# collection + online transformer training with stratified 75/25
# walk/rise replay (75% fresh + 25% recovered-v5 rehearsal per batch)
# and a VERSIONED critic snapshot that may change no faster than
# --snap-boundary-steps PPO-step boundaries, only behind the full gate
# battery (generic heldout retention, command-rich live walk
# improvement, live rise retention, latent drift, critic value jump).
# Starts as exact frozen D; the 13.62M pretrained transformer capacity
# is unchanged; the actor is raw-obs only (zero action-KL asserted
# in-process).
#
# Canary invocation (mechanism gates, small boundary to exercise the
# accept/reject paths mechanically):
#   RUN_NAME=cw-dynrep-livewalkrise1-canary1 STEPS=150000 \
#   BOUNDARY=50000 EVAL_EVERY=25000 HELDOUT_EVERY=50000 \
#   CKPT_EVERY=100000 nohup sh rl_move/dynamics/pod_livewalkrise.sh \
#     > rl_move/dynamics/logs/pod_livewalkrise_canary1.log 2>&1 &
# Continuation (the science run, launched only on canary PASS):
#   RUN_NAME=cw-dynrep-livewalkrise1 STEPS=10000000 \
#   nohup sh rl_move/dynamics/pod_livewalkrise.sh \
#     > rl_move/dynamics/logs/pod_livewalkrise1.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
RUN_NAME=${RUN_NAME:?set RUN_NAME}
SEED=${SEED:-5}
STEPS=${STEPS:-10000000}
BOUNDARY=${BOUNDARY:-1000000}
EVAL_EVERY=${EVAL_EVERY:-100000}
HELDOUT_EVERY=${HELDOUT_EVERY:-500000}
CKPT_EVERY=${CKPT_EVERY:-500000}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
ENC_MD5=${ENC_MD5:-9df48f687967c25085ee50171e4110ff}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
RISE_BANK=${RISE_BANK:-rl_move/sim/park_banks/footlow2_hard1_lower_endpoints.npz}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/livewalkrise_manifest.jsonl"
mkdir -p "$LOG"
echo "== pod_livewalkrise start $(date -u +%FT%TZ) host=$(hostname)" \
     "run=$RUN_NAME seed=$SEED steps=$STEPS boundary=$BOUNDARY"

# Preconditions: H200 CUDA, G1/G1.1-gated encoder, corpus, rise bank.
nvidia-smi --query-gpu=name --format=csv,noheader | grep -q H200 || {
    echo "POD_LIVEWALKRISE_ABORT: no H200 GPU"; exit 3; }
grep -Eq "GATE G1(\.1)? .*PASS" \
    "$LOG/$(basename "$ENC" .pt)_gate.txt" 2>/dev/null || {
    echo "POD_LIVEWALKRISE_ABORT: no G1/G1.1 PASS for $ENC"; exit 3; }
[ -f "$ENC" ] || { echo "POD_LIVEWALKRISE_ABORT: encoder missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_LIVEWALKRISE_ABORT: corpus missing"; exit 3; }
[ -f "$RISE_BANK" ] || {
    echo "POD_LIVEWALKRISE_ABORT: rise start bank missing"; exit 3; }
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.dynamics.train_ppo_transfer*)
        echo "POD_LIVEWALKRISE_ABORT: trainer already running"; exit 4;;
    esac
done

printf '{"event":"start","run":"%s","host":"%s","utc":"%s","seed":%s,"steps":%s,"boundary":%s,"encoder_md5":"%s"}\n' \
    "$RUN_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$SEED" "$STEPS" \
    "$BOUNDARY" "$ENC_MD5" >> "$MANIFEST"
set +e
OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
    --condition F --task walkrise --seed "$SEED" \
    --steps "$STEPS" --eval-every "$EVAL_EVERY" \
    --eval-tasks rise,walk,walkrise --eval-heldout \
    --heldout-every "$HELDOUT_EVERY" \
    --checkpoint-every "$CKPT_EVERY" \
    --encoder "$ENC" --encoder-md5 "$ENC_MD5" \
    --anchor-data "$DATA" --device cuda \
    --snap-boundary-steps "$BOUNDARY" \
    --gate-heldout-band 0.15 --gate-live-improve 0.0 \
    --gate-rise-band 0.05 --gate-value-jump 0.10 \
    --live-walk-frac 0.75 --rehearsal-frac 0.25 \
    --goal-set walk_cmd_resample_s=4.0 \
    --goal-set walk_cmd_resample_jitter=0.5 \
    --goal-set walk_stop_frac=0.15 \
    --goal-set walk_heading_max_rad=3.14159265 \
    --goal-set walk_yaw_cmd=1 \
    --goal-set walk_yaw_zero_frac=0.35 \
    --goal-set rise_start_bank="$RISE_BANK" \
    --goal-set rise_start_bank_frac=0.30 \
    --name "$RUN_NAME"
RC=$?
set -e
if [ "$RC" -ne 0 ]; then
    printf '{"event":"phase_fail","run":"%s","exit_code":%s,"utc":"%s"}\n' \
        "$RUN_NAME" "$RC" "$(date -u +%FT%TZ)" >> "$MANIFEST"
    echo "POD_LIVEWALKRISE_FAIL rc=$RC $(date -u +%FT%TZ)"
    exit "$RC"
fi
printf '{"event":"done","run":"%s","utc":"%s"}\n' \
    "$RUN_NAME" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_LIVEWALKRISE_DONE $(date -u +%FT%TZ)"
