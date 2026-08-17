#!/bin/sh
# pod_criticD40m.sh — cw-dynrep-criticD-40m1 (operator order
# fb_20260817T210422_9df9c7 arm B): fresh-seed 40M-step PPO walking
# run; the EXACT pretrained vt2ovznc transformer (md5-pinned) stays
# FROZEN as critic D throughout; the actor trains from scratch on raw
# obs. Proven walk reward/task stack; conservative tweaks only:
# command diversity / start-stop exposure (existing goal.* keys) and
# checkpoint selection by the pre-registered locomotion-quality
# composite (command progress, body-frame vx/vy+yaw tracking, slip per
# progress meter, roll, falls, slew, contact gait) — never scalar
# reward alone. Rise/hold retention measured at every eval. No online
# predictor.
#
#   nohup sh rl_move/dynamics/pod_criticD40m.sh \
#     > rl_move/dynamics/logs/pod_criticD40m1.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
RUN_NAME=${RUN_NAME:-cw-dynrep-criticD-40m1}
SEED=${SEED:-8}
STEPS=${STEPS:-40000000}
N_ENVS=${N_ENVS:-16}
EVAL_EVERY=${EVAL_EVERY:-250000}
HELDOUT_EVERY=${HELDOUT_EVERY:-1000000}
CKPT_EVERY=${CKPT_EVERY:-2000000}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
ENC_MD5=${ENC_MD5:-9df48f687967c25085ee50171e4110ff}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/criticD40m_manifest.jsonl"
mkdir -p "$LOG"
echo "== pod_criticD40m start $(date -u +%FT%TZ) host=$(hostname)" \
     "run=$RUN_NAME seed=$SEED steps=$STEPS n_envs=$N_ENVS"

nvidia-smi --query-gpu=name --format=csv,noheader | grep -q H200 || {
    echo "POD_CRITICD40M_ABORT: no H200 GPU"; exit 3; }
grep -Eq "GATE G1(\.1)? .*PASS" \
    "$LOG/$(basename "$ENC" .pt)_gate.txt" 2>/dev/null || {
    echo "POD_CRITICD40M_ABORT: no G1/G1.1 PASS for $ENC"; exit 3; }
[ -f "$ENC" ] || { echo "POD_CRITICD40M_ABORT: encoder missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_CRITICD40M_ABORT: corpus missing"; exit 3; }
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.dynamics.train_ppo_transfer*)
        echo "POD_CRITICD40M_ABORT: trainer already running"; exit 4;;
    esac
done

printf '{"event":"start","run":"%s","host":"%s","utc":"%s","seed":%s,"steps":%s,"n_envs":%s,"encoder_md5":"%s"}\n' \
    "$RUN_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$SEED" "$STEPS" \
    "$N_ENVS" "$ENC_MD5" >> "$MANIFEST"
set +e
OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
    --condition D --task walk --seed "$SEED" \
    --steps "$STEPS" --n-envs "$N_ENVS" \
    --eval-every "$EVAL_EVERY" --eval-episodes 6 \
    --eval-tasks rise,hold,walk --eval-heldout \
    --heldout-every "$HELDOUT_EVERY" \
    --checkpoint-every "$CKPT_EVERY" \
    --select-quality \
    --encoder "$ENC" --encoder-md5 "$ENC_MD5" \
    --anchor-data "$DATA" --device cuda \
    --goal-set walk_cmd_resample_s=4.0 \
    --goal-set walk_cmd_resample_jitter=0.5 \
    --goal-set walk_stop_frac=0.15 \
    --name "$RUN_NAME"
RC=$?
set -e
if [ "$RC" -ne 0 ]; then
    printf '{"event":"phase_fail","run":"%s","exit_code":%s,"utc":"%s"}\n' \
        "$RUN_NAME" "$RC" "$(date -u +%FT%TZ)" >> "$MANIFEST"
    echo "POD_CRITICD40M_FAIL rc=$RC $(date -u +%FT%TZ)"
    exit "$RC"
fi
printf '{"event":"done","run":"%s","utc":"%s"}\n' \
    "$RUN_NAME" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_CRITICD40M_DONE $(date -u +%FT%TZ)"
