#!/bin/sh
# pod_criticDwalkcurr.sh — cw-dynrep-criticD-walkcurr1 (operator order
# 2026-08-18, MCP operator lane, GPT-5 Codex for Lukas): CLEAN
# ONE-VARIABLE comparison to cw-dynrep-criticD-40m1. Identical: fresh
# scratch actor (seed 8), EXACT vt2ovznc frozen transformer critic D
# (md5-pinned, no online predictor), PPO architecture / rewards /
# optimizer, 40M steps, n_envs 16, W&B, CUDA, eval cadence + eval
# command distribution (the --goal-set keys below shape EVAL envs only
# under --walk-curriculum, so eval curves stay directly comparable).
# The ONLY variable: TRAINING command sampling switches from the fixed
# broad resample mix to the default-off adaptive competence+retention
# frontier curriculum (goal.walk_curriculum=1, WALKCURR_BUCKETS in
# walk_task.py): cert every 0.5M on deterministic same-backend held-out
# seeds, n>=8/bucket; promotion only when frontier AND every retained
# bucket pass (falls/six-leg gait/progress>=0.75/direction/cross-track/
# slip<=2/roll<=6/slew/stop threshold); promotions saved; 2 consecutive
# retained failures => rollback to the last retention-clean promotion;
# best checkpoint = last retention-clean promotion, never reward/latest.
#
#   nohup sh rl_move/dynamics/pod_criticDwalkcurr.sh \
#     > rl_move/dynamics/logs/pod_criticDwalkcurr1.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
RUN_NAME=${RUN_NAME:-cw-dynrep-criticD-walkcurr1}
SEED=${SEED:-8}
STEPS=${STEPS:-40000000}
N_ENVS=${N_ENVS:-16}
EVAL_EVERY=${EVAL_EVERY:-250000}
HELDOUT_EVERY=${HELDOUT_EVERY:-1000000}
CKPT_EVERY=${CKPT_EVERY:-2000000}
CERT_EVERY=${CERT_EVERY:-500000}
CERT_EPISODES=${CERT_EPISODES:-8}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
ENC_MD5=${ENC_MD5:-9df48f687967c25085ee50171e4110ff}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
WANDB_FLAG=${WANDB_FLAG:-}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/criticDwalkcurr_manifest.jsonl"
mkdir -p "$LOG"
echo "== pod_criticDwalkcurr start $(date -u +%FT%TZ) host=$(hostname)" \
     "run=$RUN_NAME seed=$SEED steps=$STEPS n_envs=$N_ENVS" \
     "cert_every=$CERT_EVERY"

nvidia-smi --query-gpu=name --format=csv,noheader | grep -q H200 || {
    echo "POD_CRITICDWALKCURR_ABORT: no H200 GPU"; exit 3; }
grep -Eq "GATE G1(\.1)? .*PASS" \
    "$LOG/$(basename "$ENC" .pt)_gate.txt" 2>/dev/null || {
    echo "POD_CRITICDWALKCURR_ABORT: no G1/G1.1 PASS for $ENC"; exit 3; }
[ -f "$ENC" ] || { echo "POD_CRITICDWALKCURR_ABORT: encoder missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_CRITICDWALKCURR_ABORT: corpus missing"; exit 3; }
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.dynamics.train_ppo_transfer*)
        echo "POD_CRITICDWALKCURR_ABORT: trainer already running"; exit 4;;
    esac
done

printf '{"event":"start","run":"%s","host":"%s","utc":"%s","seed":%s,"steps":%s,"n_envs":%s,"encoder_md5":"%s","cert_every":%s}\n' \
    "$RUN_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$SEED" "$STEPS" \
    "$N_ENVS" "$ENC_MD5" "$CERT_EVERY" >> "$MANIFEST"
set +e
OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
    --condition D --task walk --seed "$SEED" \
    --steps "$STEPS" --n-envs "$N_ENVS" \
    --eval-every "$EVAL_EVERY" --eval-episodes 6 \
    --eval-tasks rise,hold,walk --eval-heldout \
    --heldout-every "$HELDOUT_EVERY" \
    --checkpoint-every "$CKPT_EVERY" \
    --select-quality \
    --walk-curriculum \
    --walkcurr-cert-every "$CERT_EVERY" \
    --walkcurr-cert-episodes "$CERT_EPISODES" \
    --encoder "$ENC" --encoder-md5 "$ENC_MD5" \
    --anchor-data "$DATA" --device cuda \
    --goal-set walk_cmd_resample_s=4.0 \
    --goal-set walk_cmd_resample_jitter=0.5 \
    --goal-set walk_stop_frac=0.15 \
    $WANDB_FLAG \
    --name "$RUN_NAME"
RC=$?
set -e
if [ "$RC" -ne 0 ]; then
    printf '{"event":"phase_fail","run":"%s","exit_code":%s,"utc":"%s"}\n' \
        "$RUN_NAME" "$RC" "$(date -u +%FT%TZ)" >> "$MANIFEST"
    echo "POD_CRITICDWALKCURR_FAIL rc=$RC $(date -u +%FT%TZ)"
    exit "$RC"
fi
printf '{"event":"done","run":"%s","utc":"%s"}\n' \
    "$RUN_NAME" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_CRITICDWALKCURR_DONE $(date -u +%FT%TZ)"
