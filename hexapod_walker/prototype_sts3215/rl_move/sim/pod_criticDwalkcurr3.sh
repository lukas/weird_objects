#!/bin/sh
# pod_criticDwalkcurr3.sh — cw-dynrep-criticD-walkcurr3 (operator order
# 2026-08-18, MCP lane fb_20260818T065930_03b422, GPT-5 Codex for
# Lukas: "URGENT: walkcurr2 still CPU physics; launch true Warp/MJX
# walkcurr3"): the ALL-GPU replacement for cw-dynrep-criticD-walkcurr2.
# walkcurr2 put only Torch on the H200 — its SimHexapodJointWalkEnv
# physics ran C-MuJoCo on CPUs through SubprocVecEnv (n_envs 16).
# This run trains the SAME contract on batched Warp GPU physics via
# train_ppo_mjx + MjxShardedVecEnv:
#   - exact condition-D PredictiveCriticPolicy/PPO, frozen vt2ovznc
#     transformer critic (md5-pinned fail-closed), fresh scratch actor
#     seed 8, 40M steps;
#   - walkcurr V2 ignition curriculum (goal.walk_curriculum=2 +
#     goal.walk_pure=1 fixed at env CONSTRUCTION), deterministic
#     certification every 500k ON THE TRAINING BACKEND (dedicated MJX
#     cert env + in-env walk probe; C evaluation is never an admission
#     signal), all-env admission broadcast, frontier+retention
#     promotion, 2-consecutive-retained-failure rollback, best = last
#     retention-clean promotion, reset-pool flush on admission changes;
#   - update-path health: n_epochs 3, target_kl 0.01, actor 1e-4->1e-5
#     linear / critic 3e-4 constant optimizer groups (CRITIC_MARKERS
#     extended for value_gate/latent_adapter), transactional
#     KL-rollback at realized approx_kl>0.03;
#   - reward contract parity: reward.term_penalty=30 (in-env twin of
#     the transfer trainer's --term-penalty 30), same goal.walk_cmd_*
#     keys for the plain-distribution C-env audit evals;
#   - PROVEN GPU geometry: n_envs 4096, host-workers 24, n_steps 16,
#     batch 8192, impl warp (guardrails.yaml gpu block), NOT n_envs 16;
#   - --require-gpu-physics: fail-closed _assert_gpu_physics refuses
#     SubprocVecEnv / non-warp impl (unit-tested), and the log carries
#     the "[backend] training physics VERIFIED" proof line.
# Code: tag exp/cw-dynrep-criticD-walkcurr3-runner. Pre-launch on-pod:
# parity suites (test_mjx_parity, test_mjx_vec_env, test_walkcurr_mjx)
# + CUDA canary (canary-walkcurr3: cert rounds on Warp, update-health
# split, encoder md5, snapshot_version 0) — see the run ledger entry.
#
#   nohup sh rl_move/sim/pod_criticDwalkcurr3.sh \
#     > rl_move/dynamics/logs/pod_criticDwalkcurr3.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
RUN_NAME=${RUN_NAME:-cw-dynrep-criticD-walkcurr3}
SEED=${SEED:-8}
STEPS=${STEPS:-40000000}
N_ENVS=${N_ENVS:-4096}
HOST_WORKERS=${HOST_WORKERS:-24}
EVAL_EVERY=${EVAL_EVERY:-1000000}
VIDEO_EVERY=${VIDEO_EVERY:-2000000}
SAVE_EVERY=${SAVE_EVERY:-2000000}
CERT_EVERY=${CERT_EVERY:-500000}
CERT_EPISODES=${CERT_EPISODES:-8}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
ENC_MD5=${ENC_MD5:-9df48f687967c25085ee50171e4110ff}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
EXTRA_FLAGS=${EXTRA_FLAGS:-}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/criticDwalkcurr3_manifest.jsonl"
mkdir -p "$LOG"
echo "== pod_criticDwalkcurr3 start $(date -u +%FT%TZ) host=$(hostname)" \
     "run=$RUN_NAME seed=$SEED steps=$STEPS n_envs=$N_ENVS" \
     "host_workers=$HOST_WORKERS cert_every=$CERT_EVERY"

nvidia-smi --query-gpu=name --format=csv,noheader | grep -q H200 || {
    echo "POD_CRITICDWALKCURR3_ABORT: no H200 GPU"; exit 3; }
grep -Eq "GATE G1(\.1)? .*PASS" \
    "$LOG/$(basename "$ENC" .pt)_gate.txt" 2>/dev/null || {
    echo "POD_CRITICDWALKCURR3_ABORT: no G1/G1.1 PASS for $ENC"; exit 3; }
[ -f "$ENC" ] || { echo "POD_CRITICDWALKCURR3_ABORT: encoder missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_CRITICDWALKCURR3_ABORT: corpus missing"; exit 3; }
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.sim.train_ppo_mjx*|*rl_move.dynamics.train_ppo_transfer*)
        echo "POD_CRITICDWALKCURR3_ABORT: trainer already running"; exit 4;;
    esac
done

printf '{"event":"start","run":"%s","host":"%s","utc":"%s","seed":%s,"steps":%s,"n_envs":%s,"host_workers":%s,"encoder_md5":"%s","cert_every":%s,"backend":"MjxShardedVecEnv/warp"}\n' \
    "$RUN_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$SEED" "$STEPS" \
    "$N_ENVS" "$HOST_WORKERS" "$ENC_MD5" "$CERT_EVERY" >> "$MANIFEST"
set +e
OMP_NUM_THREADS=4 $PY -m rl_move.sim.train_ppo_mjx \
    --task joint_walk --seed "$SEED" \
    --steps "$STEPS" --n-envs "$N_ENVS" \
    --impl warp --host-workers "$HOST_WORKERS" \
    --require-gpu-physics \
    --critic-encoder "$ENC" --critic-encoder-md5 "$ENC_MD5" \
    --anchor-data "$DATA" \
    --walk-curriculum --walk-curriculum-version 2 \
    --walkcurr-cert-every "$CERT_EVERY" \
    --walkcurr-cert-episodes "$CERT_EPISODES" \
    --n-epochs 3 --target-kl 0.01 \
    --actor-lr 1e-4 --actor-lr-final 1e-5 --critic-lr 3e-4 \
    --kl-rollback 0.03 --kl-rollback-lr-factor 0.5 \
    --dr-scale 0.3 \
    --eval-every "$EVAL_EVERY" --video-every "$VIDEO_EVERY" \
    --save-every "$SAVE_EVERY" \
    --cfg-set obs.history_frames=16 \
    --cfg-set goal.walk_cmd_resample_s=4.0 \
    --cfg-set goal.walk_cmd_resample_jitter=0.5 \
    --cfg-set goal.walk_stop_frac=0.15 \
    --cfg-set reward.term_penalty=30 \
    $EXTRA_FLAGS \
    --run-name "$RUN_NAME" --out-name "$RUN_NAME"
RC=$?
set -e
if [ "$RC" -ne 0 ]; then
    printf '{"event":"phase_fail","run":"%s","exit_code":%s,"utc":"%s"}\n' \
        "$RUN_NAME" "$RC" "$(date -u +%FT%TZ)" >> "$MANIFEST"
    echo "POD_CRITICDWALKCURR3_FAIL rc=$RC $(date -u +%FT%TZ)"
    exit "$RC"
fi
printf '{"event":"done","run":"%s","utc":"%s"}\n' \
    "$RUN_NAME" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_CRITICDWALKCURR3_DONE $(date -u +%FT%TZ)"
