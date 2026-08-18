#!/bin/sh
# pod_criticDwalkcurr2.sh — cw-dynrep-criticD-walkcurr2 (operator MCP
# note fb_20260818T060044, "figure out how to make a great run and
# then launch it"): corrected re-run of cw-dynrep-criticD-walkcurr1
# after matched data (cw-dynrep-criticD-40m1 vs the in-flight
# walkcurr1) showed walkcurr1's B0/B1 ignition band and hard slew
# admission bar made a genuinely-good known checkpoint (the frozen-D
# 6M best) fail its own gate and made near-zero output ("parking")
# score close to peak reward for the whole B0/B1 span (0/10
# promotions at ~5.2M). Same fresh scratch actor (seed 8), EXACT
# vt2ovznc frozen transformer critic D (md5-pinned, no online
# predictor), 40M steps, n_envs 16, CUDA — this run changes THREE
# coupled things vs walkcurr1 (repealed one-variable-per-run,
# operator 08-15: coupled bundles are fine when the cycle judges the
# coupling necessary — these three ARE coupled: a curriculum redesign
# is not fairly testable on top of an unstable update path, so both
# had to move together):
#   1. --walk-curriculum-version 2 (WALKCURR_BUCKETS_V2 in
#      walk_task.py): B0 ignition speed 0.08-0.12 m/s + heading spread
#      from the first bucket (was 0.04-0.05 m/s dead-ahead, sitting
#      entirely inside SIGMA_V=0.05's velocity-tracking kernel width —
#      parking scored ~67% of peak); cert gate slew_sat_max relaxed
#      0.5->0.95 (the known-good 6M criticD-40m1 checkpoint runs
#      slew_sat~0.925 and would fail V1's hard bar outright); a later
#      per-bucket "quality" progress bar (0.75) still applies from B1
#      on, tightened relative to the B0 ignition bar (0.65).
#   2. Update-path health (rl_move/sim/update_health.py, already
#      proven in train_ppo_mjx.py): n_epochs 10->3, target_kl 0.01,
#      separate actor(1e-4->1e-5 linear decay)/critic(3e-4 constant)
#      optimizer groups, transactional KL-rollback at realized
#      approx_kl>0.03. criticD-40m1's own training log showed exactly
#      the update-health failure mode this fixes: SCORE/loco_quality
#      peaked at 6M (0.51) and never again in 34M more steps,
#      collapsing to 0.02-0.10 by 35-40M as approx_kl/clip_frac
#      climbed unchecked (no target_kl, no rollback, single LR group,
#      10 epochs/update).
#   3. (unchanged from walkcurr1) all-env admission broadcast,
#      frontier+retention promotion, 2-consecutive-failure rollback,
#      best checkpoint = last retention-clean promotion.
# Pre-launch: default-off code (goal.walk_curriculum=2 selects
# WALKCURR_BUCKETS_V2; --actor-lr/--kl-rollback/etc. all 0/off unless
# passed), test_walk_curriculum.py (19, incl. 6 new v2-specific:
# ignition-band-vs-SIGMA_V, gate-admits-known-good-6M-numbers,
# gate-tiering, table-selection-bit-exact), test_dynrep_predictive_
# critic.py (11, incl. 3 new: extended CRITIC_MARKERS covers
# value_gate/latent_adapter), test_value_learning.py (12) all green;
# full test_task_semantics.py bank 126 passed/4 skipped/1 xfailed.
# CUDA canary (canary-walkcurr2, --no-wandb, 300k steps, this exact
# CLI) verified before the full launch: update-health attaches (7
# actor/11 critic tensors matching the extended-marker split), at
# least one cert round completes without crash, GPU active.
#
#   nohup sh rl_move/dynamics/pod_criticDwalkcurr2.sh \
#     > rl_move/dynamics/logs/pod_criticDwalkcurr2.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
RUN_NAME=${RUN_NAME:-cw-dynrep-criticD-walkcurr2}
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
MANIFEST="$LOG/criticDwalkcurr2_manifest.jsonl"
mkdir -p "$LOG"
echo "== pod_criticDwalkcurr2 start $(date -u +%FT%TZ) host=$(hostname)" \
     "run=$RUN_NAME seed=$SEED steps=$STEPS n_envs=$N_ENVS" \
     "cert_every=$CERT_EVERY"

nvidia-smi --query-gpu=name --format=csv,noheader | grep -q H200 || {
    echo "POD_CRITICDWALKCURR2_ABORT: no H200 GPU"; exit 3; }
grep -Eq "GATE G1(\.1)? .*PASS" \
    "$LOG/$(basename "$ENC" .pt)_gate.txt" 2>/dev/null || {
    echo "POD_CRITICDWALKCURR2_ABORT: no G1/G1.1 PASS for $ENC"; exit 3; }
[ -f "$ENC" ] || { echo "POD_CRITICDWALKCURR2_ABORT: encoder missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_CRITICDWALKCURR2_ABORT: corpus missing"; exit 3; }
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.dynamics.train_ppo_transfer*)
        echo "POD_CRITICDWALKCURR2_ABORT: trainer already running"; exit 4;;
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
    --walk-curriculum --walk-curriculum-version 2 \
    --walkcurr-cert-every "$CERT_EVERY" \
    --walkcurr-cert-episodes "$CERT_EPISODES" \
    --n-epochs 3 --target-kl 0.01 \
    --actor-lr 1e-4 --actor-lr-final 1e-5 --critic-lr 3e-4 \
    --kl-rollback 0.03 --kl-rollback-lr-factor 0.5 \
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
    echo "POD_CRITICDWALKCURR2_FAIL rc=$RC $(date -u +%FT%TZ)"
    exit "$RC"
fi
printf '{"event":"done","run":"%s","utc":"%s"}\n' \
    "$RUN_NAME" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_CRITICDWALKCURR2_DONE $(date -u +%FT%TZ)"
