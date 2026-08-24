#!/bin/sh
# pod_tfwalk_critic.sh — decoupled predictive-critic D/E walking cohort
# (operator directive fb_20260817T052333_e5ae09, filed via the MCP
# operator lane; explicit Lukas "ok try it" after the tfwalk-joint1
# verdict that actor-side transfer fails under every tried mechanism).
#
#   D  frozen-critic  scratch-A actor; critic adds a zero-gated
#                     stop-gradient latent residual from the FROZEN
#                     pretrained transformer snapshot.
#   E  online-critic  same, plus a separate ONLINE transformer training
#                     continuously on fresh rollout windows + 25%
#                     rehearsal (own optimizer); a guarded EMA snapshot
#                     updated only BETWEEN rollout+PPO iterations feeds
#                     the critic residual.
#
# The actor NEVER consumes transformer latents (asserted in-process:
# zero action-KL from predictor updates, bit-checked actor params,
# snapshot version frozen across rollout+GAE+PPO).
# Budget: 1M steps pre-registered decision gate (canary: WALK_STEPS=
# 100000, COHORT_NAME=tfwalk-critic-canary). Design matched to
# tfwalk-joint1 (task walk, dr 0.3, eval every 10k on rise,hold,walk +
# heldout suites, term_penalty 30, n_envs 8) so the joint1 A seeds
# 6/7 + metrics1 A-s5 (at its 1M eval point) are the scratch controls.
#
# Usage (per pod):
#   COND=E SEED=5 nohup sh rl_move/dynamics/pod_tfwalk_critic.sh \
#       > rl_move/dynamics/logs/pod_tfwalk_critic1_E_s5.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
COND=${COND:?set COND=D|E}
SEED=${SEED:-5}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
WALK_STEPS=${WALK_STEPS:-1000000}
EVAL_EVERY=${EVAL_EVERY:-10000}
COHORT_NAME=${COHORT_NAME:-tfwalk-critic1}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/${COHORT_NAME}_manifest.jsonl"
NAME="dynrep-${COHORT_NAME}-${COND}-s${SEED}"
mkdir -p "$LOG"
echo "== pod_tfwalk_critic start $(date -u +%FT%TZ) host=$(hostname)" \
     "cond=$COND seed=$SEED steps=$WALK_STEPS enc=$ENC"

case "$COND" in D|E) ;; *)
    echo "POD_TFWALK_CRITIC_ABORT: COND must be D or E"; exit 3;; esac
# Preconditions: G1/G1.1 gate on the TF encoder (DYNREP.md hard gate),
# encoder + rehearsal corpus present (both conditions read the corpus:
# E for rehearsal, D+E for the drift probe / heldout reference).
grep -Eq "GATE G1(\.1)? .*PASS" "$LOG/$(basename "$ENC" .pt)_gate.txt" \
    2>/dev/null || {
    echo "POD_TFWALK_CRITIC_ABORT: no G1/G1.1 PASS on record for $ENC"; exit 3; }
[ -f "$ENC" ] || { echo "POD_TFWALK_CRITIC_ABORT: encoder $ENC missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_TFWALK_CRITIC_ABORT: corpus $DATA missing"; exit 3; }
# Refuse to stack trainers on one pod (match the REAL module invocation
# only — bare-substring matching false-aborted on train-4, 08-16).
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.dynamics.train_ppo_transfer*)
        echo "POD_TFWALK_CRITIC_ABORT: train_ppo_transfer already running on $(hostname)"
        exit 4;;
    esac
done

printf '{"event":"start","cohort":"%s","host":"%s","utc":"%s","condition":"%s","seed":%s,"encoder":"%s","data":"%s","walk_steps":%s}\n' \
    "$COHORT_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$COND" "$SEED" "$ENC" "$DATA" "$WALK_STEPS" \
    >> "$MANIFEST"
printf '{"event":"phase_start","cohort":"%s","seed":%s,"condition":"%s","task":"walk","utc":"%s"}\n' \
    "$COHORT_NAME" "$SEED" "$COND" "$(date -u +%FT%TZ)" >> "$MANIFEST"
# Never let `set -e` swallow a trainer death (tfwalk-joint1 lesson:
# memwatch SIGKILLs left the ledger stale-RUNNING for hours).
set +e
OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
    --condition "$COND" --task walk --seed "$SEED" \
    --steps "$WALK_STEPS" --eval-every "$EVAL_EVERY" \
    --eval-tasks rise,hold,walk --eval-heldout \
    --encoder "$ENC" --anchor-data "$DATA" \
    --device cuda \
    --name "$NAME"
RC=$?
set -e
if [ "$RC" -ne 0 ]; then
    printf '{"event":"phase_fail","cohort":"%s","seed":%s,"condition":"%s","task":"walk","exit_code":%s,"utc":"%s"}\n' \
        "$COHORT_NAME" "$SEED" "$COND" "$RC" "$(date -u +%FT%TZ)" >> "$MANIFEST"
    echo "POD_TFWALK_CRITIC_FAIL rc=$RC $(date -u +%FT%TZ)"
    exit "$RC"
fi
printf '{"event":"phase_done","cohort":"%s","seed":%s,"condition":"%s","task":"walk","utc":"%s"}\n' \
    "$COHORT_NAME" "$SEED" "$COND" "$(date -u +%FT%TZ)" >> "$MANIFEST"
echo "POD_TFWALK_CRITIC_DONE $(date -u +%FT%TZ)"
printf '{"event":"done","cohort":"%s","utc":"%s","seeds_done":1,"condition":"%s"}\n' \
    "$COHORT_NAME" "$(date -u +%FT%TZ)" "$COND" >> "$MANIFEST"
