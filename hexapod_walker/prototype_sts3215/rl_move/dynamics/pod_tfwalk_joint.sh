#!/bin/sh
# pod_tfwalk_joint.sh — corrected joint PPO+auxiliary A/B/C walking
# cohort (operator directive fb_20260816T203212_af7c64, filed via the
# MCP operator lane; explicit Lukas request).
#
# The metrics1 cohort showed the old condition-C AnchorCb (out-of-band
# Adam steps on the shared transformer between rollout collection and
# the PPO update) is UNSTABLE, not that the transformer is too large:
# C peaked ~400 near 940k, regressed to 262 by 2M with approx_kl
# ~0.085-0.089 vs A/B ~0.02 and DR10 early-term ~75%. This cohort
# tests the corrected mechanism:
#   A  scratch     MlpPolicy on raw stacked obs (control)
#   B  frozen      pretrained TF encoder (frozen for the whole run =
#                  head warmup that never ends), PPO heads learn
#   C  joint-aux   same encoder, 50k-step encoder-frozen head warmup,
#                  then actor/value/transformer train JOINTLY: the
#                  future-state loss joins every PPO minibatch (same
#                  backward/optimizer, encoder at 0.1x LR), auxiliary
#                  batches are ONLINE rollout windows + 25% rehearsal
#                  from the recovered v5_mjx_fresh corpus (NO
#                  recollection), total action-KL guarded (0.04
#                  rollback guard, 0.02 target) with rollback/stop.
# Budget: 1M steps = the pre-registered decision checkpoint (extend
# only if corrected C beats B). Design otherwise matched to
# tfwalk-metrics1 (task walk, seed panel, dr 0.3, eval every 10k on
# rise,hold,walk + heldout suites, term_penalty 30, n_envs 8) so the
# metrics1 seed-5 A/B curves are reusable at the 1M point.
# Checkpoints every 250k; best-by-heldout-walk retained as *_best.zip.
#
# Usage (per pod):
#   COND=C SEED=5 nohup sh rl_move/dynamics/pod_tfwalk_joint.sh \
#       > rl_move/dynamics/logs/pod_tfwalk_joint1_C_s5.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
COND=${COND:?set COND=A|B|C}
SEED=${SEED:-5}
ENC=${ENC:-rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt}
DATA=${DATA:-rl_move/dynamics/datasets/v5_mjx_fresh}
WALK_STEPS=${WALK_STEPS:-1000000}
EVAL_EVERY=${EVAL_EVERY:-10000}
COHORT_NAME=${COHORT_NAME:-tfwalk-joint1}
LOG=rl_move/dynamics/logs
MANIFEST="$LOG/${COHORT_NAME}_manifest.jsonl"
NAME="dynrep-${COHORT_NAME}-${COND}-s${SEED}"
mkdir -p "$LOG"
echo "== pod_tfwalk_joint start $(date -u +%FT%TZ) host=$(hostname)" \
     "cond=$COND seed=$SEED steps=$WALK_STEPS enc=$ENC"

# Preconditions: the cohort's premise is the G1/G1.1 gate on the TF
# encoder (DYNREP.md hard gate: no PPO before G1 passes).
grep -Eq "GATE G1(\.1)? .*PASS" "$LOG/$(basename "$ENC" .pt)_gate.txt" \
    2>/dev/null || {
    echo "POD_TFWALK_JOINT_ABORT: no G1/G1.1 PASS on record for $ENC"; exit 3; }
if [ "$COND" != "A" ]; then
    [ -f "$ENC" ] || { echo "POD_TFWALK_JOINT_ABORT: encoder $ENC missing"; exit 3; }
fi
if [ "$COND" = "C" ]; then
    [ -d "$DATA" ] || { echo "POD_TFWALK_JOINT_ABORT: rehearsal corpus $DATA missing"; exit 3; }
fi
# Match only a REAL trainer invocation (python -m rl_move.dynamics.
# train_ppo_transfer). Matching bare "train_ppo_transfer" false-aborted
# on train-4 (08-16): a leftover `bash -c` launcher wrapper from the
# risewalk cohort still carried that substring inside its own inline
# case-pattern text.
for f in /proc/[0-9]*/cmdline; do
    c=$(tr '\0' ' ' < "$f" 2>/dev/null) || continue
    case "$c" in *rl_move.dynamics.train_ppo_transfer*)
        echo "POD_TFWALK_JOINT_ABORT: train_ppo_transfer already running on $(hostname)"
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
echo "POD_TFWALK_JOINT_DONE $(date -u +%FT%TZ)"
printf '{"event":"done","cohort":"%s","utc":"%s","seeds_done":1,"condition":"%s"}\n' \
    "$COHORT_NAME" "$(date -u +%FT%TZ)" "$COND" >> "$MANIFEST"
