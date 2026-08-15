#!/bin/sh
# pod_holdwalk.sh — dynrep hold->walk transfer pair at pod budgets
# (operator directive, 08-13: lower is too close to hold to
# discriminate — all three conditions transferred at the same speed
# locally; walk is the brief's real ladder and needs pod steps).
#
# PRECONDITIONS (the launching cycle checks these — this script only
# asserts them):
#   * the pod pilot replication (pod_pilot_rep.sh) has FINISHED and
#     been TRIAGED (operator ordering: rep triage first);
#   * rl_move/dynamics/models/dyn_v2pod_obs.pt exists and its G1 gate
#     PASSED (logs/dyn_v2pod_obs_gate.txt);
#   * datasets/v2pod exists (anchor data for condition C).
#
# Design (one variable per run = condition; everything else matched):
#   phase 1  hold, 150k steps — REUSED from the rep for seeds 1..3
#            (ppo_pilot_hold_{A,B,C}_s{1,2,3}.zip, same recipe), and
#            trained fresh for any seed missing a checkpoint (seeds
#            4..5 — operator: NEW cohorts need seeds >= 5). Eval
#            cadence does not perturb training (fixed-seed separate
#            envs, deterministic predict), so 25k-grid rep checkpoints
#            and 10k-grid fresh ones are the same population.
#   phase 2  walk, WALK_STEPS (default 1M) — warm-start from the
#            seed's hold checkpoint, eval-tasks hold,walk so retention
#            curves come for free. eval-every 10k (operator: <= 10k).
#
# Parallelism: one subshell per seed (SEEDS env; default "1 2 3 4 5").
# A 26-core train pod comfortably fits 2-3 seed cohorts; spread seeds
# across pods by launching with different SEEDS (dataset + encoder
# must be synced to each pod first).
#
#   nohup sh rl_move/dynamics/pod_holdwalk.sh > .../pod_holdwalk.log &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
DATA=${DATA:-rl_move/dynamics/datasets/v2pod}
ENC=${ENC:-rl_move/dynamics/models/dyn_v2pod_obs.pt}
LOG=rl_move/dynamics/logs
SEEDS=${SEEDS:-"1 2 3 4 5"}
HOLD_STEPS=${HOLD_STEPS:-150000}
WALK_STEPS=${WALK_STEPS:-1000000}
EVAL_EVERY=${EVAL_EVERY:-10000}
COHORT_NAME=${COHORT_NAME:-holdwalk}
MANIFEST="$LOG/${COHORT_NAME}_manifest.jsonl"
mkdir -p "$LOG"
echo "== pod_holdwalk start $(date -u +%FT%TZ) host=$(hostname)" \
     "seeds=[$SEEDS] hold=$HOLD_STEPS walk=$WALK_STEPS"

[ -f "$ENC" ] || { echo "POD_HOLDWALK_ABORT: encoder $ENC missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_HOLDWALK_ABORT: dataset $DATA missing"; exit 3; }
# Accept the original G1 or the prospectively recorded G1.1 (revised
# 2026-08-13, rl_docs/DYNREP.md — G1.1 governs encoders gated from that
# date; a legacy PASS implies a G1.1 PASS).
grep -Eq "GATE G1(\.1)? .*PASS" "$LOG/$(basename "$ENC" .pt)_gate.txt" \
    2>/dev/null || {
    echo "POD_HOLDWALK_ABORT: no G1/G1.1 PASS on record for $ENC" \
         "(DYNREP.md hard gate)"; exit 3; }

if pgrep -f "rl_move.dynamics.train_ppo_transfer" >/dev/null 2>&1; then
    echo "POD_HOLDWALK_ABORT: train_ppo_transfer already running on host=$(hostname)"
    exit 4
fi

printf '{"event":"start","cohort":"%s","host":"%s","utc":"%s","seeds":"%s","encoder":"%s","data":"%s","hold_steps":%s,"walk_steps":%s}\n' \
    "$COHORT_NAME" "$(hostname)" "$(date -u +%FT%TZ)" "$SEEDS" "$ENC" "$DATA" "$HOLD_STEPS" "$WALK_STEPS" \
    >> "$MANIFEST"

for SEED in $SEEDS; do
    (
        set -e
        printf '{"event":"seed_start","cohort":"%s","seed":%s,"utc":"%s"}\n' \
            "$COHORT_NAME" "$SEED" "$(date -u +%FT%TZ)" >> "$MANIFEST"
        for C in A B C; do
            HOLD_CKPT="rl_move/dynamics/models/ppo_pilot_hold_${C}_s${SEED}.zip"
            if [ ! -f "$HOLD_CKPT" ]; then
                echo "hold checkpoint missing for ${C}_s${SEED}; training"
                printf '{"event":"phase_start","cohort":"%s","seed":%s,"condition":"%s","task":"hold","utc":"%s"}\n' \
                    "$COHORT_NAME" "$SEED" "$C" "$(date -u +%FT%TZ)" >> "$MANIFEST"
                OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                    --condition "$C" --task hold --seed "$SEED" \
                    --steps "$HOLD_STEPS" --eval-every "$EVAL_EVERY" \
                    --encoder "$ENC" --anchor-data "$DATA" \
                    --name "pilot_hold_${C}_s${SEED}"
                printf '{"event":"phase_done","cohort":"%s","seed":%s,"condition":"%s","task":"hold","utc":"%s"}\n' \
                    "$COHORT_NAME" "$SEED" "$C" "$(date -u +%FT%TZ)" >> "$MANIFEST"
            fi
            # eval-tasks includes rise: the hard rise-retention canary
            # (operator next-steps 08-14 — the measured failure mode is
            # rise competence erased by PPO); eval-heldout adds the
            # fixed held-out dynamics suites on the trained task.
            printf '{"event":"phase_start","cohort":"%s","seed":%s,"condition":"%s","task":"walk","utc":"%s"}\n' \
                "$COHORT_NAME" "$SEED" "$C" "$(date -u +%FT%TZ)" >> "$MANIFEST"
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task walk --seed "$SEED" \
                --steps "$WALK_STEPS" --eval-every "$EVAL_EVERY" \
                --eval-tasks hold,walk,rise --eval-heldout \
                --encoder "$ENC" --anchor-data "$DATA" \
                --init-from "$HOLD_CKPT" \
                --name "pilot_walk_${C}_s${SEED}"
            printf '{"event":"phase_done","cohort":"%s","seed":%s,"condition":"%s","task":"walk","utc":"%s"}\n' \
                "$COHORT_NAME" "$SEED" "$C" "$(date -u +%FT%TZ)" >> "$MANIFEST"
        done
        echo "HOLDWALK_COHORT_DONE seed=$SEED"
        printf '{"event":"seed_done","cohort":"%s","seed":%s,"utc":"%s"}\n' \
            "$COHORT_NAME" "$SEED" "$(date -u +%FT%TZ)" >> "$MANIFEST"
    ) > "$LOG/holdwalk_s${SEED}.log" 2>&1 &
done
wait

n_done=$(grep -l "HOLDWALK_COHORT_DONE" "$LOG"/holdwalk_s*.log | wc -l)
echo "cohorts done: $n_done"
# steps-to-threshold for walk needs a threshold pinned from the first
# curves; run analyze by hand after eyeballing:
#   $PY -m rl_move.dynamics.analyze_pilot --seeds $SEEDS \
#       --phase2 walk --phase2-threshold <thr> --plot
echo "POD_HOLDWALK_DONE $(date -u +%FT%TZ)"
printf '{"event":"done","cohort":"%s","utc":"%s","seeds_done":%s}\n' \
    "$COHORT_NAME" "$(date -u +%FT%TZ)" "$n_done" >> "$MANIFEST"
