#!/bin/sh
# run_pilot.sh — the local A/B/C pilot cohort (rl_docs/DYNREP.md).
#
# Phase 1: learn "hold" (quiet plant stance) from scratch, per condition.
# Phase 2: warm-start each condition's phase-1 checkpoint on "lower"
#          (controlled descent to belly) — the transfer measurement.
# Matched seed/budget/env; the only variable is the representation.
# Eval CSVs land in logs/ppo_pilot_*_eval.csv (both tasks at every
# eval point, so phase 2 rows double as hold-retention curves).
#
#   sh rl_move/dynamics/run_pilot.sh [steps] [seed]
#
# Run names are seed-suffixed (pilot_hold_A_s0 ...) so multi-seed
# sweeps don't clobber each other's checkpoints or eval CSVs.
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
STEPS=${1:-150000}
SEED=${2:-0}

for C in A B C; do
    $PY -m rl_move.dynamics.train_ppo_transfer \
        --condition "$C" --task hold --seed "$SEED" --steps "$STEPS" \
        --name "pilot_hold_${C}_s${SEED}"
done
for C in A B C; do
    $PY -m rl_move.dynamics.train_ppo_transfer \
        --condition "$C" --task lower --seed "$SEED" --steps "$STEPS" \
        --init-from "rl_move/dynamics/models/ppo_pilot_hold_${C}_s${SEED}.zip" \
        --name "pilot_lower_${C}_s${SEED}"
done
echo "PILOT_COHORT_DONE seed=$SEED"
