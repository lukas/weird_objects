#!/bin/sh
# pod_risewalk.sh — dynrep rise->walk A/B/C benchmark (operator
# next-steps 08-13/14: use the actual robot objective — belly -> rise
# -> hold -> walk — and treat "does C retain rise while PPO learns
# walking" as a FIRST-CLASS hypothesis; the measured failure mode is
# DAgger rise competence erased by PPO walk training within 1M steps).
#
# Design (one variable per run = condition):
#   phase 1  rise, RISE_STEPS (default 500k) from scratch, belly/bridge
#            starts (task rise = p_rise 1.0, the campaign goal mode).
#            eval-tasks rise,hold.
#   phase 2  walk, WALK_STEPS (default 1M), warm-started from the
#            seed's rise checkpoint. eval-tasks rise,hold,walk =
#            rise-retention canary at every eval point, --eval-heldout
#            adds the fixed held-out dynamics suites.
# Success pattern to look for (next-steps "Success criteria"): A loses
# hard-start rise (rise/return + rise/dh_m collapse) while C retains it
# and matches walk; quality columns (slip_m, peak_roll_deg, slew_sat*)
# decide whether the walk is CLEANER, not merely higher-return.
#
# PRECONDITIONS: same as pod_holdwalk.sh — G1/G1.1-passed encoder +
# its dataset present (set ENC/DATA env). Seeds via SEEDS (default
# "1 2 3"; a 26-core pod fits ~3 cohorts).
#
#   ENC=... DATA=... nohup sh rl_move/dynamics/pod_risewalk.sh \
#       > rl_move/dynamics/logs/pod_risewalk.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
DATA=${DATA:-rl_move/dynamics/datasets/v2pod2}
ENC=${ENC:-rl_move/dynamics/models/dyn_v2pod2_obs.pt}
LOG=rl_move/dynamics/logs
SEEDS=${SEEDS:-"1 2 3"}
RISE_STEPS=${RISE_STEPS:-500000}
WALK_STEPS=${WALK_STEPS:-1000000}
EVAL_EVERY=${EVAL_EVERY:-10000}
mkdir -p "$LOG"
echo "== pod_risewalk start $(date -u +%FT%TZ) host=$(hostname)" \
     "seeds=[$SEEDS] rise=$RISE_STEPS walk=$WALK_STEPS enc=$ENC"

[ -f "$ENC" ] || { echo "POD_RISEWALK_ABORT: encoder $ENC missing"; exit 3; }
[ -d "$DATA" ] || { echo "POD_RISEWALK_ABORT: dataset $DATA missing"; exit 3; }
grep -Eq "GATE G1(\.1)? .*PASS" "$LOG/$(basename "$ENC" .pt)_gate.txt" \
    2>/dev/null || {
    echo "POD_RISEWALK_ABORT: no G1/G1.1 PASS on record for $ENC" \
         "(DYNREP.md hard gate)"; exit 3; }

for SEED in $SEEDS; do
    (
        set -e
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task rise --seed "$SEED" \
                --steps "$RISE_STEPS" --eval-every "$EVAL_EVERY" \
                --eval-tasks rise,hold \
                --encoder "$ENC" --anchor-data "$DATA" \
                --name "rw_rise_${C}_s${SEED}"
        done
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task walk --seed "$SEED" \
                --steps "$WALK_STEPS" --eval-every "$EVAL_EVERY" \
                --eval-tasks rise,hold,walk --eval-heldout \
                --encoder "$ENC" --anchor-data "$DATA" \
                --init-from "rl_move/dynamics/models/ppo_rw_rise_${C}_s${SEED}.zip" \
                --name "rw_walk_${C}_s${SEED}"
        done
        echo "RISEWALK_COHORT_DONE seed=$SEED"
    ) > "$LOG/risewalk_s${SEED}.log" 2>&1 &
done
wait

n_done=$(grep -l "RISEWALK_COHORT_DONE" "$LOG"/risewalk_s*.log | wc -l)
echo "cohorts done: $n_done"
echo "POD_RISEWALK_DONE $(date -u +%FT%TZ)"
