#!/bin/sh
# pod_pilot_rep.sh — dynrep pilot SEED REPLICATION on a train pod's idle
# CPUs (track STATUS "Next", 08-13). The operator's v2 dataset + encoder
# are laptop-local (gitignored), so this regenerates both from the
# documented v2 recipe (README quick start), gates them (G1/G2 — PPO is
# NOT wired on a gate FAIL), then runs the A/B/C pilot cohort for seeds
# 1..3 (seed 0 = the operator's laptop pilot) in parallel, one subshell
# per seed. Artifacts are named *v2pod* so they can never be confused
# with the operator's laptop v2 artifacts; run names keep the standard
# pilot_* pattern so analyze_pilot.py aggregates unchanged.
#
#   nohup sh rl_move/dynamics/pod_pilot_rep.sh > .../pod_pilot_rep.log &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
DATA=rl_move/dynamics/datasets/v2pod
ENC_NAME=dyn_v2pod_obs
ENC=rl_move/dynamics/models/${ENC_NAME}.pt
LOG=rl_move/dynamics/logs
mkdir -p "$LOG"
echo "== pod_pilot_rep start $(date -u +%FT%TZ) host=$(hostname)"

# 1) dataset, v2 recipe: 3 collect seeds x 400 eps (append-safe shards).
#    NOTE: noslip actor falls back to tripod on pods (module is
#    laptop-local; collect.py warns + renormalizes).
for S in 0 1 2; do
    $PY -m rl_move.dynamics.collect --out "$DATA" --episodes 400 --seed $S
done

# 2) the PPO-facing obs-input encoder, v2 recipe
OMP_NUM_THREADS=20 $PY -m rl_move.dynamics.train \
    --data "$DATA" --name "$ENC_NAME" --input-set obs \
    --steps 40000 --lr-final-frac 0.05

# 3) gates G1+G2 (obs input set): PPO only on PASS
$PY -m rl_move.dynamics.eval_model --ckpt "$ENC" --data "$DATA" \
    --dump-latents 2>&1 | tee "$LOG/${ENC_NAME}_gate.txt"
grep -q "GATE G1 .*PASS" "$LOG/${ENC_NAME}_gate.txt" || {
    echo "POD_PILOT_REP_G1_FAIL: encoder did not beat baselines;" \
         "PPO not wired (DYNREP.md hard gate)"; exit 3; }

# 4) A/B/C pilot cohort, seeds 1..3 in parallel (one cohort per subshell;
#    phases within a seed stay sequential: lower warm-starts from hold)
for SEED in 1 2 3; do
    (
        set -e
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task hold --seed "$SEED" \
                --steps 150000 --encoder "$ENC" --anchor-data "$DATA" \
                --name "pilot_hold_${C}_s${SEED}"
        done
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task lower --seed "$SEED" \
                --steps 150000 --encoder "$ENC" --anchor-data "$DATA" \
                --init-from "rl_move/dynamics/models/ppo_pilot_hold_${C}_s${SEED}.zip" \
                --name "pilot_lower_${C}_s${SEED}"
        done
        echo "PILOT_COHORT_DONE seed=$SEED"
    ) > "$LOG/pilot_rep_s${SEED}.log" 2>&1 &
done
wait

# 5) cross-seed aggregation (pod seeds only — do NOT pool the operator's
#    s0: it used the laptop encoder/dataset)
n_done=$(grep -l "PILOT_COHORT_DONE" "$LOG"/pilot_rep_s?.log | wc -l)
echo "cohorts done: $n_done/3"
$PY -m rl_move.dynamics.analyze_pilot --seeds 1 2 3 2>&1 \
    | tee "$LOG/pilot_rep_summary.txt"
echo "POD_PILOT_REP_DONE $(date -u +%FT%TZ)"
