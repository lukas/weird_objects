#!/bin/sh
# pod_scale_sweep.sh — dynrep representation SCALING experiment
# (operator next-steps 08-13: "test whether there is a real
# representation scaling curve"; matrix = model size x dataset size x
# history/context length; do NOT scale PPO and assume semantics).
#
# Matrix (12 encoders, all --input-set obs, all 40k steps / matched
# optimizer so the only variables are the matrix axes):
#   size    S ~0.8M (hidden 256, z 128, act 128, 1 GRU layer — current)
#           M ~5.9M (hidden 768, z 256, act 256, 1 layer)
#           L ~17M  (hidden 1024, z 384, act 384, 2 layers)
#   history H16 (0.64 s — current) | H48 (1.92 s)
#   data    small = 1200 eps (v2 recipe size) | large = 4800 eps
#
# Every encoder is gate-evaluated (G1.1 + legacy G1 + per-horizon MSEs
# vs matched baselines); aggregate with analyze_scale.py. NOTE: the
# scaling verdict is the PREDICTION scaling curve — a larger encoder
# only earns PPO wiring through the normal gates + A/B/C protocol.
#
# Stages (STAGE env, default all):
#   collect  12 x 400 eps in parallel per-seed subdirs (seeds 0..11) ->
#            merged datasets/v3scale_small (s0-2) + v3scale_large (all)
#   sweep    the 12 training+gating runs, sequential (GPU if present)
#
#   STAGE=collect nohup sh rl_move/dynamics/pod_scale_sweep.sh \
#       > rl_move/dynamics/logs/scale_collect.log 2>&1 &
#   STAGE=sweep nohup sh rl_move/dynamics/pod_scale_sweep.sh \
#       > rl_move/dynamics/logs/scale_sweep.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-uv run python}
BASE=rl_move/dynamics/datasets
LOG=rl_move/dynamics/logs
STAGE=${STAGE:-all}
STEPS=${STEPS:-40000}
mkdir -p "$LOG"
echo "== pod_scale_sweep stage=$STAGE start $(date -u +%FT%TZ) host=$(hostname)"

# Same hard preflight as pod_v3_pipeline: no silent actor-mix drift.
$PY -c "import sys; sys.path[:0]=['linux_control']; \
import noslip_gait, tripod_gait" || { echo "SCALE_ABORT: gait modules"; exit 4; }
for Z in ppo_goal_cw_walk_longdist_r2 ppo_goal_cw_stance_dr10; do
    [ -f "rl_move/sim/policies/$Z.zip" ] || {
        echo "SCALE_ABORT: champion $Z.zip missing"; exit 4; }
done

if [ "$STAGE" = "collect" ] || [ "$STAGE" = "all" ]; then
    for S in 0 1 2 3 4 5 6 7 8 9 10 11; do
        OMP_NUM_THREADS=1 $PY -m rl_move.dynamics.collect \
            --out "$BASE/v3scale/s$S" --episodes 400 --seed $S \
            > "$LOG/scale_collect_s$S.log" 2>&1 &
    done
    wait
    for S in 0 1 2 3 4 5 6 7 8 9 10 11; do
        grep -q "^done:" "$LOG/scale_collect_s$S.log" || {
            echo "SCALE_ABORT: collect seed $S incomplete"; exit 5; }
    done
    $PY -m rl_move.dynamics.merge_shards \
        --src "$BASE/v3scale/s0" "$BASE/v3scale/s1" "$BASE/v3scale/s2" \
        --out "$BASE/v3scale_small" --require-actor noslip:0.05
    $PY -m rl_move.dynamics.merge_shards \
        --src "$BASE"/v3scale/s0 "$BASE"/v3scale/s1 "$BASE"/v3scale/s2 \
              "$BASE"/v3scale/s3 "$BASE"/v3scale/s4 "$BASE"/v3scale/s5 \
              "$BASE"/v3scale/s6 "$BASE"/v3scale/s7 "$BASE"/v3scale/s8 \
              "$BASE"/v3scale/s9 "$BASE"/v3scale/s10 "$BASE"/v3scale/s11 \
        --out "$BASE/v3scale_large" --require-actor noslip:0.05
    echo "SCALE_COLLECT_DONE $(date -u +%FT%TZ)"
fi

if [ "$STAGE" = "sweep" ] || [ "$STAGE" = "all" ]; then
    for D in small large; do
        [ -f "$BASE/v3scale_$D/meta.json" ] || {
            echo "SCALE_ABORT: dataset v3scale_$D missing"; exit 4; }
    done
    for SIZE in S M L; do
        case $SIZE in
            S) MARGS="--hidden 256 --z-dim 128 --act-hidden 128 --gru-layers 1";;
            M) MARGS="--hidden 768 --z-dim 256 --act-hidden 256 --gru-layers 1";;
            L) MARGS="--hidden 1024 --z-dim 384 --act-hidden 384 --gru-layers 2";;
        esac
        for H in 16 48; do
            for D in small large; do
                NAME=dyn_scale_${SIZE}_h${H}_${D}
                if [ -f "$LOG/${NAME}_gate.txt" ]; then
                    echo "skip $NAME (gate record exists)"; continue
                fi
                echo "== $NAME $(date -u +%FT%TZ)"
                OMP_NUM_THREADS=20 $PY -m rl_move.dynamics.train \
                    --data "$BASE/v3scale_$D" --name "$NAME" \
                    --input-set obs --steps "$STEPS" \
                    --lr-final-frac 0.05 --history "$H" $MARGS \
                    > "$LOG/${NAME}_train.log" 2>&1
                $PY -m rl_move.dynamics.eval_model \
                    --ckpt "rl_move/dynamics/models/${NAME}.pt" \
                    --data "$BASE/v3scale_$D" \
                    2>&1 | tee "$LOG/${NAME}_gate.txt"
            done
        done
    done
    $PY -m rl_move.dynamics.analyze_scale 2>&1 \
        | tee "$LOG/scale_summary.txt"
    echo "POD_SCALE_SWEEP_DONE $(date -u +%FT%TZ)"
fi
