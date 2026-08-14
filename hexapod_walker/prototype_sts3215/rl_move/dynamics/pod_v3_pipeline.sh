#!/bin/sh
# pod_v3_pipeline.sh — dynrep v3: DATASET-DRIFT FIX + revised gate G1.1.
#
# Why v3: the v2pod encoder G1-FAILED twice (seeds 0/1) at k=1 ONLY,
# losing to the matched ridge by ~2.7% while beating every baseline at
# k=2/5/10/25. The pre-registered fork closed training variance as the
# explanation and left the KNOWN dataset drift as prime suspect:
# noslip_gait.py never reached the pods, so its 10% actor share fell
# back to tripod — more-periodic data that strengthens exactly the
# ridge baseline doing the vetoing. noslip_gait.py is committed since
# decb1fa and rides the code sync, so this pipeline REQUIRES it (and
# both champion checkpoints): silent degradation was the bug, so any
# missing ingredient is a hard abort, never a renormalized mix.
#
# Gate: G1.1 (recorded PROSPECTIVELY 2026-08-13, rl_docs/DYNREP.md) —
# k=1 within 5% of matched ridge + beat persistence; all other horizons
# must beat both baselines. Legacy G1 is reported alongside; the two
# v2pod FAIL verdicts stand unrevised.
#
# Stages (STAGE env, default all):
#   collect  v2-recipe dataset -> datasets/v3pod: 3 x 400 eps in
#            PARALLEL per-seed subdirs, merged + actor-share-verified
#            (merge_shards.py --require-actor noslip:0.05)
#   rest     obs-input encoder (dyn_v3pod_obs) + G1.1 gate; with
#            HOLDWALK=1 chains pod_holdwalk.sh (hold->walk A/B/C) on
#            a PASS
#
#   STAGE=collect nohup sh rl_move/dynamics/pod_v3_pipeline.sh \
#       > rl_move/dynamics/logs/pod_v3_collect.log 2>&1 &
#   STAGE=rest HOLDWALK=1 nohup sh rl_move/dynamics/pod_v3_pipeline.sh \
#       > rl_move/dynamics/logs/pod_v3_rest.log 2>&1 &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
DATA=rl_move/dynamics/datasets/v3pod
ENC_NAME=dyn_v3pod_obs
ENC=rl_move/dynamics/models/${ENC_NAME}.pt
LOG=rl_move/dynamics/logs
STAGE=${STAGE:-all}
HOLDWALK=${HOLDWALK:-0}
mkdir -p "$LOG"
echo "== pod_v3_pipeline stage=$STAGE start $(date -u +%FT%TZ) host=$(hostname)"

# Preflight: the exact ingredients whose silent absence caused the v2pod
# drift are hard requirements here.
$PY -c "import sys; sys.path[:0]=['linux_control']; \
import noslip_gait, tripod_gait; print('gait modules OK')" || {
    echo "V3_ABORT: noslip_gait/tripod_gait not importable — the drift" \
         "this pipeline exists to fix; sync code first"; exit 4; }
for Z in ppo_goal_cw_walk_longdist_r2 ppo_goal_cw_stance_dr10; do
    [ -f "rl_move/sim/policies/$Z.zip" ] || {
        echo "V3_ABORT: champion checkpoint $Z.zip missing" \
             "(ops.sh pushckpt it) — no silent mix renormalization"; exit 4; }
done

if [ "$STAGE" = "collect" ] || [ "$STAGE" = "all" ]; then
    for S in 0 1 2; do
        OMP_NUM_THREADS=2 $PY -m rl_move.dynamics.collect \
            --out "$DATA/s$S" --episodes 400 --seed $S \
            > "$LOG/v3pod_collect_s$S.log" 2>&1 &
    done
    wait
    for S in 0 1 2; do
        grep -q "^done:" "$LOG/v3pod_collect_s$S.log" || {
            echo "V3_ABORT: collect seed $S did not finish (see" \
                 "$LOG/v3pod_collect_s$S.log)"; exit 5; }
        grep -i "WARNING.*missing" "$LOG/v3pod_collect_s$S.log" && {
            echo "V3_ABORT: collect seed $S degraded its actor mix"; \
            exit 5; } || true
    done
    $PY -m rl_move.dynamics.merge_shards \
        --src "$DATA/s0" "$DATA/s1" "$DATA/s2" --out "$DATA" \
        --require-actor noslip:0.05 2>&1 | tee "$LOG/v3pod_merge.txt"
    grep -q "^merged " "$LOG/v3pod_merge.txt" || exit 4
    echo "V3_COLLECT_DONE $(date -u +%FT%TZ)"
fi

if [ "$STAGE" = "rest" ] || [ "$STAGE" = "all" ]; then
    [ -f "$DATA/meta.json" ] || { echo "V3_ABORT: dataset missing"; exit 4; }
    OMP_NUM_THREADS=20 $PY -m rl_move.dynamics.train \
        --data "$DATA" --name "$ENC_NAME" --input-set obs \
        --steps 40000 --lr-final-frac 0.05
    $PY -m rl_move.dynamics.eval_model --ckpt "$ENC" --data "$DATA" \
        --dump-latents 2>&1 | tee "$LOG/${ENC_NAME}_gate.txt"
    grep -q "GATE G1.1 .*PASS" "$LOG/${ENC_NAME}_gate.txt" || {
        echo "V3_G1_1_FAIL: encoder failed the REVISED gate on the" \
             "drift-fixed dataset; PPO not wired (DYNREP.md hard gate)." \
             "This closes the dataset-drift hypothesis — triage before" \
             "any further encoder work."; exit 3; }
    echo "V3_GATE_PASS $(date -u +%FT%TZ)"
    if [ "$HOLDWALK" = "1" ]; then
        DATA="$DATA" ENC="$ENC" exec sh rl_move/dynamics/pod_holdwalk.sh
    fi
fi
echo "POD_V3_PIPELINE_DONE $(date -u +%FT%TZ)"
