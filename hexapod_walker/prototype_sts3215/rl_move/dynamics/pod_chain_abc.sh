#!/bin/sh
# pod_chain_abc.sh — chain the hold->walk A/B/C cohort behind the scale
# sweep (session A, operator next-steps 08-14).
#
# Waits for POD_SCALE_SWEEP_DONE in logs/scale_sweep_gpu.log, then picks
# the first gate-passing sweep cell in a FIXED preference order (declared
# here BEFORE any gate verdicts beyond S_h16_small were known: mid
# capacity + short history + large data first — cheapest robust choice
# for PPO wiring), and execs pod_holdwalk.sh with fresh seeds (operator:
# NEW cohorts need seeds >= 5).
#
#   nohup sh rl_move/dynamics/pod_chain_abc.sh > .../chain_abc.log &
set -e
cd "$(dirname "$0")/../.."
LOG=rl_move/dynamics/logs
SWEEP_LOG=${SWEEP_LOG:-$LOG/scale_sweep_gpu.log}
SEEDS=${SEEDS:-"5 6 7"}

echo "== pod_chain_abc start $(date -u +%FT%TZ) waiting on $SWEEP_LOG"
while ! grep -q "POD_SCALE_SWEEP_DONE" "$SWEEP_LOG" 2>/dev/null; do
    sleep 60
done
echo "sweep done at $(date -u +%FT%TZ); selecting encoder"

ENCNAME=""
for CAND in M_h16_large M_h48_large S_h16_large S_h48_large \
            L_h16_large L_h48_large M_h16_small S_h16_small; do
    if grep -Eq "GATE G1(\.1)? .*PASS" \
            "$LOG/dyn_scale_${CAND}_gate.txt" 2>/dev/null; then
        ENCNAME="dyn_scale_$CAND"
        break
    fi
done
[ -n "$ENCNAME" ] || { echo "CHAIN_ABC_ABORT: no gate-passing cell"; exit 3; }

case $ENCNAME in
    *_large) DATA=rl_move/dynamics/datasets/v3scale_large;;
    *)       DATA=rl_move/dynamics/datasets/v3scale_small;;
esac
echo "CHAIN_ABC_PICK encoder=$ENCNAME data=$DATA seeds=[$SEEDS]"

SEEDS="$SEEDS" DATA="$DATA" \
    ENC="rl_move/dynamics/models/${ENCNAME}.pt" \
    exec sh rl_move/dynamics/pod_holdwalk.sh
