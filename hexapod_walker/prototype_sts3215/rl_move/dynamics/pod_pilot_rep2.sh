#!/bin/sh
# pod_pilot_rep2.sh — dynrep G1 replication on the INTENDED v2 recipe
# (RISE_WALK_NEXT_48H P2 "Continue dynrep cleanly", 08-13).
#
# Context: v2pod drifted from the successful laptop-v2 recipe because
# noslip_gait.py was laptop-local — collect.py silently reassigned the
# noslip actor's 10% share to tripod, and the more-periodic tripod data
# is exactly what strengthens the ridge baseline the encoder must beat.
# Both v2pod seeds then failed G1 at k=1 only. noslip_gait.py is in git
# since decb1fa; collect.py now HARD-FAILS on a missing mix component
# (no --allow-degraded-mix here, ever: this run exists to test the
# dataset-recipe-replication hypothesis, so the recipe must be exact).
#
# This script: recollect (v2pod2) -> retrain encoder -> gate on the
# ORIGINAL preregistered G1 read from the report JSON (gate_g1_pass —
# NOT the console text, and NOT the revised G1.1, which the directive
# forbids using post hoc). The A/B/C pilot cohort only runs on a G1
# PASS, unchanged from pod_pilot_rep.sh. On PASS the follow-up priority
# is dynrep on the rise->walk unified task, not more hold/lower arms.
#
#   nohup sh rl_move/dynamics/pod_pilot_rep2.sh > .../pod_pilot_rep2.log &
set -e
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
DATA=rl_move/dynamics/datasets/v2pod2
ENC_NAME=dyn_v2pod2_obs
ENC=rl_move/dynamics/models/${ENC_NAME}.pt
LOG=rl_move/dynamics/logs
mkdir -p "$LOG"
echo "== pod_pilot_rep2 start $(date -u +%FT%TZ) host=$(hostname)"

# 0) recipe preflight: the exact-known-good collector must be present.
$PY - <<'EOF'
import sys
from pathlib import Path
root = Path("linux_control")
sys.path.insert(0, str(root))
import noslip_gait  # hard requirement — the drifted ingredient
print("preflight: noslip_gait importable from", root)
EOF

# 1) dataset, intended v2 recipe: 3 collect seeds x 400 eps.
#    collect.py hard-fails if ANY mix component would be substituted.
for S in 0 1 2; do
    $PY -m rl_move.dynamics.collect --out "$DATA" --episodes 400 --seed $S
done
$PY - "$DATA" <<'EOF'
import json, sys
from pathlib import Path
meta = json.loads((Path(sys.argv[1]) / "meta.json").read_text())
bad = [r for r in meta["runs"] if r.get("mix_degraded")]
assert not bad, f"degraded-mix runs present: {bad}"
counts = {}
for r in meta["runs"]:
    for k, v in r["actor_counts"].items():
        counts[k] = counts.get(k, 0) + v
print("recipe check OK; pooled actor counts:", counts)
assert counts.get("noslip", 0) > 0, "no noslip episodes collected"
EOF

# 2) the PPO-facing obs-input encoder, v2 recipe unchanged
OMP_NUM_THREADS=20 $PY -m rl_move.dynamics.train \
    --data "$DATA" --name "$ENC_NAME" --input-set obs \
    --steps 40000 --lr-final-frac 0.05

# 3) gate on the ORIGINAL G1 from the report JSON (newest report for
#    this encoder). G1.1 is recorded in the same JSON for evidence but
#    MUST NOT gate this replication (directive: no post-hoc weakening).
$PY -m rl_move.dynamics.eval_model --ckpt "$ENC" --data "$DATA" \
    --dump-latents 2>&1 | tee "$LOG/${ENC_NAME}_gate.txt"
$PY - "$ENC_NAME" <<'EOF'
import json, sys
from pathlib import Path
reports = sorted(Path("rl_move/dynamics/logs").glob(
    f"eval_{sys.argv[1]}_*.json"))
assert reports, "no eval report written"
rep = json.loads(reports[-1].read_text())
g1 = bool(rep.get("gate_g1_pass"))
print(f"ORIGINAL G1 (gate_g1_pass) on intended recipe: "
      f"{'PASS' if g1 else 'FAIL'}   [{reports[-1].name}]")
if not g1:
    print("POD_PILOT_REP2_G1_FAIL: the intended-recipe dataset did not "
          "fix k=1 — the recipe-drift hypothesis is DISCONFIRMED; "
          "escalate to the operator (do not retry seeds, do not use "
          "G1.1, do not launch A/B/C).")
    raise SystemExit(3)
EOF

# 4) A/B/C pilot cohort (only reached on a G1 PASS), seeds 1..3
for SEED in 1 2 3; do
    (
        set -e
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task hold --seed "$SEED" \
                --steps 150000 --encoder "$ENC" --anchor-data "$DATA" \
                --name "pilot2_hold_${C}_s${SEED}"
        done
        for C in A B C; do
            OMP_NUM_THREADS=4 $PY -m rl_move.dynamics.train_ppo_transfer \
                --condition "$C" --task lower --seed "$SEED" \
                --steps 150000 --encoder "$ENC" --anchor-data "$DATA" \
                --init-from "rl_move/dynamics/models/ppo_pilot2_hold_${C}_s${SEED}.zip" \
                --name "pilot2_lower_${C}_s${SEED}"
        done
        echo "PILOT2_COHORT_DONE seed=$SEED"
    ) > "$LOG/pilot_rep2_s${SEED}.log" 2>&1 &
done
wait
n_done=$(grep -l "PILOT2_COHORT_DONE" "$LOG"/pilot_rep2_s?.log | wc -l)
echo "cohorts done: $n_done/3"
echo "POD_PILOT_REP2_DONE $(date -u +%FT%TZ) — on PASS, next dynrep"
echo "priority is the rise->walk unified task (directive P2), not more"
echo "hold/lower arms."
