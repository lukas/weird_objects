# cw-stand-anchormix1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T23:06:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-loweranchor1

**wandb_id**: at00cbie

**hypothesis**: ANCHOR DILUTION FIX, relaunch of cw-stand-anchormix1 (first launch died on tick one: warm-start checkpoints pickle pre-tag anchor buffers, _bc_mode missing -- fixed + regression-tested, commit dc5b7f4). Same one-axis test: config = loweranchor1 exactly + train.bc_anchor_stratified=1.0. loweranchor1 measured anchor dilution directly (uniform sampling makes per-mode supervision track emission share: lower joined -> lower 6/6 but hold leg-4 duty 0.95 -> 0.02 and flat rise re-stalled). Stratified per-mode quotas make each skills anchor full-strength regardless of goal mix. Prediction: lower keeps >= 3/6, hold recovers >= the anchorstate2 level (leg-4 duty >= 0.8), flat rise recovers; if full-strength hold supervision is what the six-foot plant needed, leg 1 finally moves.

**gate**: PASS if det lower success >= 3/6 with <= 1 fall AND det crouch-start rise valid >= 3/4 with zero tilt falls AND det flat/bridge rise not worse than hard1 AND hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet AND no cheat on video. PASS -> unified stance line SOLVED: replaces hard1 as deploy candidate (ship WITH goal-ramp profile), record the per-mode-anchors + stratified-sampling recipe as standard. PARTIAL (everything recovers except leg-1 duty < 0.8) -> dilution fixed the seesaw but leg-1 is a separate habit; line stops here on pre-registration -- hard1 + specialist handoff stays, record closed, no further blind axes. FAIL (lower or flat rise regress despite stratification) -> dilution theory wrong or incomplete; inspect per-mode train/bc_anchor_loss before anything else.

