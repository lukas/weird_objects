# cw-stand-anchormix1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T23:03:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-loweranchor1

**hypothesis**: ANCHOR DILUTION FIX (2M, warm from cw-stand-holdbc1-hard1 via inherited init; config = loweranchor1 exactly + ONE axis: train.bc_anchor_stratified=1.0): loweranchor1 measured the dilution mechanism directly -- all three anchors share ONE ring buffer and ONE mse with uniform sampling, so per-mode supervision strength tracks emission share; adding lower pairs made lower 6/6+6/6 (from 2/6) but regressed hold leg-4 duty 0.95 -> 0.02 and re-stalled flat rise 96mm short. The seesaw across anchorstate1/2/loweranchor1 tracks the pair mix, not the skills. Stratified sampling (bc_mode tag per pair: rise 0, hold 1, lower 2; equal per-mode minibatch quotas, pinned in test_bc_anchor.py incl. a 50:1 imbalance drawing ~50% quota) makes each skills anchor full-strength regardless of goal mix. Prediction: lower keeps >= 3/6 (its pairs did not weaken) AND hold recovers at least the anchorstate2 leg-4 fix (duty >= 0.8 on five feet) AND flat rise recovers -- and if hold supervision at full strength is what a quiet six-foot plant needed all along, leg 1 finally moves.

**gate**: PASS if det lower success >= 3/6 with <= 1 fall AND det crouch-start rise valid >= 3/4 with zero tilt falls AND det flat/bridge rise not worse than hard1 AND hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet AND no cheat on video. PASS -> the unified stance line is SOLVED: replaces hard1 as deploy candidate (ship WITH goal-ramp profile), record the recipe (per-mode anchors + stratified sampling) as standard. PARTIAL (everything recovers except leg-1 duty < 0.8) -> dilution fixed the mix seesaw but leg-1 is a genuinely separate habit; the line stops here on pre-registration -- hard1 + specialist handoff stays, record closed, no further blind axes. FAIL (lower or flat rise regress AGAIN despite stratification) -> the dilution theory is wrong or incomplete; inspect train/bc_anchor_loss per-mode before anything else.

**failed_reason**: run never appeared as 'running' in W&B within 240s

