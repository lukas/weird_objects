# cw-stand-minfeet1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T00:40:43+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-stand-anchormix1-r1

**wandb_id**: 7wkkimqn

**hypothesis**: Make five-foot standing UNPROFITABLE: six straight stand runs (crouchrise1/2/3, holdload1, anchorstate1/2, anchormix1-r1) end hold with exactly ONE foot parked (contact duty 0.01-0.04) because the current load gate multiplies per-foot factors with a 0.5 floor — a single parked foot costs only half pay, and a five-foot stance at half pay is sufficient and cheaper; supervision changes only moved WHICH foot parks. This arm is the PRE-REGISTERED reopen lever (CURRENT_TRUTHS): ONE CHANGE vs cw-stand-anchormix1-r1 — reward.hold_feet_load_min=1.0 (new, snapshot feb5c97, bank green): hold income is priced on the WORST foot's measured load, so the one-foot park drops from 0.49x quiet (measured in the bank) to scraps (0.1x floor, with linear slope as the foot loads). Also the FIRST run with per-mode bc_anchor_loss telemetry (landed same snapshot, the ruling's other precondition): the hold-mode anchor-loss trace decides between 'the anchor itself teaches the park' (loss LOW while parked) and 'PPO defies the supervision' (loss HIGH) regardless of outcome. Prediction-if-true: det hold ends with all six feet loaded (duty >=0.5 each), rise/lower retention holds. Prediction-if-false: the park persists paying scraps — pricing family then TERMINALLY closed, and the anchor-loss telemetry names the next lever. Strongest alternative: the harsher gate destabilizes hold into stepping/adjustment churn (stillness factor + retention gate catch it).

**gate**: PASS if det hold: every foot contact duty >=0.5 in all det hold episodes (the fingerprint metric valid_plant misses) AND hold valid_plant >=5/6 AND retention vs parent anchormix1-r1: det+sto lower >=5/6 zero falls (parent 6/6), det crouch rise >=3/4 (parent 4/4), flat rise stall no worse than parent (106mm under-drive band). FAIL-A (park persists at scraps, hold anchor-loss LOW): the anchor teaches the park — pricing family terminally closed, next lever is anchor-side (mask/repair hold supervision at unloaded-foot reference states), spec first. FAIL-B (park persists, hold anchor-loss HIGH): PPO defies both pricing and supervision — escalate with telemetry, no more solo-lever arms. FAIL-C (hold destabilizes into churn or rise/lower regress): one dose retry at hold_feet_load_min=0.5; double-miss closes. Any PASS is a deployable stance candidate: same Gate 0 contract as hard1, export + handoff composition re-run next.

