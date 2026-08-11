# cw-stand-loweranchor1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T22:14:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-anchorstate2

**wandb_id**: 925o7jiv

**hypothesis**: LOWER-TICK ANCHOR (2M, warm from cw-stand-holdbc1-hard1 via inherited init; config = anchorstate2 exactly + ONE axis: train.bc_anchor_lower=1.0): the leg-1 hold park survived six runs -- four pricing changes (incl. a measured 4x income cut) and two anchor variants that fixed leg 4 -- and det lower sits at 2/6 with the SAME dangling-leg class. The one documented incentive gap not yet attacked is the lower banks strict xfail: rise_posture_gate prices a lifted leg at pf=5/6, so one-leg-aloft keeps ~85% of the honest lower return (the deployed specialists 62-99mm dangling foot is its measured residue). Since pricing provably does not move this stack but anchor supervision provably does, loweranchor1 supervises lower ticks toward the banks own honest demonstration: FixedFootBodyIK descent anchored at the settled stance, body at the next commanded height (per-tick analytic solve, pool-restore safe; pinned in test_bc_anchor.py incl. a feet-planted chained descent). Prediction: det lower recovers to >= 3/6 AND, if the hold park is the lower-taught dangling habit generalizing (the shared-fingerprint hypothesis), leg-1 det-hold duty finally moves off 0.03.

**gate**: PASS if det lower success >= 3/6 with <= 1 fall AND det crouch-start rise valid >= 3/4 with zero tilt falls AND det flat/bridge rise not worse than hard1 AND hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet AND no flag-leg/tripod/hover cheat on video. PASS -> replaces hard1 as the stance deploy candidate (ship WITH goal-ramp profile); close RL_PLAN queue items for the stand line as SOLVED and record the anchor-per-mode recipe (rise state-aligned + hold q_nom + lower IK-descent) as the standard. PARTIAL (lower >= 3/6 but leg-1 duty still < 0.8) -> the lower-dangle and hold-park are INDEPENDENT mechanisms; the park suspect list is empty and the unified line falls back to hard1 + specialist handoff -- record the line closed. FAIL (lower still <= 2/6) -> the IK descent target is not learnable at coef 1.0 alongside the other anchors; inspect train/bc_anchor_loss before any retune.

**verdict**: LOWER SOLVED, park REGRESSED -- outcome outside all pre-registered branches, and it names the next mechanism. The IK-descent lower anchor delivered det lower 6/6 AND sto lower 6/6 (from 2/6; zero falls anywhere det) -- the lower-bank xfail lever works exactly as specced. But det-hold duty flipped BACK to the two-leg park [1.0, 0.02, 0.89, 1.0, 0.02, 0.91] (leg 4 was 0.93-0.95 in anchorstate1/2) and det flat rise re-stalled (96mm short, feet planted -- the under-drive class, worse than anchorstate1s 62mm). hold/sto misses are all the known >2A current soft-flag class, not a new cheat. INTERPRETATION: leg-4s park now toggles WITH the anchor mix -- fixed when rise+hold anchors ran, regressed when lower pairs joined -- and flat-rise drive weakened in the same step. All three anchors share ONE ring buffer and ONE mse: adding thousands of lower pairs dilutes hold and rise supervision (uniform sampling = per-mode strength proportional to emission share). This refutes the PARTIAL branch premise (park independent of lower) and supersedes the record-line-closed clause: ANCHOR DILUTION is a new, directly testable mechanism, not another blind retune. Next (specced): stratified per-mode anchor minibatch sampling (train.bc_anchor_stratified, mode tag per pair, equal per-mode quotas), everything else frozen at loweranchor1. hard1 stays deployed; loweranchor1 is the strongest lower checkpoint to date if a lower specialist is ever wanted.

