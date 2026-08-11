# cw-stand-anchorstate2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T21:32:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-anchorstate1

**wandb_id**: 6wln41w9

**hypothesis**: LOOKAHEAD DOSE (2M, warm from cw-stand-holdbc1-hard1 via inherited init; config = anchorstate1 exactly + ONE axis: bc_anchor_lookahead_s 0.25 -> 0.5): anchorstate1 delivered the first park-fingerprint movement in five runs (leg 4 duty 0.01 -> 0.93; leg 1 still 0.04) -- anchor-bleed CONFIRMED as a mechanism -- but det flat rise degraded to a STALL 62mm short with feet planted and det lower grew three tilt_pitch falls. Stall diagnosis: the legacy clock anchor dragged the policy through the rise regardless of its actual progress; the state anchor points only 0.25s up-path from wherever the policy IS, so a stalled policy receives barely-moving supervision (measured pre-launch: pure target-chaining advances ~1 ref tick per 60 steps at 1-tick lookahead; 0.25s fixed chaining but training says it is still too weak against PPO noise). 0.5s doubles the forward pull while keeping the state-locality that fixed leg 4 (crouch/plant-adjacent states still anchor to the planted tail -- the lookahead clamps at the path end). Tests whether flat-rise drive and hold-park fix can coexist on the lookahead axis.

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND det flat/bridge rise not worse than hard1 AND hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet AND det lower success >= 3/6 with <= 1 fall AND no flag-leg/tripod/hover cheat on video. PASS -> replaces hard1 as the stance deploy candidate (ship WITH goal-ramp profile) and closes RL_PLAN queue 2a state-alignment CONFIRMED. FAIL with leg-1 park persisting but flat rise restored -> the lookahead axis is exhausted for the park; the remaining leg-1 suspect is the lower-bank dangling-foot incentive gap (documented xfail) -- open THAT as its own specced lever. FAIL with flat rise still stalled -> the state-aligned anchor cannot both drive and localize at any lookahead; consider hybrid indexing (clock floor: j = max(j_state, j_clock)) as a NEW specced axis, do not tune blindly past two doses.

**verdict**: LOOKAHEAD AXIS VALIDATED for its own costs, leg-1 park now isolated. bc_anchor_lookahead_s 0.25 -> 0.5 fixed BOTH anchorstate1 regressions exactly as hypothesized: det flat rise restored to 1/1 (was 0/1 stall 62mm short), det lower falls 3 -> 0, crouch rise retained 4/4 det + 3/3 sto, bridge 1/1, leg 4 stays recovered (duty 0.95). Hold: det 6/6 + sto 6/6 success, valid_plant 11/12 (>= 10/12 gate). REMAINING FAILS: det-hold per-foot duty [0.99, 0.03, 0.98, 0.96, 0.95, 0.98] -- leg 1 parks at 0.03 for the SIXTH consecutive run, unmoved by four pricing changes, the state-aligned anchor, and two lookaheads; and det lower 2/6 (< 3/6 band; sto 4/6, zero falls -- the dangling-foot shortfall, not instability). VERDICT: FAIL on the leg-1 duty clause and lower clause, but this is the pre-registered (leg-1 persists + flat restored) branch: the anchor axis is EXHAUSTED for the park. Next lever per pre-registration: the lower-bank dangling-foot incentive gap (documented xfail) -- leg-1/leg-2 dangling is the shared fingerprint between det-lower shortfall and the hold park, and it is the one documented incentive gap not yet attacked. Open it as its own specced lever. hard1 stays deployed. NOTE: anchorstate2 is otherwise the strongest unified-stand checkpoint to date (crouch+flat+bridge rise, six-foot-minus-one hold, zero falls anywhere det).

