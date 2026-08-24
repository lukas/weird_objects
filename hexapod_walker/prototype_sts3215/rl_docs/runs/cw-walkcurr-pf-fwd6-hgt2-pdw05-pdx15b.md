# cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T01:05:53+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-hgt2

**wandb_id**: shjm1zyx

**hypothesis**: Plain English: dose sibling of hgt2-pdw05b -- same park_duty_window_s de-confound (2.0->0.5s so the duty-history buffer fills before the 2.0s height-drop termination fires) PLUS 1.5x k_park_duty (0.08->0.12) to test whether the de-confounded charge merely needs to fire vs. needs to fire HARDER to out-price the frozen/collapsing stance. A 3x dose (0.24) was tried in the bank first and REJECTED (breaks the required park_gated>belly_sit_gated ranking -- overpriced the honest 'refuse to move' park contrast behavior below the still-fast-terminating belly_sit); 1.5x is the largest dose that stays bank-legal (test_walkcurr_pf_hgt_gait_beats_belly_sit/ranking_still_holds green at 'tight_pdw05_pdx1p5'). Same sigma=11/drop=25/grace=2.0, same fresh 2M discovery init, not warm-started. Prediction-if-true: freeprog crosses toward/past 0 faster or further than the pdw05b sibling. Prediction-if-false (same or worse than pdw05b): the extra duty pricing doesn't help once the confound itself is fixed -- read both jointly, whichever (or neither) moves decides if height-gate can still land before escalating to a new foot-contact mechanism or BC-kickstart.

**gate**: Same rung-1 gate as hgt2-pdw05b; PASS = rung-1 lands. Read jointly with hgt2-pdw05b (window-fix-only) before any further height-gate calibration or new-mechanism escalation.

**verdict**: Replicate confirmation of the already-FAILed cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15 (byte-identical extra_args -- same park_duty_window_s=0.5 confound fix + 1.5x k_park_duty=0.12 dose, same hgt2 parent; this launch is the auto-suffixed 'b' retry after an earlier attempt at the same name was REFUSED on a stale-code-marker race, independent fresh init). Evidence: env/walk_freeprog_score flat -0.10..-0.11 the whole 2M run (identical band to pdx15 and to plain-dose pdw05), reward quarters 24.2/13.8/13.7/13.6 (falling then flat, not rising). Own-cfg gate: 6/6 det + 6/6 sto TERM walk_low_height, gait_valid True (no sacrificed legs, duty spread 0.24-0.78, swing 6-12/leg) but forward_dist ~0.01m/25s, slip/m 6.3-6.8, height_err_end 65-94mm. Video (walk_det_0_sheet.png) shows the identical progressive splay-and-sink into a low wide crouch across all 6 frames, zero net translation, cut off by the height-drop safety termination -- visually indistinguishable from the pdx15/pdw05 siblings. Aligned FAIL per 08-21 (reward not rising, eval flat/bad, adequate 2M budget, no optimizer crush -- clip_fraction healthy). Second independent seed confirming: the park_duty-class charge (confound-fixed, at either bank-legal dose 0.08 or 0.12) is CLOSED as a rung-1 unlock mechanism -- no new information, does not reopen the class. Track escalation stays exactly where the pdx15/actbias1/actbias1-pdw05 chain already pinned it: raise k_park_duty dose was itself just closed by actbias1-pdw05's read too, so the next real lever is the direct minimum-total-foot-contact charge (with a hard termination, per the stagea-slip1 lesson) or flagging BC-kickstart to the operator -- no further park_duty dose/confound arms.

