# cw-standwalk-stance-mesh2-cur1-reftrack10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T07:05:39+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: fhk6ir7p

**hypothesis**: Plain English: cur1's full-mix rise topples instead of following the demonstrated rise reference (k_rise_ref_track=2.0). Companion to riseonly1's curriculum-share test: this arm keeps the ORIGINAL full goal-mix (hold=.1/rise=.45/lower=.45) but multiplies the tracking weight 5x (k_rise_ref_track=10.0), testing the other candidate fix (the weight is too weak vs other shaping terms, not that rise is undertrained by curriculum share). Prediction-if-true: rise's tilt_pitch/over_current-at-rise rate drops and DR-0 rise episodes end closer to valid plant even with hold/lower still competing for gradient. Prediction-if-false: still topples -- rules out a pure tracking-weight fix; combine with riseonly1's read to decide whether the fix is curriculum (split), pricing (weight), or budget (neither) at all.

**gate**: Discovery/canary read at 2M: pod_eval stance panel, all 3 modes, n=6 det DR-0. Read jointly with riseonly1: if reftrack10 fixes rise but riseonly1 doesn't (or vice versa), that names the correct lever for the real rung-3 launch; if neither fixes it, escalate to budget/teacher-signal instead of more reward-weight iteration.

**verdict**: FAIL for its own tested hypothesis: 5x rise-tracking weight (k_rise_ref_track 2.0->10.0, full goal-mix) does NOT fix rise -- 0/6 det, 0/6 sto, same over_current/tilt_pitch signature as cur1 (confirms refgain15's independent 15.0-dose finding: ref-tracking weight is not the rung-2 bottleneck at any tested dose 2/10/15). NOTABLE SIDE FINDING (not the tested hypothesis, flagging for the rung-3 design): LOWER mode alone did comparatively well at this 2M read despite full hold/rise/lower competition -- 4/6 det success and 4/6 own-DR(0.2) success, all clean (no term, height_err <18mm) -- a much better lower read than cur1's own full-20M-run 0/36 collapse. Read together with reward quarters that start positive then decline (32.1/-4.6/-83.3/-149.9, unlike cur1's flat-negative-from-Q1 shape): this is consistent with the run finding a partially-good policy early and drifting AWAY from it with more training under this exact pricing (misalignment, not undertraining) -- worth a dedicated look (early-checkpoint pull + panel read at intermediate steps) before assuming more budget helps the full-mix recipe. Not launching a follow-up this cycle (single 2M canary, no reproduction) -- flagging in STATUS for whichever cycle designs rung-3.

