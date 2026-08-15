# cw-stand-postlower3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-15T01:20:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-postlower2

**wandb_id**: 44wowzno

**hardware_ready**: False

**hypothesis**: Teach the standing robot to stand back up right after it sits down -- the one boundary that still fails in the joystick session -- by practicing sit-down-then-stand-up as ONE continuous episode instead of teleporting it to a cold reconstructed sitting pose. Both cold-spawn bank arms (postlower1/2) failed and the dig-in showed why: reconstructed sitting starts are off-distribution and their rise targets were mechanically impossible (~5cm above standing), so this arm switches mechanism to in-context sequences: new goal.mode_seq_stance=0.5 (default-off key, stance-only grammar rise->hold->lower->rise, tests test_mode_seq_stance.py, snapshot exp/cw-stand-postlower3) makes half of all episodes two-segment stance sequences -- a lower-first sequence IS the post-lower rise with real transition context (warm policy state, canonical per-family re-anchor, blend window), and the mid-sequence rise target is anchored at the sequence's own commanded stand height, reachable by construction. Bank exposure is OFF (frac=0); everything else is the footlow2_hard1 recipe warm from footlow2_hard1. Prediction-if-true: the 2M artifact completes in-context lower->rise sequences and the pre-registered Cohort c3 bulk read moves sto post-lower rise above the parent's 0.801 with cold rise/lower retention intact. Prediction-if-false: post-lower rise stays at/below parent despite clean sequence training -- transition context is not the binding constraint, pointing at belly-calibrated rise pricing (rise_ref_track) as the next mechanism. Strongest alternative: the 50% sequence diet dilutes single-mode rise quality the way 35% bank exposure did (caught by the c3 retention strata clause).

**gate**: Discovery, 2M, judged on the PRE-REGISTERED Cohort c3 bulk gate (SESSION_BULK_GATE.md 'Cohort c3', fresh held-out banks det 940000../sto 950000.., candidate spec-pl3, registered before training). Full PASS = all five c2-style clauses (sto post-lower rise >=0.90 with CI lower >0.842, det session >=0.95, det post-lower >=0.967, first-rise strata + lower retention, no visual regression vs footlow2_hard1). PARTIAL (mechanism confirmed) = sto post-lower rise CI-separated ABOVE parent 0.801 but short of 0.90 with retention clean -> one 6M hardening rerun judged on fresh cohort c4. FAIL = post-lower rise <= parent OR any retention/visual clause broken -> next change must be mechanism-level (sequence-RSI or rise pricing), never a dose/diet resweep.

**verdict**: FAIL (Cohort c3, n=600 held-out: det session 0.413 vs parent 0.967; det post-lower rise 0.419 vs 0.967; sto 0.631 vs 0.801; cold rise/lower retention intact). DIG-IN root cause: the mode_seq_stance rise schedule starts at belly-frame 0 (blend down + 1s hold at 0), so training PAID a re-descend/splay-to-belly + flat-demo redo after every lower (detour visible in fail AND success re-renders; over_current 166/172 det post-lower falls; det<sto because det commits to the taught detour; training telemetry read ok because on-policy the detour completes and reward pays it). Fix landed same cycle: goal.mode_seq_rise_from_h (default-off, tests green, snapshot exp/cw-stand-postlower4); follow-up cw-stand-postlower4 judged on pre-registered Cohort c4. Full chain: SESSION_BULK_GATE.md 'Cohort c3 DIG-IN VERDICT'.

