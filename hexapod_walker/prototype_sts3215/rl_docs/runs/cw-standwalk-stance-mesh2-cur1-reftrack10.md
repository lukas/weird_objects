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

**verdict**: Dose answer: k_rise_ref_track=10 does not fix the full-mix recipe, but it lands in a DIFFERENT basin than cur1's topple — the policy survives det hold and lower with ZERO terminations (both DR-0 and own-DR) by FREEZING in a five-leg stance with one front leg flagged aloft (video), while rise still terminates 5-6/6 and every sto panel collapses (hold sto 5/6 terms). hold ok=0/6 despite the survival (invalid stance), and NOTE: lower det scores 4/6 ok on the height criterion while the video shows the same frozen flag stance — the lower ok clause is lenient to a flag-leg freeze and should not be trusted alone. Read with refgain15 (k=15, 20M, 0/36 total collapse): more ref-gain is NOT the lever at any dose; at 2M it merely buys a freeze-and-survive local optimum that 20M optimization destroys. Diagnostic wave complete; rung-3 (hold-first + measured-load hold income) already in flight. Eval was run manually this cycle (watcher prestage skipped this run).

