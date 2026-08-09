# cw-walk-badstart

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:36:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: mpyvrdq1

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b next isolated axis: BAD START POSES. On real power-up the legs are wherever the last session left them - the sim always starts from the clean nominal plant. ISOLATED: dr-scale 0.0 with ONLY the bad-start mechanism on at its full-DR values (dr.bad_start_prob=0.25, dr.bad_start_deg=8,35; up to 3 joints start 8-35deg off; champion trained with 0). One mechanism = one variable (prob+magnitude enable the same event; magnitudes are zeroed at dr-scale 0 so both must be set). Plain: recover from a sloppy starting pose and walk off cleanly instead of tripping on the first step. Prediction-if-true: bad-start episodes recover in the first ~2s and gate holds (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) with DR0 retention clean - boot-pose robustness is trainable by exposure. Prediction-if-false: off-pose starts cause first-second falls or the policy learns a lurching recovery that contaminates the nominal gait (retention slip/prog erodes). Strongest alternative: recovery works but costs distance only in bad-start draws - per-episode spread will show a bimodal fwd distribution, judge median + frames. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.bad_start_prob=0.25 + dr.bad_start_deg=8,35, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: FAIL on the letter (partial result). Own-cfg det med fwd 1.17m vs gate 1.2m. Bad-start draws recover upright every time (0 term, gv 12/12, no lurch in frames) but transport only 0.83-0.85m at slip 2.4-3.2 — the pre-registered strongest alternative (recovery costs distance) is what happened. DR0 nominal retention letter-passes (gv 6/6, slip 1.13) but fwd med shaded to 1.32m vs champion 1.57m, outside noise. Boot-pose recovery is trainable, not free; no identical requeue (continuations closed); badstart-dr05 compose cancelled.

