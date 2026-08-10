# cw-walk-torquescale-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:03:40+00:00

**pod**: hexapod-mjx-train-10

**steps**: 6000000

**parent**: cw-walk-longdist-r2

**wandb_id**: pgjpu63e

**hypothesis**: OPERATOR WISHLIST 13b next isolated axis: TORQUE DROOP UNDER LOAD (battery sag / unit-to-unit torque spread, dr.torque_scale). Real STS3215s lose torque as the battery sags mid-walk and unit-to-unit vary; the champion has only ever felt full nominal torque. ISOLATED: dr-scale 0.0 with ONLY dr.torque_scale=0.80,1.05 (the full DR envelope). No pre-measured champion baseline this cycle (time-boxed triage) -- the harness champ-baseline eval should be run at verdict time before judging pass/fail, same as contactstiff/linklen. Prediction-if-true: exposure keeps median fwd>=1.2m at 30s with gv 12/12 0 term even at the low end of the torque range -- champion already has enough margin, this axis is free. Prediction-if-false: the low-torque draws collapse gait (worst draws far below the untrained-champion band, gv breaks or fwd<0.8m) -- torque droop joins the contact/current-pricing operator-calibration class like paddling. Strongest alternative: median holds but worst-draw tail suffers (partially trainable, judge per-episode).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.torque_scale=0.80,1.05, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; measure the untrained-champion baseline under the same cfg-set FIRST (same checkpoint, same eval) and require this run's worst-2 det draws to beat it outside noise; DR0 nominal retention det 6/6 gv, slip/m med<=1.24; frames watched det

