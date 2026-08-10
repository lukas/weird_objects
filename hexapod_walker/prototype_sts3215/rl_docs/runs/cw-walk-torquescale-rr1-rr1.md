# cw-walk-torquescale-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T03:03:40+00:00

**pod**: hexapod-mjx-train-10

**steps**: 6000000

**parent**: cw-walk-longdist-r2

**wandb_id**: pgjpu63e

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13b next isolated axis: TORQUE DROOP UNDER LOAD (battery sag / unit-to-unit torque spread, dr.torque_scale). Real STS3215s lose torque as the battery sags mid-walk and unit-to-unit vary; the champion has only ever felt full nominal torque. ISOLATED: dr-scale 0.0 with ONLY dr.torque_scale=0.80,1.05 (the full DR envelope). No pre-measured champion baseline this cycle (time-boxed triage) -- the harness champ-baseline eval should be run at verdict time before judging pass/fail, same as contactstiff/linklen. Prediction-if-true: exposure keeps median fwd>=1.2m at 30s with gv 12/12 0 term even at the low end of the torque range -- champion already has enough margin, this axis is free. Prediction-if-false: the low-torque draws collapse gait (worst draws far below the untrained-champion band, gv breaks or fwd<0.8m) -- torque droop joins the contact/current-pricing operator-calibration class like paddling. Strongest alternative: median holds but worst-draw tail suffers (partially trainable, judge per-episode).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.torque_scale=0.80,1.05, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; measure the untrained-champion baseline under the same cfg-set FIRST (same checkpoint, same eval) and require this run's worst-2 det draws to beat it outside noise; DR0 nominal retention det 6/6 gv, slip/m med<=1.24; frames watched det

**verdict**: NO-EFFECT (if-false confirmed, and duplicates the already-CLOSED torquedroop axis at a narrower range). OBSERVATIONS: own-cfg (dr.torque_scale=0.80,1.05) det 4/6 healthy (med fwd 1.43m, gate>=1.2 letter-met) but det eps4/5 crater (prog 0.45/0.43, slip/m 3.66/3.81, fwd 0.66-0.67m, six legs cycling on video, no falls/flag-leg -- slow paddle-shuffle). sto 6/6 clean (prog med 0.93). DR0 retention clean (det 6/6 gv, slip med 1.04, fwd 1.59m). NAMED BASELINE (measured this triage): champion longdist-r2 under the IDENTICAL torque_scale=0.80,1.05 spread, same seed/draws (logs/ckpt_eval/cw_walk_longdist_r2_torquescale80_base) matches episode-for-episode -- same 2 craters (prog 0.45/0.41, slip 3.84/3.80, fwd 0.69/0.69m) and same 4 healthy draws (med fwd 1.51m). Per the pre-registered gate, worst-2 det draws must BEAT the baseline outside noise -- they do not (0.66-0.67m trained vs 0.69m champion, inside noise, marginally worse if anything). INTERPRETATION: this is the SAME conclusion as cw-walk-torquedroop (0.60-1.05 range, FAIL/NO-EFFECT) at a narrower/easier range -- the champion already tolerates 0.80-1.05x torque for free (SKILLS.md: tolerates sag to ~0.75x FOR FREE), so 20M steps of exposure on a sub-range of an already-covered band bought nothing. This run duplicates the torquedroop axis (flagged post-hoc in c73 log) via the self-repair rename chain; no new science, closes cleanly. No requeue; torque_scale exposure ladder stays CLOSED.

