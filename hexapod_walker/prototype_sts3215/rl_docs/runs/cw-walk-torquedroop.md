# cw-walk-torquedroop

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:40:46+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: rg6gujl6

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run): torque droop / battery sag is a cheap-servo sim2real axis (STS3215 under load + voltage sag). ISOLATED axis via cycle-49 dr.<field> overrides: dr-scale 0.0 with ONLY dr.torque_scale=0.60,1.05 randomized (down to 60% torque, wider than the standard 0.80-1.05 envelope) - one variable off the no-DR champion. If-true: gait holds across the torque spread (own-cfg harness gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - torque robustness is trainable by exposure and joins the transfer recipe. If-false: low-torque draws collapse height/progress - torque sag needs explicit adaptation (estimator rung), not exposure. Strongest alternative: policy survives by slowing cadence/crouching - check cadence + height_err vs champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.torque_scale=0.60,1.05, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: FAIL (refuted, NO-EFFECT). OBSERVATIONS: own-cfg (torque 0.60-1.05) det 4/6 healthy (med fwd 1.38, slip ~1.0-1.2) but det eps4/5 collapse (prog 0.41/0.39, slip/m 4.7/3.9, near-stationary paddling on frames, Imax pinned 1.65A; no falls/flag legs); sto 6/6 prog med 0.96; DR0 retention clean (det gv 6/6, slip/m 1.05, fwd 1.55 = champion parity). NAMED BASELINE (dig-in): parent longdist-r2 under the IDENTICAL spread fails the same two det draws (prog 0.45/0.35, slip 3.5/4.4) and matches on healthy draws — all deltas inside eval noise. INTERPRETATION: the gate letter passes for BOTH policies, so it measured pre-existing champion tolerance, not a training effect; torque <=~0.7x is a transport boundary of the paddle gait (torque-starved), and exposure does not move it. VERDICT: exposure lever for torque droop CLOSED; champion already covers ~0.75-1.05x; sub-0.75x waits for the estimator rung (explicit adaptation), no continuation. Baseline artifacts: logs/ckpt_eval/cw_walk_longdist_r2_torquedroop_base.

