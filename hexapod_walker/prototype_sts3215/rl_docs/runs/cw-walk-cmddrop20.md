# cw-walk-cmddrop20

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T18:00:27+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: m5z03wmc

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run): lost SyncWrite commands are a REAL defect of the robot's serial servo bus (servos chase the stale goal for a tick when a write drops). ISOLATED axis via dr.<field> overrides: dr-scale 0.0 with ONLY dr.cmd_drop_prob_max=0.20 (per-episode drop prob u(0,0.20) per control tick; full-DR default max is 0.05, champion trained at 0) - one variable off the no-DR champion. Plain: the robot should keep walking smoothly even when up to a fifth of its commands never arrive. Prediction-if-true: gait holds across the drop spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - bus-drop robustness is trainable by exposure and joins the transfer recipe. Prediction-if-false: high-drop draws destabilize the gait (jitter, falls, or progress collapse) - command dropout needs smoother action targets or filtering, not exposure. Strongest alternative: policy compensates by moving slower / smaller strides on all draws - check cadence + stride vs champion on low-drop draws. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.cmd_drop_prob_max=0.20, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det

**note**: OVERLAP FLAG (c59): same dr.cmd_drop_prob_max axis as cw-walk-cmddrop10 (c57, max 0.10 vs this 0.20) — 3 concurrent refill cycles drew from the same dry READY well. Triage the pair as ONE intensity-ladder study. LESSON FROM torquedroop (c59): before verdicting, run the PARENT (longdist-r2) under the same own-cfg drop spread — the champion may already tolerate command drops for free, and the gate letter cannot distinguish that from a training effect.

