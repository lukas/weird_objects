# cw-standwalk-stance-mesh2-holdonly1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T06:47:36+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: khyece06

**hypothesis**: Plain English: cur1/cur1-seed1 both failed EVERY mode including hold (which starts from an already-planted pose and just needs to stand still) -- hold is only 10% of the training goal-mix (0.1 of 1.0) alongside rise/lower at 0.45 each, so a plausible root cause is that hold gets starved of gradient signal, not that balance itself is unlearnable on the mesh model. This arm isolates hold: same cur1 pricing/recipe, goal-mix forced to hold=1.0/rise=0/lower=0, 2M-step discovery budget. Prediction-if-true (starved, not broken): quiet standing emerges within 2M -- DR-0 hold panel gets several ok/6 with roll settling, no tilt_roll/over_current pattern. Prediction-if-false: hold-only ALSO fails the same way (tilt_roll fall, similar reward trajectory to cur1's hold slice) -- balance itself is broken on the mesh/100Hz stack (a gains/PID/contact issue, not a curriculum-weight issue), and the next escalation is a physics/servo-gain audit, not a goal-mix split. Strongest alternative: 2M is too short even for hold-only (rung needs more budget regardless of split).

**gate**: Discovery/canary read at 2M: pod_eval stance panel, hold mode only, n=6 det DR-0. PASS-qualitative: majority (>=4/6) survive the full 15s without a tilt_pitch/tilt_roll/over_current termination, roll_tail settling <5deg. FAIL: 0-2/6 survive or the same tilt_roll signature as cur1 -- escalate to a physics/gains audit instead of more curriculum splits.

