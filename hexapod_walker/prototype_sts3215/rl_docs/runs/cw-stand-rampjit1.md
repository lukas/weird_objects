# cw-stand-rampjit1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T10:51:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-footz1-r1

**wandb_id**: 74bwxuy8

**hypothesis**: Make the deployed stance survive the JOYSTICK's goal ramps, not just the training profile: the 08-11 model tour showed holdbc1_hard1 tips deterministically when asked to sit from the 142mm walk-plant frame and stalls its belly rise at 55mm under the interactive ramp — a profile-overfit defect (training always used rise_ramp_s=6.0 / lower fixed ramps, so the policy memorized one timing). This arm warm-retrains the champion with goal.rise_ramp_jitter=0.3 + goal.lower_ramp_jitter=0.3 (the landed default-off axis, bank green: jitter varies ramp TIMING only, never targets/holds; bc_anchor_foot_z stays 0 = bit-exact absent, so ramp jitter is the ONE variable vs holdbc1-hard1). Prediction-if-true: rl_move.sim.eval_session hard gates pass (belly rise completes under the interactive ramp, sit-from-142mm-plant no tip) with det mode retention at the parent's own baseline (rise 6/6, hold 6/6, lower >=4/6, no new falls). Prediction-if-false: session gates still fail with jitter trained in — the overfit is to something other than ramp timing (e.g. the start pose itself), pointing at start-state DR not profile DR; or retention breaks (jitter destabilizes the memorized rise), meaning dose down to 0.15 once, then stop.

**gate**: PASS if rl_move.sim.eval_session exit-code gates pass (interactive-ramp belly rise reaches target height; sit-from-142mm-plant with no tilt trip) AND det retention vs parent baseline: rise 6/6, hold 6/6 valid_plant, lower >=4/6 with no NEW outrigger pattern, zero falls, roll_tail/drag no worse than parent's own numbers. FAIL if any session hard gate still fails or retention regresses. One dose-down retry (0.15) allowed on a retention-only miss; session-gate miss = axis closed, look at start-state instead.

