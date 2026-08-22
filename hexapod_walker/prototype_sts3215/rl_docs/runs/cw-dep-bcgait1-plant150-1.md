# cw-dep-bcgait1-plant150-1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T04:03:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: ig0tnmh5

**hypothesis**: Re-harden the deployed walking champion on the corrected robot geometry: the sim's shin length was re-measured at 150mm (was 128mm) and the old champion now falls when driving backwards, so this arm fine-tunes the same policy on the corrected body so it walks reliably again. Mechanism: warm-start from ppo_goal_cw_dep_bcgait1_hard1's checkpoint, identical recipe/reward, only the plant is the measured tibia-150 tree; prediction-if-true the policy re-adapts its stance/gait within the budget and passes the tibia-150 session drive gate its parent fails; prediction-if-false falls persist (geometry shift too large for fine-tune), pointing at a fresh BC-INIT on the tibia-150 scripted teacher instead.

**gate**: PASS if at tibia-150: det eval_session drive segments zero falls INCLUDING reverse, fwd_heading soft PASS, gait_valid 6/6, slip/m <= 2.0, and visual stats (roll_tail_deg, drag_m) not worse than the parent's 128mm baseline. Control = parent's tibia-150 session (back-fall + fwd yaw -21.8). FAIL if any drive-segment fall remains at 10M or gait degrades to <6/6 legs cycling.

