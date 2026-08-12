# cw-dep-tip1-push1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T03:19:24+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: 68cm76id

**hypothesis**: Teach the deployed walking champion to survive the roll TORQUE every real walk shows at takeoff, using the mechanism-corrected axis (dr.walk_push_*, an xfrc base torque pulse) instead of the command-side kick that turned out to structurally cap out below the hardware regime. One change vs tip1: dr.walk_push_prob=0.5 at the calibrated dose (peak 2.0-3.0 Nm over 0.8-1.5s, the dose that reproduces the measured 13-27deg/11-46deg-per-s hardware takeoff coin-flip when it lands on a tripod swing). Prediction-if-true: under a forced walk_push injection at the calibrated dose, this checkpoint's fall rate is measurably lower than a frozen tip1 under the identical injection (matched-parent separation, unlike every command-pulse-family arm before it). Prediction-if-false: zero separation again -- meaning even a true torque disturbance during training does not teach active takeoff leveling, and the next lever is contact/pinning geometry work itself (belly/tucked-leg collision modeling), not any more DR.

**gate**: PASS if under a forced dr.walk_push injection at the calibrated dose (prob=1.0, nm=2.6 fixed, s=1.5, det, n>=12 seeds) this checkpoint's terminal-fall rate is at least 2x lower than frozen tip1 under the IDENTICAL injection (matched-parent control) AND nominal DR0 walk retention matches tip1's own band (gait_valid, slip/m, prog_ratio within noise) with zero new falls. FAIL if zero separation from frozen tip1 (3rd command/torque-side no-separation arm -> the whole 'perturb during training' family closes in favor of contact/pinning modeling).

