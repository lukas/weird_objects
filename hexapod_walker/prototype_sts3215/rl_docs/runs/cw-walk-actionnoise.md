# cw-walk-actionnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:56:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 0lmpzf4t

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13c untested axis: dr.action_noise (Gaussian noise added directly to the commanded joint target before the safety layer, distinct from encoder observation noise already tested) at 4x default (0.02 -> 0.08 rad std, ~4.6deg). Plain: does noisy actuator command delivery (servo bus jitter / quantization on the WRITE side) perturb the gait? ISOLATED axis, dr-scale 0.0, only dr.action_noise changed. Prediction-if-true: gait holds (own-cfg gv 12/12, 0 term, det median fwd >=1.2m) and DR0 retention holds - closes as another free axis. Prediction-if-false: noisy commands destabilize footfall timing/current. Strongest alternative: champion already tolerates this free at this magnitude - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting (c59 rule).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.action_noise=0.08, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT (if-false confirmed). Actuator command-noise (dr.action_noise=0.08, 4x default) letter-passes own-cfg (det gv 6/6, 0 term, det med fwd 1.56m>=1.2 gate) but champion measured under the IDENTICAL spread matches it draw-for-draw, including the same worst-case stochastic crater (sto/3: prog 0.65/slip 2.46/fwd 1.23m here vs prog 0.60/slip 2.55/fwd 1.17m for champion) -- training on this noise changed nothing because the champion already tolerates it free. Frames: clean six-leg tripod cycling, no flag legs, no lurching. This was the LAST untried single-axis DR/calibration field -- the 13b/13c exposure ladder is now CLOSED 12-for-12 NO-EFFECT (joins encodernoise, zerobias3, gyrobias3, gyronoise15, legmass25, stiffvar, gainvar, imubias3, contactstiff, linklen, cmddrop). Stop probing single-axis DR fields.

