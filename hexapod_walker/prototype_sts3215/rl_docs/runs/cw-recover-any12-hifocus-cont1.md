# cw-recover-any12-hifocus-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-16T16:15:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-recover-any11-rsi-scratch1

**wandb_id**: 93tlavwp

**hypothesis**: Teach the fallen robot to untangle its own crossed legs more reliably, by dedicating far more training time to that specific case once the robot is already good at everything easier. any7 (bigger cert sample, more time) and any11 (RSI + the current default spaced-replay mix, ~50% mass on the frontier) both independently plateaued at the SAME tangle cert-fraction band (0.25-0.44) despite different mechanisms -- neither has ever tried genuinely MAXIMIZING frontier concentration beyond that default ~50% share. This arm warm-starts from any11 (zero-safe, already sitting at the B15 tangle/bank wall) and pushes recover_focus_mix 0.50->0.80 (recent 0.25->0.10, weak 0.15->0.05, uniform 0.10->0.05 -- coupled renormalization, RSI kept ON for zero-safety). Prediction-if-true: tangle CERT fraction breaks clearly above the 0.44 ceiling (>=0.7 sustained across >=2 late certs) within the 20M budget. Prediction-if-false: tangle plateaus again in the same ~0.25-0.44 band even at near-maximal concentration -- a THIRD miss that definitively closes curriculum-weight as a lever for tangle (any7, any11, any12 all independently capped at the same band) and names the next lever as a genuinely new mechanism: a tangle-specific on-path RSI bank harvested from successful tangle-recovery rollouts (generalizing the belly->plant RSI trick beyond its current hardcoded rise-path reference), which is CODE, not yet built.

**gate**: Read at 20M or earlier plateau: tangle CERT success_fraction (kind-level, 16-ep denominator) must sustain >=0.7 across >=2 consecutive late certs to call the lever a win. FAIL (plateaus in the pre-existing 0.25-0.44 band, or below) = curriculum-weight for tangle is CLOSED (3rd independent miss: any7, any11, any12); write the harvested tangle-RSI mechanism next, do not resample a 4th weight combination. Zero-bucket retention (RSI-protected) must stay >=0.8 at the final cert -- a regression there would mean high focus-mass starves the RSI replay itself. video-verify tangle successes (no flag/stilt/park) before crediting any improvement.

