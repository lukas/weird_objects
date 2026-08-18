# cw-dep-bcgait1-hard1-steer1c

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T15:48:48+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: bpz63iwd

**hypothesis**: Teach the strong tall walker to survive abrupt joystick direction changes without tangling its legs: this 2M canary checks that the long-episode multi-command training recipe (120 s episodes, all six command-schedule families, INSTANT no-blend switches, irregular 2-20 s dwells) boots and trains healthily on the hard1 champion without erasing its tall tripod gait. Ordered by operator fb_20260818T152717_278879. The rot60 on/off probe (probe_dirswitch_tangle, 08-18) showed sector crossings are NOT the tangle trigger — rot60 ON is strictly safer on every proxy — so plain transition exposure is the smallest correct fix; the gaps to train away are yaw-limit saturation after switches (margins pressed past the hard limit) and legs that stop cycling for seconds after a command change. Prediction-if-true: finite losses, KL rollback machinery quiet, periodic eval keeps tall height and zero-fall while episodes now contain many abrupt switches. Prediction-if-false: 120 s episodes + conservative actor LR destabilize PPO (KL rollbacks firing constantly / value loss diverging) or the gait visibly collapses within 2M — meaning the recipe needs staged dwell curriculum instead of the full mix.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): PASS if the run reaches 2M with finite losses, train/approx_kl bounded (kl-rollback fires <20% of updates), episodes average >60 s (no mass early termination), and the last periodic eval shows the tall gait not collapsed (no fall explosion, height not re-crouched). FAIL on boot failure, KL/value divergence, or immediate catastrophic forgetting of the tall gait. NO mature-behavior verdict at 2M; the larger hardening continuation launches only after this passes.

