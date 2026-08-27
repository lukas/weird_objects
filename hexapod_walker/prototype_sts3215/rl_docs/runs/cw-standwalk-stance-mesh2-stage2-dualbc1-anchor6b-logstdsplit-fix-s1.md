# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T06:55:42+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit-s1

**wandb_id**: ecbymjjc

**hypothesis**: Plain sentence: seed1 twin of anchor6b-logstdsplit-fix — the per-core exploration-noise split relaunched with the knob actually wired (the anchor6 pair never built log_std_b; launch-path bug fixed in commit 4fe10154). Same single mechanism, distinct seed, warm start from anchor2-s1's own checkpoint, for the 2-seed joint call the gate requires. Prediction-if-true/false: see anchor6b-logstdsplit-fix (joint gate).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY, seed1 half of the JOINT 2-seed call — same gate text as anchor6b-logstdsplit-fix: wiring check first (retrofit log line + log_std_b tensor present and != log_std in the checkpoint), then FULL PASS = WALK-SURVIVES (det gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=~0.2) AND HOLD-HELPS-FULL (hold/sto DR-0 term <=2/6) on BOTH seeds; FAIL-with-wiring-green on either seed = the genuine refutation of the exploration-noise theory -> dig-in moves to shared critic/trunk.

