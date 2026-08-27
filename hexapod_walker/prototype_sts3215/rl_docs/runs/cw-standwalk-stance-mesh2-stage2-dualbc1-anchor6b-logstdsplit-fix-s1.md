# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T06:55:42+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit-s1

**wandb_id**: ecbymjjc

**hypothesis**: Plain sentence: seed1 twin of anchor6b-logstdsplit-fix — the per-core exploration-noise split relaunched with the knob actually wired (the anchor6 pair never built log_std_b; launch-path bug fixed in commit 4fe10154). Same single mechanism, distinct seed, warm start from anchor2-s1's own checkpoint, for the 2-seed joint call the gate requires. Prediction-if-true/false: see anchor6b-logstdsplit-fix (joint gate).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY, seed1 half of the JOINT 2-seed call — same gate text as anchor6b-logstdsplit-fix: wiring check first (retrofit log line + log_std_b tensor present and != log_std in the checkpoint), then FULL PASS = WALK-SURVIVES (det gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=~0.2) AND HOLD-HELPS-FULL (hold/sto DR-0 term <=2/6) on BOTH seeds; FAIL-with-wiring-green on either seed = the genuine refutation of the exploration-noise theory -> dig-in moves to shared critic/trunk.

**verdict**: CANARY FAIL - MECHANISM: seed1 half of the anchor6b-logstdsplit-fix joint 2-seed call (see that run's verdict for full evidence). Wiring check green (log_std_b tensor annealed to -4.0/std 0.018, distinct from walk's untouched log_std ~-1.49/std 0.225 -- checkpoint-verified). Despite correct wiring, walk shows the anchor4-class catastrophe: det gait_valid 0/6, 3-5 legs sacrificed every episode (sac=[0,1,5] or worse), prog med 0.01, video-confirmed static splayed-leg freeze (walk_det_2_sheet.png) -- essentially no walking. Twin seed0 (fix, no -s1) passed cleanly (WALK-SURVIVES + HOLD-HELPS-FULL). Per the pre-registered gate, FAIL-with-wiring-green on either seed is the decisive result -> exploration-noise theory for the anchor4-class collapse is REFUTED; the per-mode log_std split is not the fix. Dig-in moves to shared critic/trunk; no further log_std-split arms.

