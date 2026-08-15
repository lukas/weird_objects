# cw-recover-any4-b0scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T21:56:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**wandb_id**: brjnwcnb

**hypothesis**: Teach the robot from a blank slate to catch itself and get back to a stand, starting from the tiniest disturbance (a foot planted 2 degrees wrong) and only earning harder falls as it masters each level; this arm tests whether the new zero-indexed easy-first recovery ladder (B0 plant_catch -> B1 onefoot_micro -> B2 onefoot_mid -> B3 onefoot -> B4 tripod park -> B5 crouch/partial/bank -> B6 zero/tangle -> B7 flip, promote at EMA>=0.8 n>=4, retreat+re-certify at <0.2 n>=6, no harder probes) lets recovery learning start where warm-started attempts (cw-recover-any1 zero success through 13.5M, cw-recover-any2b) did not. OPERATOR-ORDERED launch (fb_20260815T214555_008f42, exact code SHA c60c7ac): genuinely FROM SCRATCH -- no --init-from, no --obs-pad-transplant, no checkpoint load of any kind; parent/provenance scratch; cw-recover-any2b is comparison evidence only. Otherwise clones any2b MDP/PPO exactly (recover-only mix, 40M, 512x128, batch 8192, gamma .995, lambda .98, DR .1, 16s episodes, ent .003, no-canary, eval 1M, video 2M, seed 11, roll/pitch 185, walk_obs_body_vel=2, rise ref, recover bank, BC anchor coef/recover=1 with foot_z=1/3mm/min_h_ahead 15mm/lookahead .5s, admit_n=4, retreat_n=6, ema beta .25, warp, 24 workers). Prediction-if-true: SCORE/recover_bucket_0_success rises, frontier promotes past B0 legitimately with per-bucket denominators visible. Prediction-if-false: B0 stays flat with valid resets and nonzero BC fill, pointing at reward/anchor coverage rather than curriculum granularity.

**gate**: B0-first gate: forced SCORE/recover_bucket_0_success must RISE from scratch; frontier starts and stays at bucket 0 (env/recover_start_bucket=0, env/recover_frontier_bucket=0, recover_active_families=1) until legitimate promotion at EMA>=0.8 with n>=4; STOP EARLY if reset telemetry shows invalid settled starts or BC eligibility/fill stays zero. Full-arm PASS bar: high held-recovery success on the earned mixture by 40M, VIDEO-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park), with per-bucket B0-B7 denominators reported.

