# cw-recover-any2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STOPPED

**created**: 2026-08-15T19:55:44+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: lf5afhd6

**hardware_ready**: False

**hypothesis**: Teach the robot to get up from the two easiest disturbed starts first -- one foot parked wrong (onefoot) and the parked crouch (park) -- and only admit harder falls once those are mastered; this arm tests whether the bucket-1-first recovery curriculum plus restored foot-height BC-anchor supervision (operator-implemented fix, main commit aa1023c: curriculum starts at bucket 1 only with re-certifying retreat, per-kind train/eval telemetry, named recover_success termination, height-conditioned anchor matching with min-height-ahead pursuit) lets recovery learning actually start, where cw-recover-any1 sat at zero success through 13.5M with declining stand quality. OPERATOR-ORDERED (fb_20260815T194955_9441a0): warm from cw-stand-footlow2-hard1 (never any1's degraded checkpoint), any1's MDP/PPO settings otherwise unchanged (512x128, gamma .995, lambda .98, DR .1, 16s episodes, ent .003, safety envelope, rise ref, recover bank, BC coef/recover gate, obs-pad transplant, 40M). Prediction-if-true: forced onefoot AND park success curves rise, per-kind EMAs cross 0.8, curriculum admits bucket 2 legitimately. Prediction-if-false: a kind stays flat with valid resets and nonzero BC fill -- pointing at reward/anchor coverage, not curriculum order.

**gate**: Bucket-1 gate: both forced onefoot and park eval success curves must RISE; no curriculum promotion before both per-kind EMAs >=0.8 with count >=4; STOP EARLY if reset telemetry (post-settle height/tilt/min-load/pad-spread) proves either settled start invalid, or if BC eligibility/fill stays zero. Full-arm PASS keeps any1's bar: >=95 pct det / >=85 pct sto held recovery on the active mixture by 40M, VIDEO-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park), no rise/hold/lower regression vs cw-stand-footlow2-hard1.

**verdict**: OPERATOR-ORDERED STOP at 5.1M/40M (fb_20260815T201417_5f7f0e): mechanically healthy (fps ~4800, BC fill 131072, footz loss ~0.037) but overall/onefoot/park success all ZERO, bucket-1 EMAs collapsed to ~0.02, every episode a full 400-step failure, no split SCORE/recover eval metrics emitted. PRESERVED as the warm-start diagnostic: with any1 it shows two warm-started (stand-champion init) recovery attempts both flatlined at zero -- cw-recover-any3-scratch1 tests scratch init as the isolated change.

