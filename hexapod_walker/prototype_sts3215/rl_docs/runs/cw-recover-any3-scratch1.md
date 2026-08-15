# cw-recover-any3-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T20:26:00+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: scratch (null init; respec comparison: cw-recover-any2)

**hypothesis**: Teach the robot to get up from the two easiest disturbed starts -- one foot parked wrong (onefoot) and the parked crouch (park) -- training FROM SCRATCH, with no warm-start checkpoint of any kind; this arm tests whether the stand-champion initialization itself is what blocks recovery learning, since both warm-started attempts (cw-recover-any1, cw-recover-any2) flatlined at exactly zero success while mechanically healthy (any2: 5.1M steps, overall/onefoot/park all 0, bucket-1 EMAs ~0.02, every episode a full 400-step failure). OPERATOR-ORDERED (fb_20260815T201417_5f7f0e, execute-not-analyze): clone any2's full recovery MDP/PPO recipe and 40M budget EXACTLY (512x128, batch 8192, gamma .995, lambda .98, DR .1, 16s, ent .003, rise ref, recover bank, BC anchor coef/recover/foot-z with foot_z_mm=3, min_h_ahead_mm=15, lookahead .5s, admit 4 / retreat 6 / ema .25, safety 185/185, no-canary, seed 11, warp/24) on current main containing recovery fix aa1023c; the ONLY training change is scratch initialization -- REMOVED --init-from and REMOVED --obs-pad-transplant. Parent/provenance: SCRATCH/NULL; cw-recover-any2 is the respec comparison only, never an init source. Prediction-if-true: onefoot/park success curves rise where both warm-started twins stayed at zero, implicating the champion weights as a local-minimum trap. Prediction-if-false: zero success from scratch too -- pointing at the recovery MDP/reward/reset validity, not initialization.

**gate**: Bucket-1 gate vs the preserved any2 diagnostic: forced onefoot and park eval success curves must RISE above any2's flat zero (per-kind EMAs were ~0.02 at 5M); no curriculum promotion before both per-kind EMAs >=0.8 with count >=4; STOP EARLY if reset telemetry (post-settle height/tilt/min-load/pad-spread) proves either settled start invalid, or if BC eligibility/fill collapses to zero, or a pre-registered banked cheat class appears in video. Full-arm PASS keeps the line's bar: >=95 pct det / >=85 pct sto held recovery on the active mixture by 40M, VIDEO-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park). Research specialist -- never auto-replaces the product baseline.

**refused_reason**: hexapod-mjx-train-1 already runs cw-recover-any2b — GPU pods host exactly one run; pick a free GPU pod.

