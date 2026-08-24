# cw-walkcurr-pf-fwd6-rscale50-sde-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T03:16:00+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50-sde

**wandb_id**: oqp7upaw

**hypothesis**: Plain English: seed replicate of the sde breakthrough before building a lineage on it -- the parent produced the track's first-ever forward-progress states (sto prog 0.32-0.47) and the first belly-sit escape WITHOUT the action-bias fix (height 10-18mm all run), and both claims currently rest on a single seed. Exact parent recipe, seed 2, fresh 2M. Prediction-if-true (mechanism, not luck): level stance + fall-dominated forward excursions reproduce (height <30mm all run, tilt-term-dominated training, nonzero sto prog). Prediction-if-false (belly-sit or frozen splay recurs): the parent's signature was seed-contingent and the sde-actbias combos must be read with that caveat -- any success there would credit actbias, not sde.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Replication read: env/height_err_mm band (<30mm = replicated escape), sto prog > 0.2, tilt-term-dominated training.

**verdict**: Result: seed-2 replicate of the rscale50-sde arm FAILS the gate, and the replication read is decisive — the parent's gSDE lurch MECHANISM replicates but its forward DIRECTION does not, demoting the parent's 'first-ever forward-progress states' from mechanism to seed luck. Evidence: det gate 0/6 gait_valid, prog med 0.00, fwd 0.01m, slip/m 13.8 (in-place micro-slip), identical static splayed-crouch strip (sac=[0,4] all 6 eps); sto gate 0/6 with prog med -0.46 — real displacement excursions (fwd med 0.07m) but BACKWARD, all 6 eps terminating tilt_pitch/tilt_roll, strip shows the robot lurching and tipping over backward. Pre-registered replication reads: env/height_err_mm 6.9-22.5mm all run (<30mm band — belly-sit escape REPLICATED without actbias), tilt-term-dominated training with ep_len flat ~60-73 all 2M (parent: 65-70) REPLICATED, healthy clip_fraction 0.13-0.16 all run REPLICATED, nonzero sto excursions REPLICATED — but sto prog is -0.46 vs parent's +0.32: the single per-rollout correlated noise draw biases the action mean in a SEED-RANDOM direction, and the mean policy never learns to walk in either seed (det static both times). walk_freeprog_score deteriorates -0.149->-0.214 (parent -0.164->-0.194, same worsening band), reward falls every quarter (17.9->12.4). Aligned FAIL per 08-21 (healthy optimizer, adequate budget, eval bad and deteriorating). Why it matters: (1) confirms the sde lineage is closed — no further sde-only variants; (2) sharpens the caveat for the concurrent sde-actbias combos (other cycle): any directional success there credits actbias, not sde; (3) leaves the track's pinned fork unchanged — the parkstart-p50 read and the BC-kickstart operator question (q_20260824T0233Z) remain the live decision. Evidence: logs/ckpt_eval/cw_walkcurr_pf_fwd6_rscale50_sde_s2_gate/, W&B oqp7upaw.

