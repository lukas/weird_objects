# cw-dynrep-criticD-walkcurr3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: SUPERSEDED_REWARD_EXPLOIT

**created**: 2026-08-18T08:34:15+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**git_sha**: f759b6ba260cd6d39814c4ea0809436d20cc6c2d

**wandb_id**: 4dih5ztm

**hardware_ready**: no

**hypothesis**: Teach the walking robot joystick commands with a coach-style curriculum - master real walking speed first, then wider speeds, angled headings, smooth direction changes, stop-and-go, rougher physics - promoting only on deterministic held-out proof and rolling back if old rungs decay; this arm tests whether that whole recipe works when the PHYSICS simulation itself runs batched on the GPU (4096 worlds on Warp/MJX) instead of 16 CPU processes, which is the operator-required compliant backend and ~100x more experience per wall-clock hour. Everything semantic is the walkcurr2 contract unchanged: fresh scratch actor seed 8, exact frozen vt2ovznc dynamics-transformer critic D (md5-pinned), V2 ignition buckets, calibrated cert gates, update-path health (3 epochs, target_kl 0.01, actor 1e-4->1e-5 / critic 3e-4 groups, KL rollback), terminal charge 30, 40M steps. Operator order 2026-08-18 fb_20260818T065930_03b422.

**gate**: Pre-registered 40M decision checkpoint, no extension without a verdict. Judged on the BEST checkpoint = last retention-clean promotion (never reward/latest). PASS = (a) frontier reaches >= B6 (stop/restart mastered with retention) by 40M, AND (b) on the shared 6-episode command-rich own-DR walk eval the promotion checkpoint meets the 40m1 bars (early_term_rate 0, cmd_prog_frac >= 0.6, slip_per_m <= 2.0, peak_roll_deg <= 8, contact_sw_per_s >= 3, SCORE/loco_quality >= 10), AND (c) beats cw-dynrep-criticD-40m1's best-loco checkpoint on loco_quality at matched or better slip/roll. Mechanical invariants: '[backend] training physics VERIFIED: MjxShardedVecEnv impl=warp' in the log (SubprocVecEnv fail-closed), encoder md5 match, pred/snapshot_version stays 0, per-bucket walkcurr/* panels advancing, promotions saved, no locked-bucket training, kl_rollback counters logged. Verdict must quote the per-bucket cert table + visual-quality stats vs 40m1 AND state the backend proof line.

**verdict**: FAIL pre-registered 40M gate + SUPERSEDED_REWARD_EXPLOIT (operator order fb_20260818T085648_2a0a60): 0 promotions in 80 cert rounds; final B0 prog 0.90 but no_falls+six_leg_gait+roll(11deg) fail - a falling, lurching paddle, and the operator-measured -65mm crouch was never priced because the calibrated walk_height_gate/walk_kernel_prog_gate were omitted from this arm. PPO stayed numerically healthy throughout (no KL rollbacks, EV .995+) - this is a reward-spec failure, not a backend/optimizer failure. Checkpoints + 40M final preserved on train-5 and W&B. Successor lineage: walkcurr4 tournament (scratch A/B/C all FAILED admission - gates work, ignition does not happen from scratch; operator addendum fb_20260818T085834_588d9a's gaitinit-bcinit/-hard1 corrective canaries running under the concurrent cycle).

