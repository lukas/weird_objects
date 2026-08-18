# cw-dynrep-criticD-walkcurr3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T08:34:15+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**git_sha**: f759b6ba260cd6d39814c4ea0809436d20cc6c2d

**wandb_id**: 4dih5ztm

**hardware_ready**: False

**hypothesis**: Teach the walking robot joystick commands with a coach-style curriculum - master real walking speed first, then wider speeds, angled headings, smooth direction changes, stop-and-go, rougher physics - promoting only on deterministic held-out proof and rolling back if old rungs decay; this arm tests whether that whole recipe works when the PHYSICS simulation itself runs batched on the GPU (4096 worlds on Warp/MJX) instead of 16 CPU processes, which is the operator-required compliant backend and ~100x more experience per wall-clock hour. Everything semantic is the walkcurr2 contract unchanged: fresh scratch actor seed 8, exact frozen vt2ovznc dynamics-transformer critic D (md5-pinned), V2 ignition buckets, calibrated cert gates, update-path health (3 epochs, target_kl 0.01, actor 1e-4->1e-5 / critic 3e-4 groups, KL rollback), terminal charge 30, 40M steps. Operator order 2026-08-18 fb_20260818T065930_03b422.

**gate**: Pre-registered 40M decision checkpoint, no extension without a verdict. Judged on the BEST checkpoint = last retention-clean promotion (never reward/latest). PASS = (a) frontier reaches >= B6 (stop/restart mastered with retention) by 40M, AND (b) on the shared 6-episode command-rich own-DR walk eval the promotion checkpoint meets the 40m1 bars (early_term_rate 0, cmd_prog_frac >= 0.6, slip_per_m <= 2.0, peak_roll_deg <= 8, contact_sw_per_s >= 3, SCORE/loco_quality >= 10), AND (c) beats cw-dynrep-criticD-40m1's best-loco checkpoint on loco_quality at matched or better slip/roll. Mechanical invariants: '[backend] training physics VERIFIED: MjxShardedVecEnv impl=warp' in the log (SubprocVecEnv fail-closed), encoder md5 match, pred/snapshot_version stays 0, per-bucket walkcurr/* panels advancing, promotions saved, no locked-bucket training, kl_rollback counters logged. Verdict must quote the per-bucket cert table + visual-quality stats vs 40m1 AND state the backend proof line.

**verdict**: FAIL — full 40M budget spent, zero curriculum progress: walkcurr/frontier and walkcurr/promotions stayed at 0 for the entire run (never certified even bucket B0, gate a requires >=B6), because the calibrated walk_height_gate/walk_kernel_prog_gate income gates were left OFF and the actor update (1e-4 x 3 epochs) was far below the proven acquisition recipe — exactly the operator's own pre-registered failure mode (fb_20260818T085648_2a0a60, measured already at 7.5M). Gate-eval confirms behaviorally: DR-0 det gait_valid 0/6, ALL 6 episodes end in a tilt_pitch fall from a low crouch (slip_per_m med 2.61 > 2.0 bar, prog med 0.90 < prog_frac>=0.5 bar once falls counted), sto 0/6 with one episode going backward (prog -0.10); own-DR(0.3) det is gait_valid 6/6 by the harness's shape check but ALSO falls in all 6 episodes (tilt_roll/tilt_pitch). Backend proof line present ([backend] training physics VERIFIED: MjxShardedVecEnv impl=warp n_envs=4096); mechanism is healthy, the RECIPE is not. Superseded in-flight by the operator's same-seed canary tournament (cw-dynrep-criticD-walkcurr4-canA-r2 RUNNING per fb_20260818T085648_2a0a60 + URGENT addendum fb_20260818T085834_588d9a correcting arms B/C to actor-init from the proven BC-gait recipe) — no new follow-up queued here, that tournament already supersedes this line.

