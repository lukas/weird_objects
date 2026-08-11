# cw-dep-hgt1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T03:00:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: hm99c57g

**hardware_ready**: False

**hypothesis**: DISCOVERY (2M, warm from cw-dep-tip1, the hardware-validated walk arm 08-10: 3 clean walks / 1 runaway): height-keeping income gate (reward.walk_height_gate, landed 08-10 with an MDP_PREFLIGHT bank) removes the commanded sag. Hardware finding (HARDWARE.md 08-10): every deployed walk policy migrates to a crouch 54-70 mm below the spawn stance; knees track commands within 1-3 deg so the crouch is COMMANDED posture; base k_height charge (~0.36/tick at 60 mm) is outbid by walk income (~3/tick). The gate multiplies velocity income by a Gaussian on body height vs the episode anchor (sigma 30 mm): upright gait keeps 0.99 of income, the measured -51 mm crouch keeps 0.13 (probe 08-10).

**gate**: PASS if env/walk_height_factor reaches >= 0.8 rollout mean by end of run (deployed-champion posture measures ~0.1-0.2) AND walk retention holds: matched-parent eval_checkpoint under identical config/seed, progress_ratio 0.75-1.25, walk_vel_err within 15% of the frozen parent, SCORE/tipped_recovery_success and SCORE/roll_trap_pass not worse than parent. Behavioral check on video: walking at spawn height, not stilting or freezing. FAIL if height factor stays < 0.5 or walk breaks. PASS -> hardening 18M with this run as --evidence, then export to the robot picker for a bench A/B against dep_tip1.

**verdict**: FAIL -- height-keeping income gate did not fix the commanded sag: env/walk_height_factor started near the parents upright 0.87 but collapsed to a flat 0.24-0.26 within ~50 updates and stayed there the whole 2M steps (gate needs >=0.8, misses even the <0.5 fail floor); harness confirms ~50-77mm crouch persists at eval. Not a new exploit -- walk itself is still fine (gait_valid 5-6/6, progress_ratio ~1.0, no flag-leg/freeze) -- the income discount (already at max gate=1.0) just does not outbid whatever makes the crouch cheaper. Cosmetic issue per HARDWARE.md (sag is commanded, tracked, not a hardware fault); not a blocker.

