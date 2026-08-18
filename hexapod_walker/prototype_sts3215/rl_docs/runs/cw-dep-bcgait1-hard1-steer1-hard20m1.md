# cw-dep-bcgait1-hard1-steer1-hard20m1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED_BUG

**created**: 2026-08-18T16:32:01+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-dep-bcgait1-hard1-steer1c

**wandb_id**: w3fbxfj7

**hypothesis**: Teach the tall walker to survive abrupt joystick direction changes for real, not just survive a 2M mechanism check: continue the exact steer1c recipe (120s stress-mix episodes, all six command-schedule families, instant no-blend switches, irregular 2-20s dwells) to ~20M steps with a --best-ckpt retention guard so the run can never publish a checkpoint worse than its own best. Ordered lineage: operator fb_20260818T152717_278879's direction-switch fix, now the pre-registered follow-up named in hw/STATUS.md WAITING-ON. Prediction-if-true: at 20M the checkpoint retains the fixed-command hard1 quality (height >= -20mm, six-leg gait-valid, zero falls, slip <= 1.8/m) AND passes long det+sto direction-switch panels with zero falls/tangles, all legs cycling after every command change, no accumulating yaw-limit saturation, and prompt tracking recovery post-switch. Prediction-if-false: the longer stress-mix exposure either erodes the base fixed-command gait (retention loss) or the tangle/saturation symptoms persist despite the exposure, meaning the fix needs a staged-dwell curriculum instead of the full mix from step 0.

**gate**: HARDENING: --best-ckpt retention guard active throughout. PASS requires ALL: (1) fixed-command panel (dr-scale 0, no switches) matches or beats hard1: height >= -20mm, six-leg gait-valid every episode, zero falls, slip <= 1.8/m; (2) long det+sto direction-switch panels (probe_dirswitch_tangle-style, both DR-0 and own-DR) show zero falls/tangles; (3) all six legs resume cycling within a bounded number of ticks after every command change (no multi-second freeze); (4) no accumulating yaw-limit saturation across repeated switches; (5) command tracking recovers promptly post-switch (vx/vy/yaw error back in-band within the same bound as hard1's steady-state). FAIL on any bar; name it. Matched-parent (hard1) control required for the fixed-command panel.

**verdict**: STALE ledger row: this entry (train-5) never actually ran — train_ppo_mjx.py crashed at argparse before any step ('unrecognized arguments:', the trailing empty-string artifact from my earlier respec --arg='--best-ckpt' bug, root-caused/fixed in the sibling FAILED_BUG row for this same run name). The launcher's own retry/self-repair loop apparently re-verified and marked this row RUNNING at 16:41:10 despite the process being dead (mechanical verification gap — no live process, log unchanged since 16:32, W&B never actually started for this row) -- reconciled by hand since a duplicate real run (this same name, created 16:39:50, train-4, W&B w3fbxfj7) is genuinely RUNNING with the corrected args and is the canonical one. No GPU-seconds lost on train-5 beyond the instant argparse crash; train-5 is free.

