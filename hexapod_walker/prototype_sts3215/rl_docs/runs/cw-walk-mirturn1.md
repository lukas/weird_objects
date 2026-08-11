# cw-walk-mirturn1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T19:36:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: akig0wvx

**hardware_ready**: no

**hypothesis**: Can the walking robot finally learn to TURN when the joystick says turn? Every reward-tuning attempt failed because the network itself has a built-in left-handed bias; this arm adds a mirror-symmetry loss during training (the policy is penalized whenever its answer to a mirrored world is not the mirrored answer), riding the newly bank-verified turn pricing that no longer pays a frozen or drifting body. Warm from THE hardware walk champion (cw-dep-vref1-r1, pad-transplant), forward wedge + yaw command set, all three 08-11 pricing fixes ON (walk_kernel_yaw_gate, walk_yaw_hold_prog_gate, yaw_still_avg_s) plus train.mirror_loss_coef=1.0 — the pre-registered step 3 of the TURN plan (rl_docs/TURN.md). Prediction-if-true: eval_yaw turn-segment |wz_err| median drops toward <=0.10 in BOTH turn directions (parent/turnfix1 flat ~0.236 command-invariant drift) with walking staying healthy. Prediction-if-false: gait stays healthy and the symmetry loss converges low, yet yaw output remains command-invariant — then mirror-symmetry TRAINING closes for good and the shipped turning story is eval-time MirrorPolicy chirality selection (~2 deg/s steering). Strongest alternative: the mirror loss fights the warm-started asymmetric gait and degrades walking itself — visible as slip/gait_valid regression vs the matched frozen-parent control.

**gate**: 2M ckpt, eval_yaw both signs + matched frozen-parent control (same seeds/cfg): PASS if turn-segment |wz_err| median <= 0.15 in BOTH signs (vs parent's command-invariant ~0.236) AND hold-segment |wz| median <= 0.06 AND zero falls AND forward walk stays in the parent band (gait_valid 6/6, slip/m <= 2x parent) on det video review. FAIL if either sign stays at the structural drift (command-invariant wz) OR walking degrades below the parent band — mirror-symmetry training then CLOSES (two-miss rule already spent on reward shape) and MirrorPolicy selection ships as the turning story.

**verdict**: FAIL on both halves of the pre-registered gate; the 'strongest alternative' fired. Yaw tracking did NOT arrive: eval_yaw turn |wz_err| med 0.254 (gate <=0.15; yawgate2/turnfix1 era was 0.236) with LEFT-RIGHT asymmetry intact (tip-left 0.409 vs tip-right 0.255) and heading-hold drift WORSE than the parent (hold |wz| med 0.148 vs vref1-r1's measured 0.04). And walking DEGRADED below the parent band: DR0+own-DR0.5 harness prog med 0.41-0.43 (parent ~1.0), slip/m med 6.0-7.6 (parent 1.1-1.5), video shows near-in-place churning; zero falls. train/mirror_sym_loss engaged and converged 28.3->0.51 — the warm-started champion is massively asymmetric (initial loss 28) and coef 1.0 symmetry pressure rewrote the gait instead of adding turn authority. Per the pre-registered FAIL branch, mirror-symmetry TRAINING on a warm champion CLOSES; the shipped turning story is eval-time MirrorPolicy chirality selection (arc-left/arc-right/straight, zero training, probe-proven ~2 deg/s). Step 4 (BC anchor on turn ticks) stays in reserve, unpromising after transbc1's walk-tick freeze. Do not warm from this checkpoint.

