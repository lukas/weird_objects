# cw-getup3-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T02:51:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: cw-getup3

**wandb_id**: r6tvbq5t

**hardware_ready**: False

**hypothesis**: Give the first WORKING getup recipe the steps it was still climbing at the buzzer. cw-getup3 (BC-anchored, warm-started unified recover-stand-walk) ended its 2M discovery budget mid-climb: stand quality env/getup_S 0.09->0.17 monotone, returns positive and rising, height factor 0.31->0.62. Continue from its checkpoint with the IDENTICAL recipe (zero variables changed) and a 10M budget. Prediction-if-true: S keeps climbing toward 0.3+, reward_getup_hold becomes a visible income line, and video shows sustained supported stands from floor-adjacent starts. Prediction-if-false: S plateaus below 0.2 — the BC anchor can drag the policy near the stand but the S-gated income cannot close the last gap; next lever is stand-income depth (sibling cw-getup4 tests exactly that in parallel). Strongest alternative: the S rise entrenches as a partial tripod pose the fades under-price — visible as feet_loaded stuck ~2.4 while f_height keeps climbing.

**gate**: PASS if env/getup_S ends >0.30 and non-declining, mean reward_getup_hold >0.05, and video shows at least one sustained (>3 s) supported stand from a floor-adjacent start with no flag-leg/stilt exploit; FAIL if S plateaus <0.2 by 6M or the video shows static collapse or an exploit dominating.

**verdict**: Crashed at launch, 0 training steps (~1s runtime, exit before first rollout): the respec inherited a stale --obs-pad-transplant=4 flag from the getup2-r1 obs-widening fix, but this continuation warm-starts from cw-getup3 itself (same joint_walk task, obs already 72-wide, widened-by-0) so the transplant's own safety check raised SystemExit("--obs-pad-transplant 4 but obs widened by 0 (72 -> 72)"). Mechanical launch bug, not a science result -- no verdict on the BC-anchor-getup recipe itself. Relaunched corrected as cw-getup3-c2 (--obs-pad-transplant=0, otherwise identical), confirmed training cleanly (1.38M steps, bc_anchor_loss active, fps ~9.5k) on hexapod-mjx-train-2.

**failed_reason**: W&B global_step not advancing (0 -> 0) after 120s (n_steps=64, cpu-time flat for 2 polls)

