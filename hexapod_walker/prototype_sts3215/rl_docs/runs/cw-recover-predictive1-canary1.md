# cw-recover-predictive1-canary1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-18T16:17:51+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-recover-any21-pop3-s11

**wandb_id**: sp8pdpxi

**hypothesis**: Give the get-back-up policy a real sense of its own body at wake-up: a frozen pretrained dynamics transformer (the encoder that already passed its prediction gates) now feeds BOTH actor and critic as read-only context, the robot sees its joint angles relative to the known good stance, and the first moment of each episode records 15 real sensor ticks instead of one repeated frame. This canary ONLY proves the new mechanism boots and optimizes on GPU physics (operator order fb 20260818T161001Z, code at d37fee09); behavior is judged later by the cohort, never here. From scratch, no --init-from, no --recover-init-curriculum; single run, no population machinery.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY PASS - all of: (1) obs space is 1440 = 90 fields x 16 frames; (2) reset info reports reset_history_probe_ticks=15 real ticks; (3) encoder md5 9df48f687967c25085ee50171e4110ff verified at load; (4) boot log prints 'frozen actor+critic snapshot'; (5) W&B actor/predictive_enabled=1 with finite actor/predictive_gate and residual metrics; (6) train/bc_anchor_fill > 0 (recover BC mentor buffer filling); (7) PPO global steps advance across updates; (8) --require-gpu-physics accepted = Warp physics live. Mechanism health only; any visible immature behavior is an observation, not a verdict.

**verdict**: CANARY PASS - every mechanism proof met: obs 1440 (90 fields x 16 frames) verified in-env; reset probe recorded 15 real, all-distinct sensor ticks; encoder md5 9df48f68... verified at load; boot log printed "frozen actor+critic snapshot"; actor/predictive_enabled=1 with finite gate (0.018) and residual/adapter means; recover BC buffer filled to cap 131072; PPO ran full 2M (907s) with cert machinery live (B0 plant_catch 16/16 -> promotion checkpoint B1 at 1,048,576) on VERIFIED Warp GPU physics; new 23-rung ladder (B14/B15 repair, B16-B20 tangle 60-100, B21 bank, B22 flip) active. Final reel: 4/4 recover_success on plant_catch. Mechanism health only - no behavior judgment; licenses the predictive1 from-scratch 3-seed cohort.

