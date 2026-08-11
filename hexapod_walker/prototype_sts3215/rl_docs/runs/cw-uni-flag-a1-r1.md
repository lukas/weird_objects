# cw-uni-flag-a1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T15:44:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-uni-flag-a1

**wandb_id**: rpvt0xjp

**hardware_ready**: no

**hypothesis**: Can ONE from-scratch brain with an explicit 'which skill am I being asked for' input learn to hold still, stand up from a crouch, and sit down -- stage A of the operator's single-checkpoint joystick policy (RL_PLAN FLAGSHIP). Mechanical retry of cw-uni-flag-a1, which crashed at 0 steps (SIGBUS, /dev/shm too small for hist16+mode_onehot obs width at n-envs=4096, gotcha 13c) -- not a science result. Same spec, one infra fix: --n-envs 3072 (documented workaround, fits within the 64M pod shm). Hypothesis, predictions and gate UNCHANGED from cw-uni-flag-a1: hist16 + 256x256 + obs.mode_onehot=1 riding the proven stand-specialist reward stack incl. the twice-validated BC anchor; near-plant only (100% crouch rise starts), no walking yet. Prediction-if-true: hold/rise/lower coexist at near-specialist honesty by 2M. Prediction-if-false: the known feet-factor collapse/flag-leg cheat reappears despite anchor + mode input = multitask interference, fork to MoE per the flagship pre-registration. Strongest alternative: the BC anchor alone carries the skills and the mode bit is dead weight -- settled later by a mode-scrambled ablation, not this run.

**gate**: 2M det harness, own stack: hold valid_plant >= 4/6 AND rise (all-crouch starts) valid_plant >= 3/6 AND lower posture-strict >= 3/6, ZERO known-exploit fingerprints (flag-leg/tripod/park/freeze) in gated-mode video.

**verdict**: GATE FAIL (rise leg only) but MECHANISM SUPPORTED — budget-limited from scratch, NOT interference, NOT the cheat. OBSERVATIONS: hold det 6/6 valid_plant (motionless level stand, video), lower det 6/6 posture-strict (honest sit, video); rise all-crouch valid_plant 1/6 det + 1/6 sto vs >=3/6. Rise failure anatomy (12 eps): 2 full valid_plant (video: genuine crouch->level six-foot stand); 3 full-height six-foot stands missing ONLY current-tail (p95A 1.6-1.9A)/footprint-by-mm; 3 honest stalls 22-100mm short, feet down, posture OK; 2 tilt_roll/over_current falls mid-rise. The one 91mm-elevated-leg frame (det/5) is MID-FALL under a tilt_roll termination, not a stable height-cheat stand — no episode holds height with a leg aloft. Training: rise_feet_factor 0.95->0.33@0.4M->0.62@2M STILL RISING (historical cheat = 0.87->0.17 and dead), rise_plant_factor recovering 0.15->0.31, reward quarters -4/-17/+10/+41 (steepest at cutoff), bc_anchor_loss 0.20->0.011 (specialist-healthy), hold/lower factors stable-good all run — no skill seesaw. INTERPRETATION: prediction-if-false (feet-factor collapse/flag-leg cheat reappears) REFUTED; interference NOT SUPPORTED (hold+lower at specialist-grade honesty from scratch while rise still climbing); every rise miss is the same current/footprint/height finishing tail that 10M hardening resolved on the specialist lineage (bc1->bc1-hard1), and from-scratch-vs-warm-start explains the 2M gap. VERDICT: no MoE fork; hardening continuation cw-uni-flag-a1-h1 (10M, warm from this ckpt) queued with the MoE fork pre-registered as its prediction-if-false.

