# cw-uni-flag-a1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T15:46:53+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**wandb_id**: u5za5jof

**hypothesis**: Can ONE from-scratch brain with an explicit 'which skill am I being asked for' input learn to hold still, stand up from a crouch, and sit down -- stage A of the operator's single-checkpoint joystick policy (RL_PLAN FLAGSHIP, unblocked today: mode one-hot obs landed, all stance semantics banks pass, turn de-scoped). Config: hist16 + 256x256 + obs.mode_onehot=1 riding the proven stand-specialist reward stack incl. the twice-validated BC anchor; near-plant only (100% crouch rise starts), no walking yet (stage C). From-scratch is forced (obs-width change) and pre-registered in the plan. Prediction-if-true: hold/rise/lower coexist at near-specialist honesty by 2M (specialist discovery bars: holdbc1 hold 12/12 valid_plant, bc1 crouch rise 6/8). Prediction-if-false: the known feet-factor collapse / flag-leg cheat reappears despite anchor + mode input = multitask interference, fork to MoE per the flagship pre-registration. Strongest alternative: the BC anchor alone carries the skills and the mode bit is dead weight -- settled later by a mode-scrambled ablation, not this run.

**gate**: 2M det harness, own stack: hold valid_plant >= 4/6 AND rise (all-crouch starts) valid_plant >= 3/6 AND lower posture-strict >= 3/6, ZERO known-exploit fingerprints (flag-leg/tripod/park/freeze) in gated-mode video.

**verdict**: No science verdict — auto-retry launch attempt that died at 0 steps (W&B u5za5jof, steps=None), same 08-11 launch-infra failure chain; superseded by cw-uni-flag-a1-r1 which trained and is fully verdicted. Do not retry.

**failed_reason**: W&B global_step not advancing (0 -> 0)

