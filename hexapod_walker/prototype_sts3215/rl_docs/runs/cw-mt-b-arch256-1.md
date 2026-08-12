# cw-mt-b-arch256-1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T21:28:49+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-mt-b1

**wandb_id**: qf4cyj51

**hardware_ready**: False

**hypothesis**: Wave-2 capacity lever for the multitask track (rl_docs/MULTITASK.md 'Later waves'), now that wave 1 has read out: at the SAME 2M discovery budget where the 128x128 narrow-generalist baseline (cw-mt-b1) completely failed to walk (det prog med 0.16, gait_valid 0/6, low-crouch splay creep), does simply widening the policy/value network to 256x256 change discovery at all? Fresh init, same command distribution (vx 0-0.06, +-0.15 rad/s yaw on 20% of segments, 40% stop segments) -- net width is the ONLY variable. Prediction-if-true: 256x256 shows meaningfully more forward progress and/or partial gait_valid at 2M than b1's 0.16/0-of-6 baseline -- a positive discovery signal worth scaling to 20M as its own arm. Prediction-if-false: same nothing-walks-yet low-crouch creep regardless of width -- capacity is not the discovery bottleneck, consistent with the fact that b2 (128x128, 20M) already produces a REAL six-leg gait (just underpowered on speed/yaw), meaning 128x128 was never too small to represent walking. Strongest alternative: wider net changes discovery speed but not final quality, which this 2M probe cannot resolve either way and would need a separate 20M arm.

**gate**: At 2M: det prog med >= 0.32 (2x cw-mt-b1's 0.16) OR gait_valid >=1/6 det counts as a positive capacity signal -> queue a matched 20M 256x256 arm next. FAIL if prog med and gait_valid both stay at/below cw-mt-b1's baseline (0.16, 0/6) -- capacity is not the lever; wave-2/phase-2 planning stays on b2/a2 (128x128) per multitask/STATUS.md, no further net-arch retry at this budget.

**verdict**: FAIL per pre-registered gate: width alone (256x256, fresh init, 2M) does NOT beat cw-mt-b1's 2M baseline — gate(DR0) det prog med 0.11 vs baseline 0.16 (gate needed >=0.32), gait_valid 0/6 det (needed >=1/6); owncfg(DR0.2) det likewise 0.12 / 0/6. Same low-crouch splay + leg-1 near-sacrifice (sac [1] in 11/12 det eps, slip med 8-9/m, fwd ~0.10m) as b1, video-confirmed. Capacity at 2M is not the lever; per the gate's FAIL branch wave-2/phase-2 planning stays on a2/b2 (128x128), no further net-arch retry at this budget.

