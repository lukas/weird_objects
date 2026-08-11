# cw-stand-scoreref1-dr0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T01:27:45+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-scoreref1

**wandb_id**: aodyj39g

**hypothesis**: Operator hypothesis (08-10 21:26): DR is the discovery blocker. Walk was discovered at DR0 and hardened gradually; every rise arm tonight trained at DR0.2 (placement noise, 1.6-7deg bad starts, kp/kv spread) while the two mechanisms that must engage have thin margins by design (6-deg ref kernel, conjunction stand-score) -- scatter may be washing both out. The MDP_PREFLIGHT bank also validates at DR0, so DR0 training is the config the bank actually proves. ONE change vs cw-stand-scoreref1: --dr-scale 0.2 -> 0.0, identical reward stack (score income + feet-gated 6-deg ref crutch).

**gate**: posture-strict harness DR0: rise >=4/6 det valid_plant AND lower retains >=5/6 by 2M; W&B env/rise_score must lift off the 0.01 floor and env/reward_rise_ref off the 0.02/tick floor (both flatlined in scoreref1 at DR0.2). If BOTH still flatline at DR0, DR is exonerated and the blocker is kernel-basin exploration -> RSI (reference state init) is the next mechanism, already scoped.

