# cw-arch-joystick-switch-scratch2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T20:18:57+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**hypothesis**: From random weights, learn stable 15-second joystick translation with instantaneous command switches by matching PPO credit assignment to the whole episode. Each episode contains multiple commands and draws a structured schedule including random holds, 180-degree back-and-forth, square turns, stop-go, circle sweeps, or local jitter; discrete switch intervals vary from about 0.8 to 7.2 seconds. The recipe uses n_steps=384, n_envs=128, gamma=0.995, GAE=0.97, normal entropy pressure, stronger dense tilt cost, and the full 40M budget.

**gate**: No reward-only pass. Fresh initialization must be confirmed by absence of --init-from. Final C-MuJoCo gate requires at least 5/6 deterministic and 4/6 stochastic walk trials to survive the full 15 seconds, mean direction error <=20 deg, p90 <=45 deg, wrong-direction fraction <=0.10, velocity error <=0.02 m/s, and video showing controlled responses to multiple instantaneous command switches without tilt termination.

**refused_reason**: hexapod-mjx-train-3 code marker 500e856b5695747d11ba1918d7c417216510b9c8 != local HEAD 9009c90d3620291945deb66fd24600c7a98f8e9c. Sync first: snapshot.sh --sync hexapod-mjx-train-3 (and snapshot/commit before that if the tree is dirty).

