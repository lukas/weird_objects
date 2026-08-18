# cw-arch-joystick-switch-scratch2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-15T20:40:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**wandb_id**: 9olozoz7

**hardware_ready**: False

**hypothesis**: From random weights, learn stable 15-second joystick translation with instantaneous command switches by matching PPO credit assignment to the whole episode. Each episode contains multiple commands and draws a structured schedule including random holds, 180-degree back-and-forth, square turns, stop-go, circle sweeps, or local jitter; discrete switch intervals vary from about 0.8 to 7.2 seconds. The recipe uses n_steps=384, n_envs=128, gamma=0.995, GAE=0.97, normal entropy pressure, stronger dense tilt cost, and the full 40M budget.

**gate**: No reward-only pass. Fresh initialization must be confirmed by absence of --init-from. Final C-MuJoCo gate requires at least 5/6 deterministic and 4/6 stochastic walk trials to survive the full 15 seconds, mean direction error <=20 deg, p90 <=45 deg, wrong-direction fraction <=0.10, velocity error <=0.02 m/s, and video showing controlled responses to multiple instantaneous command switches without tilt termination.

**verdict**: FAIL — full 40M-step from-scratch acquisition never learned to survive. DR0 gate + own-cfg (DR0.3) both 0/6 det and 0/6 sto walk success, gait_valid 0/6, roll_class fell in every deterministic episode, terminated tilt_roll at t=0.84s (roll_peak 10.6deg) having moved 4mm; sacrificed_legs [3,4] (gate) / [0,3,4] (owncfg) in all trials — the recurring parked-leg pattern. Training reward improved (-1015->-229 across quarters) but the real task metrics did not: eval/walk/survived_frac=0 throughout, dir_err_deg_mean 90 (worse than fallfix1's 19), wrong_dir_frac 0.32. This is the third independent from-scratch attempt at the joystick command-switch recipe on arch (after tf-joymodes-scratch1 known-exploit FAIL and its fallfix1 FAIL) to fail identically — confirms CURRENT_TRUTHS/arch-STATUS cross-track finding that the command-tracking reward/curriculum recipe itself, not architecture/init/fall-pricing, is the blocker. No further chunks on this exact recipe; the already-running cw-arch-joystick-long-scratch3 (60s horizon, explicit low-height collapse termination, spawned off an early switch-scratch2 checkpoint read) is the standing next attempt and needs no new launch.

