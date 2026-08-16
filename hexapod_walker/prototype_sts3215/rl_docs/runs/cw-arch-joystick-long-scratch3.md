# cw-arch-joystick-long-scratch3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-15T22:29:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**wandb_id**: a7rtr3lq

**hardware_ready**: False

**hypothesis**: From random weights, learn durable joystick walking over 60-second sessions. Each PPO rollout spans a full session and each episode contains many instantaneous, irregular command changes drawn from random headings, 180-degree back-and-forth, square, stop-go, circle, and local-jitter schedules. A walk-only 90 mm chassis-drop termination closes the seated-scooting loophole and charges the remaining-session fall cost.

**gate**: Fresh initialization with no init checkpoint. Final C-MuJoCo gate requires at least 5 of 6 deterministic and 4 of 6 stochastic trials to survive the full 60 seconds; mean direction error at most 20 degrees, p90 at most 45 degrees, wrong-direction fraction at most 0.10, velocity error at most 0.02 m/s, no walk_low_height termination, and video must show upright controlled responses across multiple instantaneous command switches.

**verdict**: FAIL — 4th independent from-scratch joystick command-switch attempt, full 40M budget spent. The low-height termination did kill the seated-scooting cheat (gait_valid 6/6 det, 6/6 sto legs moving, no parked legs) but the robot still cant survive: DR0 gate 0/6 det + 0/6 sto, own-DR 0/6 + 0/6, every det episode roll_class=fell (over_current x2, tilt_roll x2, tilt_pitch x2, peak 8.7-10.5deg), forward_dist 1-6cm before collapse. Reconfirms the standing cross-track finding (tf-joymodes-scratch1, fallfix1, switch-scratch2, now this): the instantaneous command-switch reward/curriculum recipe itself is the blocker, immune to architecture, init, rollout horizon, fall pricing, AND the collapse-termination fix. No further chunks on this exact recipe from arch.

