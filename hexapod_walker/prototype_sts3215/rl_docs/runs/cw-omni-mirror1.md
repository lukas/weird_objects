# cw-omni-mirror1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T22:58:16+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-hist16-dep1

**wandb_id**: 9lza65vp

**hypothesis**: Mirror-symmetry mechanism probe (operator, 08-10 evening). Every walk policy carries a command-invariant ~+0.09 rad/s left-yaw drift; EIGHT reward-side arms across two mechanism families failed to move it (price tuning doubly closed, turnfix1 matched-parent null). The untried lever is structural: MirrorPPO (rl_move/sim/mirror.py, snapshot 95e133e) adds a soft auxiliary loss pushing pi(mirror(obs)) toward mirror(pi(obs)) — legs 0-5/1-4/2-3 swapped, yaw negated, roll/vy/wz flipped; SOFT because the 25.65 mm hip-anchor offset makes the body slightly chiral. This DISCOVERY probe asks only the mechanism-health binary on the full omni stack (hist16-dep1 boot recipe + full-circle headings + turn-in-place 30 pct + signed yaw income + dep contract + k_current=0, coef 1.0, 4x4096 aux minibatches per rollout): does train/mirror_sym_loss fall materially while PPO trains normally from scratch? If-true: relaunch as hardening 40M (cw-omni-mirror1-r1) — the run intended to produce the omnidirectional joystick policy. If-false (sym loss flat/NaN, or reward collapses vs cw-arch-hist16-dep1 first-2M band): the aux step is mis-scaled — tune coef/minibatch count before any long budget. MDP_PREFLIGHT: OMNI bank added to test_task_semantics.py and PASSING 08-10 (gait beats stall+park in all four directions, direction income even, turn ordering survives repricing); test_mirror.py 11/11 incl. live-env MirrorPPO integration.

**gate**: Mechanism health at 2M, NOT behavior (from-scratch gaits do not exist at 2M — dep1 precedent): (1) train/mirror_sym_loss at end < 0.5x its early value and trending down; (2) W&B reward quarters within the cw-arch-hist16-dep1 first-2M band (no collapse, no NaN, std sane); (3) fps within pod norms (aux step adds <15 pct). Behavioral verdict (eval_yaw both signs, joystick panel, drift at zero command) belongs to the 40M hardening follow-up.

