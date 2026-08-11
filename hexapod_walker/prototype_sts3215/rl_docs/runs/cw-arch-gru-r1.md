# cw-arch-gru-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T12:54:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: ej7fcabu

**hardware_ready**: no

**hypothesis**: Temporal-arch ladder rung 4 (WISHLIST): a GRU recurrent actor-critic (128 hidden, BPTT 64 steps = 2.6 s) replaces the frame-stack MLP and learns joystick walking AND sit/stand (rise/lower) in ONE policy from scratch on joint_walk with a mixed goal diet. Discovery question: does qualitatively correct behavior emerge in all modes from a single recurrent net? If yes -> adjusted r2, then a hardening-length run; if no -> adjust recipe (BPTT window, ent, goal mix) before spending more budget.

**gate**: PASS if the 2M det eval shows walk episodes tracking the commanded velocity (eval/walk/vel_err_m_s clearly below the untrained ~0.06 baseline and moving with the stick), >50% survival across modes, and at least one rise or lower completion. FAIL = any mode totally absent or walk not directional.

**verdict**: FAIL (known exploit, no forensics needed): det walk collapses to a static tripod leg-sacrifice (legs [1,3,5] parked, speed 0.004 m/s, gait_valid 0/6, prog_ratio 0.00); sto walk avoids the leg-sacrifice flag (gait_valid 6/6) but is a jittery no-net-progress paddle (prog med -0.02, slip/m 17, 2/6 pitch-tilt falls). Return climbed (-68->53 over 2M) while the real task (directional walk) never moved -- reward/spec shortcut, not learning. Rise/lower/hold did show some life in training's own periodic eval (rise crouch 2/2, bridge 1/2, tipped 2/2) but the pre-registered gate needed walk to be directional, which it is not -- GATE FAIL. GRU-from-scratch on a 0.55/0.15/0.15/0.15 mixed diet did not solve (or even attempt) walking in 2M steps; recipe (BPTT window/ent/goal-mix) unadjusted since arch ladder is frozen pending the flagship and this is not on the blocker list -- no continuation queued.

