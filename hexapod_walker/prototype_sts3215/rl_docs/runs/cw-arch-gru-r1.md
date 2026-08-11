# cw-arch-gru-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T12:54:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: ej7fcabu

**hypothesis**: Temporal-arch ladder rung 4 (WISHLIST): a GRU recurrent actor-critic (128 hidden, BPTT 64 steps = 2.6 s) replaces the frame-stack MLP and learns joystick walking AND sit/stand (rise/lower) in ONE policy from scratch on joint_walk with a mixed goal diet. Discovery question: does qualitatively correct behavior emerge in all modes from a single recurrent net? If yes -> adjusted r2, then a hardening-length run; if no -> adjust recipe (BPTT window, ent, goal mix) before spending more budget.

**gate**: PASS if the 2M det eval shows walk episodes tracking the commanded velocity (eval/walk/vel_err_m_s clearly below the untrained ~0.06 baseline and moving with the stick), >50% survival across modes, and at least one rise or lower completion. FAIL = any mode totally absent or walk not directional.

