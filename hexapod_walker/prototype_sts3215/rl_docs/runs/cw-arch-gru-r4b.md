# cw-arch-gru-r4b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T18:03:54+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-gru-r3

**wandb_id**: eh66kiqp

**hypothesis**: r3 showed the GRU learns every stance skill to champion grade but walks straight into the leg-sacrifice/paddle cheat that the same reward stack prices out for MLPs; its 2.56s BPTT window cannot carry credit across a full swing-recover-replant cycle plus the progression-gate horizon, and 128 hidden units must compress what the MLP reads directly from a 16-frame stack. If a 10.24s window + 256 hidden units produces a cheat-free walk attempt at 2M, the rung reopens for hardening; if the identical fingerprint survives BOTH levers, from-scratch GRU walking is dead at this budget and recurrence waits for flagship distillation.

**gate**: PASS: 2M harness forensics (DR0 det+sto walk, standard fingerprint) show NO leg-sacrifice (det gait_valid > 0/6, no parked-leg collapse at duty<0.1), positive det median progress ratio, AND stance still emerging (>=1 rise completion, tipped recovery >=1/2). Walk tracking quality NOT required. FAIL: same parked-leg/paddle fingerprint -> rung closes for real (window AND capacity exonerated the reward twice); no further from-scratch GRU variants, recurrence deferred to flagship distillation.

**verdict**: INFRA FAIL, config was CORRECT (n_envs 256 / n_steps 256 / hidden 256): launch verifier timed out inside the first slow BPTT-256 update (one 65,536-step rollout logged, update takes minutes) and marked FAILED; trainer was then killed in the cw-arch-gru-r4-rr1 launch collision. Relaunched verbatim as cw-arch-gru-r4c — expect the same one-rollout verifier timeout, the run is healthy.

**failed_reason**: W&B global_step not advancing (65536 -> 65536)

