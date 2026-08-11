# cw-arch-gru-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T17:52:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-gru-r3

**wandb_id**: f5gbd63w

**hypothesis**: r3 showed the GRU learns every stance skill to champion grade but walks straight into the leg-sacrifice/paddle cheat that the same reward stack prices out for MLPs; its 2.56s BPTT window cannot carry credit across a full swing-recover-replant cycle plus the progression-gate horizon, and 128 hidden units must compress what the MLP reads directly from a 16-frame stack. If a 10.24s window + 256 hidden units produces a cheat-free walk attempt at 2M, the rung reopens for hardening; if the identical fingerprint survives BOTH levers, from-scratch GRU walking is dead at this budget and recurrence waits for flagship distillation.

**gate**: PASS: 2M harness forensics (DR0 det+sto walk, standard fingerprint) show NO leg-sacrifice (det gait_valid > 0/6, no parked-leg collapse at duty<0.1), positive det median progress ratio, AND stance still emerging (>=1 rise completion, tipped recovery >=1/2). Walk tracking quality NOT required. FAIL: same parked-leg/paddle fingerprint -> rung closes for real (window AND capacity exonerated the reward twice); no further from-scratch GRU variants, recurrence deferred to flagship distillation.

**verdict**: OPERATOR KILL (mis-launch, no science): n_envs 1024 x n_steps 256 = 262k rollout = only 8 PPO iterations in the 2M budget (r3 had 30) — update-starved by construction. Killed ~5 min in. Superseded by cw-arch-gru-r4c (n_envs 256 keeps rollout at 65,536 = r3-identical iteration count). Do NOT auto-retry this config.

**failed_reason**: W&B global_step not advancing (262144 -> 262144)

