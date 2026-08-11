# cw-arch-gru-r4-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T18:08:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-gru-r4

**wandb_id**: 1id129gv

**hypothesis**: Mechanical retry of cw-arch-gru-r4 (identical from-scratch GRU config: BPTT window 256 steps / 10.24s, hidden 256, mixed diet walk.60/rise.15/lower.15/hold.10): the first attempt never got a real science verdict — the launcher's fixed 90s post-launch liveness check false-killed it after iteration 1 alone logged 84s of pure env-collection time (before the heavier GRU backward pass), misreading a slow-but-live long-BPTT run as stalled. Fixed in the same commit (launch_run.py: post-launch wait now scales with --n-steps, capped at 600s) and this is the clean re-run on the fixed launcher, config completely unchanged.

**gate**: PASS: 2M harness forensics (DR0 det+sto walk, standard fingerprint) show NO leg-sacrifice (det gait_valid > 0/6, no parked-leg collapse at duty<0.1), positive det median progress ratio, AND stance still emerging (>=1 rise completion, tipped recovery >=1/2). Walk tracking quality NOT required. FAIL: same parked-leg/paddle fingerprint -> rung closes for real (window AND capacity exonerated the reward twice); no further from-scratch GRU variants, recurrence deferred to flagship distillation.

**verdict**: MECHANICAL, not a science result: killed by the launcher again despite the n_steps-scaled wait (360s) — direct /proc CPU-time sampling on the pod during this attempt confirmed the process was genuinely alive and computing (utime climbing at ~25 cores, matching host-workers=24; GPU util 0% because the GRU policys PPO update runs on CPU per the trainers own "Using cpu device" line), it just needed longer than 360s to finish a 256-step BPTT backward pass over a 262144-sample rollout. Fixed properly in launch_run.py (CPU-liveness polling: keeps waiting past budget while cumulative CPU time is still climbing, only fails on two consecutive flat samples, capped 20 min). A CONCURRENT cycle independently hit + diagnosed the same infra bug via its own r4b attempt and is already running the real retry as cw-arch-gru-r4c (verified, global_step advancing) — no duplicate rr2 launched from here; cw-arch-gru-r4c is the run carrying this hypothesis forward.

**failed_reason**: W&B global_step not advancing (262144 -> 0) after 360s wait (n_steps=256)

