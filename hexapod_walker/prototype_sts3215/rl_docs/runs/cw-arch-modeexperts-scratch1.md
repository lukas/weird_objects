# cw-arch-modeexperts-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-15T02:07:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: aiwkxeho

**hypothesis**: Can a robot brain built as four fully walled-off specialists inside one checkpoint (stand-up, hold, sit-down, walking - each with its own memory, its own value estimate and its own exploration noise) learn to walk, stand up and sit down FROM RANDOM WEIGHTS, with no imitation of any teacher? This 2M-step canary answers only the MECHANISM question - does the new --gru-experts architecture train stably (no NaN/crash, all four experts receive their modes' ticks, per-expert std/gradients move independently)? Skill acquisition is NOT judged here: budget honesty per operator directive fb_20260815T013349_488ffd says walking alone needed ~20M active ticks (cw-mt-a2), so the pre-registered staged full budget (~60M, MODE_EXPERTS_DIRECTIVE.md Arm B) launches only after this canary passes. From-scratch is genuine: no init checkpoint, no BC anchor, no DAgger, and reward.rise_ref tracking is EXCLUDED (demo-derived = hidden imitation); disclosed curriculum = validated env reward stack + goal-mix walk/rise/lower/hold 0.35/0.35/0.20/0.10 + 50% chained mode sequences; no transition adapter (a shared trainable residual would reopen a cross-mode gradient channel). Operator override: multitask pause + nobc gait closure explicitly lifted for this arm.

**gate**: MECHANISM HEALTH ONLY (pre-registered, MODE_EXPERTS_DIRECTIVE.md Arm B stage 1): run completes 2M with no NaN/crash/canary-stop; W&B experts/tick_frac_* shows all four experts active with fractions within ~0.10 of the commanded mix; experts/std_* diverge per expert (not identical curves); per-expert learning signal visible (any expert's env reward terms moving). PASS = launch pre-registered stage 2 (scratch2, 40M). Skill success/failure at 2M is explicitly NOT part of this gate and produces NO capacity verdict.

**verdict**: INFRA FAIL (not science): trainer died silently ~2min after learn() start on train-0 (no traceback, no OOM — oom_kill 0; pod was shared with another cycle's CPU distill/ffmpeg workload at the time). Identical retry cw-arch-modeexperts-scratch1-r1 on clean train-2 VERIFIED RUNNING (fps ~168), so the --gru-experts stack itself trains. No science verdict; the Arm B mechanism gate transfers to -r1.

**failed_reason**: W&B global_step not advancing (0 -> 0) after 1200s (n_steps=256, cpu-time flat for 0 polls)

