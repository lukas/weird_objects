# cw-arch-modeexperts-scratch1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T02:33:52+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**wandb_id**: ww6xarcw

**hypothesis**: Can a robot brain built as four fully walled-off specialists inside one checkpoint (stand-up, hold, sit-down, walking - each with its own memory, value estimate and exploration noise) learn to walk, stand up and sit down FROM RANDOM WEIGHTS, with no imitation? 2M MECHANISM canary (identical retry of cw-arch-modeexperts-scratch1, which died silently ~2min in on train-0 with no traceback while another cycle's CPU distills shared that pod; this retry runs on a clean pod). Judges ONLY mechanism health - stable training, all four experts receiving their modes' ticks, independent per-expert std/gradients. Skill acquisition NOT judged at 2M (budget honesty: cw-mt-a2 needed ~20M active walk ticks; staged ~60M full budget pre-registered in MODE_EXPERTS_DIRECTIVE.md Arm B). Genuinely from scratch: no init checkpoint, no BC anchor/DAgger, rise_ref tracking EXCLUDED; disclosed curriculum = validated env reward stack + goal-mix 0.35/0.35/0.20/0.10 + 50% mode_seq sessions; no transition adapter. Operator directive fb_20260815T013349_488ffd (multitask pause + nobc gait closure lifted for this arm).

**gate**: MECHANISM HEALTH ONLY (pre-registered, MODE_EXPERTS_DIRECTIVE.md Arm B stage 1): completes 2M with no NaN/crash/canary-stop; experts/tick_frac_* shows all four experts active within ~0.10 of the commanded mix; experts/std_* diverge per expert; per-expert learning signal visible. PASS = launch pre-registered stage 2 (scratch2, 40M). Skill success at 2M is explicitly NOT gated and produces NO capacity verdict. Second silent death = NEEDS OPERATOR (infra).

