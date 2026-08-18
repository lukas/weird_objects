# cw-dynrep-criticD-walkcurr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T04:50:05+00:00

**pod**: hexapod-mjx-train-4

**steps**: 40000000

**git_sha**: 5e7c1db3f3b216eff9dadf496fc013a313a3c691

**wandb_id**: 137olxtr

**hypothesis**: Teach the walking robot joystick commands the way a coach would: master slow straight walking first, then wider speeds, then angled headings, then smooth direction changes, then stop-and-go, then rougher physics - each rung unlocked only after the robot PROVES it can do the new rung AND still do every old one on held-out deterministic tests, with automatic rollback to the last proven checkpoint if old skills decay. This arm tests whether that adaptive curriculum beats the same 40M-step run that throws the full random command mix at the robot from step 0 (cw-dynrep-criticD-40m1) - everything else (fresh actor, exact frozen dynamics-transformer critic, rewards, optimizer, evals) is identical. Operator order 2026-08-18 (MCP lane, GPT-5 Codex for Lukas).

**gate**: Pre-registered 40M decision checkpoint, no extension without a verdict. Judged on the BEST checkpoint = last retention-clean promotion (never reward/latest). PASS = (a) frontier reaches >= B6 (stop/restart mastered with retention) by 40M, AND (b) on the shared 6-episode command-rich own-DR walk eval the promotion checkpoint meets the 40m1 gate bars (early_term_rate 0, cmd_prog_frac >= 0.6, slip_per_m <= 2.0, peak_roll_deg <= 8, contact_sw_per_s >= 3, SCORE/loco_quality >= 10), AND (c) beats cw-dynrep-criticD-40m1's best-loco checkpoint on loco_quality at matched or better slip/roll (a higher-reward pick with worse drag/roll is not a PASS). Mechanical invariants: encoder md5 match, pred/snapshot_version 0, per-bucket walkcurr/* W&B panels advancing, promotions saved, no locked-bucket training (cert logs). Rise/hold retention curves reported alongside (monitored, scratch actor - no hard gate). Verdict must quote per-bucket cert table + visual-quality stats vs 40m1.

**note**: Script-owned (pod_criticDwalkcurr.sh, manifest criticDwalkcurr_manifest.jsonl on-pod). Operator order 2026-08-18 MCP lane 20260818T041434Z. Code 5e7c1db3 (main, tag exp/cw-dynrep-criticD-walkcurr1). ONE-VARIABLE twin of cw-dynrep-criticD-40m1 (same seed 8, n_envs 16, 40M, frozen vt2ovznc critic D md5 9df48f687967c25085ee50171e4110ff asserted, same eval cadence AND eval command distribution): ONLY the training command sampling changes, from the fixed broad resample mix to the default-off adaptive competence+retention frontier curriculum (goal.walk_curriculum=1, WALKCURR_BUCKETS B0 fwd 0.04-0.05 long-holds DR0 ... B10 rear; cert every 0.5M deterministic same-backend held-out seeds n=8/bucket; promote only on frontier+all-retained pass; 2 consecutive retained failures = rollback; best checkpoint = last retention-clean promotion, never reward/latest). CUDA canary passed on train-4 04:45-04:48Z (md5 assert, cert rounds, clean exit).

