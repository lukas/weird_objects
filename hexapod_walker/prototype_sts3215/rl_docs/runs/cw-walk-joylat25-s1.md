# cw-walk-joylat25-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T19:22:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-c1

**wandb_id**: q5o8osnu

**hypothesis**: Seed concordance for the new best driving candidate (ruling 7: promotion needs multi-seed panels): identical config to cw-walk-joylat25 (DR0.5 + dr.latency_scale=0.5,2.5 abrupt-resample driving package off joyjit-dr05-c1) with ONLY the seed changed 0->1. Plain: prove the latency-hardened driving result was not a lucky seed. If-true: joystick gate zero falls + own-cfg gv 12/12, prog med >=0.85, slip/m in steering band 1.3-1.8 - result seed-CONFIRMED, panel evidence banked. If-false (gate miss or slip/prog off-band): joylat25's PASS is seed-fragile, demote to single-seed evidence and investigate before composing further. Strongest alternative: both seeds pass but drift to different gaits - compare cadence/duty columns.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 retention det 6/6 gv; frames watched det

