# probe-walk-hist8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T22:31:37+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**hypothesis**: MECHANICAL PROBE (audit sec6: new mechanism = smoke first). obs.history_frames=8 on joint_walk: env-side newest-first stack (obs 576), tail transplant from stance champ via --obs-pad-transplant 508 (bit-identical parity verified locally, max action diff 0.0). If-true: trainer healthy at 150k, obs width 576 end-to-end, canary baseline matches parent (zero-column history invisible at init), fps within ~2x of a plain walk run on the 30-core pod. If-false: crash/width mismatch/canary baseline broken -> fix code before any 4M launch. No behavioral claim at 150k.

**gate**: mechanical only: process healthy to 150k, no traceback, canary protected groups pass at baseline, W&B-off smoke

**verdict**: PROBE PASS (mechanical, cycle 13): 150k steps in 89s on lower pod, obs-pad transplant 68->576 fired, canary parent baseline all 8 cases pass (4 groups protected) — confirms bit-identical warm start on-pod; zero tracebacks; internal eval rise 2/2 raise 1/1 track 0.81deg. checkup DEAD verdict = process exited after completing the smoke budget, not a crash (final save line + checkpoint present). No behavioral claim at 150k.

