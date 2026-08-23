# cw-arch-hist16-dep1-s1r1-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T18:12:34+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-s1r1

**hypothesis**: Plain English: operator-requested replication — does the impressive hist16-dep1 -> c1 continuation result reproduce on the seed twin? cw-arch-hist16-dep1-c1 (+40M continuation, seed 0) closed the dep-contract walker's efficiency gap and PASSed; the from-scratch recipe already replicated on seed 1 (cw-arch-hist16-dep1-s1r1, PASS, det slip 1.28-1.35). This run is the missing closest replication of the continuation step: continue s1r1 for +40M with c1's exact config (joint_walk, DR0.5, hist16, deployment contract walk_obs_body_vel=2, same walk reward/goal cfg), changing only parent/init/name and keeping s1r1's seed 1. If-true: slip/economy improves or holds toward c1's levels (det ~<=1.25, sto ~<=1.37) with 6/6 gait_valid, 0 term, joystick gate 0 falls — the c1 result is a reproducible recipe, not seed-0 luck. If-false: seed-1 continuation stalls or degrades — the c1 gain is seed-specific or noise, and the lineage claim needs an audit before any further continuation. Per operator note 20260823T180148Z: if reward rises while eval/visual gait worsens, stop and audit reward/eval disagreement instead of running more seeds.

**gate**: vs parent s1r1 and vs c1: own-cfg DR0.5 det+sto 6/6 gait_valid, 0 terminations; DR0 gate det+sto 6/6 gv, 0 term; joystick gate (eval_drive) 0 falls; video visually ordinary six-leg gait, no flag leg; slip/m improves or holds from s1r1's det 1.28-1.35 toward c1 levels (target det <=~1.25, sto <=~1.37)

