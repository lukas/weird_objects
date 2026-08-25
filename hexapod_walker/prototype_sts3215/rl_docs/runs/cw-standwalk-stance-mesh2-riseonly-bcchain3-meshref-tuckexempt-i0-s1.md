# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt-i0-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - INFRASTRUCTURE

**created**: 2026-08-25T19:16:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt-i0

**wandb_id**: 9pivvxai

**hypothesis**: Seed twin of tuckexempt-i0 (same recipe, seed=1): does the tuck-exempt BC-anchor floor's flat-tuck rescue replicate across seeds, the same joint-pair discipline used for every prior mesh-rise canary in this rung?

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as tuckexempt-i0 (joint pair): PASS/PARTIAL/FAIL per that run's registered clauses, read jointly with seed-0.

**verdict**: CANARY FAIL - INFRASTRUCTURE: exact duplicate launch, not a science result. Bit-identical (seed=1, same cfg incl. train.bc_anchor_min_h_ahead_mm=8 + train.bc_anchor_min_h_tuck_exempt_i0=1) to the concurrent cycle's cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt0-s1 (train-2, created 19:14:37, before mine at 19:16:48). Same root cause as its seed-0 twin: launched believing the pre-registered pair had never landed. Killed cleanly on train-3 (kill sent to pids 99719/99720/99725, confirmed dead via ps afterward), no information lost. tuckexempt0/-s1 is the pair of record; do not relaunch.

