# cw-walk-joyhead90-lat25-s1r

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:30:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**hypothesis**: Retry of cw-walk-joyhead90-lat25-s1 (lost a launch-placement race, worker EOFError at init, 0 steps — no science result). Same hypothesis: seed-1 replication of today's joyhead90-lat25 PASS (widest driving package: +-90deg abrupt flips + DR0.5 + bus-latency 0.5-2.5x jitter) per ruling-7. If-true: JOYSTICK GATE @90 0 falls AND own-cfg DR0.5+latency harness gv 12/12, 0 term, prog med >=0.80 — matches seed0 (0.93/0.96). If-false: seed0 was lucky — widest package is seed-fragile.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5+latency harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**failed_reason**: Launch-collision, not a science result: many concurrent cycles' drains fired near-simultaneously at the same handful of free GPU pods right now (observed ps aux: 3+ other cycles' launch_run.py launch processes targeting train-0/train-3/train-9 within the same ~2min window). Two different runs landed on train-0 back-to-back within seconds; worker EOFError at env reset (GPU/multiproc contention), 0 steps. Requeuing to backlog for the self-repairing drain to retry once the storm clears rather than fighting it directly.

