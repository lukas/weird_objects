# cw-walk-joyhead90-lat25-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:22:58+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**hypothesis**: Seed-1 replication of today's joyhead90-lat25 PASS (widest driving package: +-90deg abrupt flips + DR0.5 + bus-latency 0.5-2.5x jitter) per ruling-7 — confirm before promoting to unified-policy warm-start. If-true: JOYSTICK GATE @90 0 falls AND own-cfg DR0.5+latency harness gv 12/12, 0 term, prog med >=0.80 — matches seed0 (0.93/0.96) — compose is seed-robust. If-false: seed0 was lucky, retention/falls appear with this seed — widest package is seed-fragile, needs a wider panel before it anchors the unified policy.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5+latency harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

