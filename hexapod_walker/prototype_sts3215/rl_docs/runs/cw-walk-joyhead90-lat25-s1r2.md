# cw-walk-joyhead90-lat25-s1r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T22:42:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**hypothesis**: Second retry of joyhead90-lat25 seed-1 twin (first two attempts both lost launch-collision races amid a concurrent-cycle drain storm on train-0/train-3/train-9, worker EOFError at init, 0 steps each -- no science result either time). Same hypothesis: seed-1 replication of joyhead90-lat25 PASS (+-90deg abrupt flips + DR0.5 + latency 0.5-2.5x) per ruling-7. If-true: JOYSTICK GATE @90 0 falls AND own-cfg DR0.5+latency harness gv 12/12, 0 term, prog med >=0.80 (matches seed0 0.93/0.96). If-false: seed0 was lucky.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5+latency harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

