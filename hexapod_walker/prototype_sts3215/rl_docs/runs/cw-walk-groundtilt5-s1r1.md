# cw-walk-groundtilt5-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T22:33:17+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**hypothesis**: Retry of cw-walk-groundtilt5-s1 (died at env reset with worker EOFError, /dev/shm leak gotcha 13 -- W&B run names append-only so retried under -r1). Seed twin of PASSED cw-walk-groundtilt5 (0-5deg floor slope, DR0, isolated 13b axis off champion), promotion-panel completeness (ruling-7). Same config, seed 1. If-true: own-cfg gv 12/12, DR0 retention matches seed0's champion-band slip/prog, same steepest-tail shuffle pattern -- recipe confirmed. If-false: seed-sensitive result, groundtilt5's PASS was seed luck.

**gate**: Own-cfg harness tilt u(0,5deg) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare episode pattern to seed0 at triage; frames watched det

