# cw-walk-fricvar-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:28:14+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-fricvar

**wandb_id**: 617y9ouq

**hardware_ready**: no

**hypothesis**: Seed twin of PASSED cw-walk-fricvar (0.4-1.6x floor grip, DR0, isolated 13b axis off champion). Promotion-panel completeness (ruling-7) -- same config, seed 1. If-true: own-cfg gv 12/12, DR0 retention matches seed0's champion band, same slickest-tail churn pattern -- recipe confirmed, not luck (mirrors comshift-dr05-s1's seed-robustness method). If-false: seed-sensitive, fricvar's PASS was seed luck.

**gate**: Own-cfg harness grip 0.4-1.6x det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m <=1.24, prog>=0.9; compare episode pattern to seed0 at triage; frames watched det

**verdict**: PASS -- seed twin confirms cw-walk-fricvar (floor grip 0.4-1.6x, DR0). Own-cfg gv 12/12, 0 term, det prog median 0.88 (gate 0.75); 2/6 slickest det draws churn near-in-place (prog 0.37-0.46, slip/m 3.7-4.1, fwd 0.66-0.70m) -- same slickest-tail pattern as seed0 (prog 0.36-0.56, slip 2.4-4.2), not seed luck. DR0 nominal retention CLEAN: det gv 6/6, slip/m 0.91-1.16, fwd 1.52-1.64m, prog ~1.0 -- matches/slightly beats seed0's retention (slip 1.09). Recipe confirmed, ruling-7 panel satisfied for this axis. Paddle lineage, not hardware-ready.

