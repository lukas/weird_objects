# cw-walk-deadband30-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:26:58+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-deadband30

**wandb_id**: 2q8xd8j7

**hardware_ready**: no

**hypothesis**: Seed twin of PASSED cw-walk-deadband30 (servo deadband 1-3x nominal, DR0, isolated 13b/13c axis off champion). Promotion-panel completeness (ruling-7) -- same config, seed 1. If-true: own-cfg gv 12/12, DR0 retention matches seed0's champion band, same worst-draw pattern (0.68m/slip 2.86-ish) -- recipe confirmed, not luck. If-false: seed-sensitive, deadband30's PASS was seed luck.

**gate**: Own-cfg harness deadband 1-3x det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; DR0 nominal retention det 6/6 gv, slip/m <=1.24; compare episode pattern to seed0 at triage; frames watched det

**verdict**: PASS -- seed twin confirms cw-walk-deadband30 (servo deadband 1-3x, DR0). Own-cfg gv 12/12, 0 term, det med fwd 1.33m (gate 1.2); 2/6 det draws crater (0.66/0.70m, slip 3.3-3.5) matching seed0's worst-draw pattern almost exactly (seed0: 0.68m/slip~2.86) -- same pathology (slow shuffle on the hardest deadband draws, no falls, no flag leg), not seed luck. DR0 nominal retention CLEAN: det gv 6/6, slip/m 1.09-1.27 (well under 1.24 cap), fwd 1.36-1.58m -- matches seed0's clean retention. Recipe confirmed, ruling-7 panel satisfied for this axis. Paddle lineage, not hardware-ready.

