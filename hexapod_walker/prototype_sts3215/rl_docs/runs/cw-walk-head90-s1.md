# cw-walk-head90-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:19:27+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-wander30

**hardware_ready**: no

**hypothesis**: Seed twin of head90 (PASS c51, single-seed): promotion ruling 7 requires multi-seed panels, and the front-hemisphere envelope is the base every joystick arm composes on. IDENTICAL config to head90, one variable: seed 0->1. Plain: prove driving anywhere in the front half-circle is the recipe, not seed luck. Prediction-if-true: s1 reproduces the gate (gv 12/12, 0 term, lateral err <=2x fwd) - envelope is seed-robust, panel-ready. Prediction-if-false: s1 misses (flag leg, lateral collapse, or falls) - head90 was seed luck and the ladder needs re-evidence before wider rungs ride on it. Strongest alternative: passes but with a DIFFERENT left/right asymmetry signature than head90's (left strafe ~half of right) - asymmetry is seed-level noise and the mirror-symmetry [CODE] line gains urgency.

**gate**: own-cfg DR0 30s det+sto 6/6 resampled +-90deg cmds: gait_valid 12/12, 0 term, no sacrificed leg, lateral tracking err <=2x forward err; compare L/R strafe displacement vs head90; frames watched det

**verdict**: PASS (seed twin confirms ±90deg envelope, ruling-7 evidence). Own-cfg DR0 gv 12/12, 0 term, 0 sacrificed legs; det prog med 0.89/slip 1.46/fwd 1.00m (head90: 0.84/1.56/0.94 — same band, slightly better); lateral trk err 1.55x fwd (gate <=2x); JOYSTICK GATE PASS @DR0.2, 90deg envelope, 0 falls incl flip-stress. L/R strafe 0.234/0.295 (ratio 0.79) vs head90's 0.147/0.274 (0.54) — head90's weak left strafe does NOT reproduce: L/R asymmetry is seed-level noise, not a structural defect; mirror-symmetry line stays useful but loses its urgency trigger. Front-hemisphere ±90 envelope is seed-robust and panel-ready. Paddle lineage — not hardware-ready.

**refused_reason**: hexapod-mjx-train-6 already runs cw-walk-stiffvar — GPU pods host exactly one run; pick a free GPU pod.

