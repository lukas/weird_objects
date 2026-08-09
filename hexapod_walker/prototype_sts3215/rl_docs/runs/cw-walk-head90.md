# cw-walk-head90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T15:06:42+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-wander30

**wandb_id**: wymqwc2j

**hardware_ready**: no

**hypothesis**: Heading LADDER rung 1 (after cw-walk-backforth 0/12 FAIL confirmed abrupt +-180 widening breaks the gait, matching steer-explore/fdiag — binding review 9 said widen gradually, not abruptly). The robot already walks and changes direction within +-45deg (wander30); this run widens commands to +-90deg warm-started FROM wander30 itself, one rung not four. If-true: gait_valid tracking at +-90 (strafe-capable base, next rung +-135); if-false: even one rung of widening off a command-trained parent degrades gait, and omnidirectional needs from-scratch or mirror-symmetry, not warm-starts.

**gate**: DR0 det+sto 6/6 with resampled +-90deg commands: gait_valid 12/12, zero terminations, no sacrificed leg, lateral tracking err <= 2x forward err

**verdict**: PASS. Heading ladder rung 1 (±90°): own-cfg DR0 gv 12/12, 0 term, 0 sacrificed legs, clean alternating tripod (duty ~0.6/0.4); lateral tracking err ≤1.6x forward (gate ≤2x); JOYSTICK GATE PASS at DR0.2 (0 falls incl. flip stress). Pathologies to watch: prog median 0.84 (parent wander30 0.94-1.02 at ±45 — lateral segments cost progress) and left-strafe covers half the distance of right (0.147 vs 0.274 m, L/R asymmetry — mirror-symmetry line target). Next rung ±135.

