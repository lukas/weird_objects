# cw-walk-head90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:06:42+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-wander30

**wandb_id**: wymqwc2j

**hypothesis**: Heading LADDER rung 1 (after cw-walk-backforth 0/12 FAIL confirmed abrupt +-180 widening breaks the gait, matching steer-explore/fdiag — binding review 9 said widen gradually, not abruptly). The robot already walks and changes direction within +-45deg (wander30); this run widens commands to +-90deg warm-started FROM wander30 itself, one rung not four. If-true: gait_valid tracking at +-90 (strafe-capable base, next rung +-135); if-false: even one rung of widening off a command-trained parent degrades gait, and omnidirectional needs from-scratch or mirror-symmetry, not warm-starts.

**gate**: DR0 det+sto 6/6 with resampled +-90deg commands: gait_valid 12/12, zero terminations, no sacrificed leg, lateral tracking err <= 2x forward err

