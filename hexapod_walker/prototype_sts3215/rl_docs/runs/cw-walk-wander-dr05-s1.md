# cw-walk-wander-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:54:28+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-wander

**wandb_id**: yg1un0ql

**hypothesis**: Seed concordance for the steering-DR champion (operator ruling 7: promotion needs multi-seed panels; wander-dr10 just FAILED the DR1.0 rung, so wander-dr05 is the driving line's robustness ceiling and must not rest on one seed). Identical config to cw-walk-wander-dr05 (parent cw-walk-wander, DR 0.5, +/-45deg 5s resample, 15% stops), ONE variable: seed 1 instead of 0. If-true: seed 1 lands the same gate (own-cfg DR0.5 gv 12/12, 0 term, prog med >=0.85, slip/m med <=2.4) - the DR0.5 steering rung is seed-robust and promotable. If-false: seed 1 misses the gate - the wander-dr05 PASS was seed luck and the steering-DR rung is unproven. Strongest alternative: partial concordance (passes gv but worse slip) - report the corner, keep dr05 as champion with the caveat.

**gate**: Own-cfg DR0.5 6+6 resampled commands (same as wander-dr05): gv 12/12, 0 term, prog_ratio med >=0.85, slip/m med <=2.4; DR0 det retention gv 6/6; frames watched det

