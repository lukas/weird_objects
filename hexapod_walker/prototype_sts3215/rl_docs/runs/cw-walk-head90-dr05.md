# cw-walk-head90-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:13:50+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-head90

**wandb_id**: zajjrji4

**hardware_ready**: no

**hypothesis**: DR-robustness rung for the widest passed steering envelope: head90 (±90° resampled cmds) PASSED at DR0 + joystick gate at DR0.2; wander-dr05 and strafe-dr05 both showed DR0.5 hardening works on driving lines without gait loss. One variable off head90: --no-dr → --dr-scale 0.5. If-true: ±90 driving is robust to physics variation (deployment-relevant envelope; DR1.0 rung next). If-false: the heading-wide policy is more DR-fragile than narrow/fixed-command ones — widen-then-harden ordering is wrong, harden first.

**gate**: own-DR0.5 det+sto 6/6: gait_valid 12/12, zero terminations, no sacrificed leg, prog_ratio median >= 0.75; DR0 retention det 6/6: gv 6/6, prog within 0.1 of parent (0.84); frames watched det

**verdict**: PASS. DR0.5 rung on the ±90 resampled-command envelope: own-DR0.5 det+sto gv 12/12, 0 term, no sacrificed legs, prog med 0.83 det / 0.90 sto (gate ≥0.75), slip/m med 1.83/1.66 (steering-DR lineage band); DR0 retention det gv 6/6, prog med 0.84 = parent head90 (gate: within 0.1). Frames det (own-DR): level six-leg cycling, no flag leg. Widen-then-harden ordering CONFIRMED: ±90 envelope and DR0.5 compose. DR1.0 rung NOT queued (full-DR refuted twice: wander-dr10 c55, longdist-dr10 c53). Not hardware-ready (paddle slip root, contact pricing).

