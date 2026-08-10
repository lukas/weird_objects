# cw-dep-vref1-r1-legmass

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T16:57:29+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly if each leg's physical mass/length is a bit off from the CAD ideal, the way real 3D-printed/assembled legs always are. vref1-r1 has never tested per-leg manufacturing tolerance (leg_mass_jitter_pct, link_len_leg_pct) in isolation -- distinct from the already-PASSed chassis-level payload/comshift axes, which move the WHOLE body's mass, not per-leg asymmetry. Elevates dr.leg_mass_jitter_pct 0.10->0.20 and dr.link_len_leg_pct 0.012->0.025 (2x) on the contract-exact checkpoint. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- per-leg build variance composes free like the other axes. If-false: asymmetric leg geometry/mass breaks the gait -- flags a real hardware print-tolerance risk to check before attempt #2. Per P0 rule 3, k_current=0.

**gate**: Own-cfg (DR0.35+leg_mass_jitter_pct=0.20+link_len_leg_pct=0.025) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 baseline retention clean; frames watched det

