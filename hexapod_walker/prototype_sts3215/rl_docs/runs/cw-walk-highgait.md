# cw-walk-highgait

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:37:14+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-anchorgate

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST: walk in a HIGHER stance. Height ref +20 mm during walk (goal.walk_height_off_mm=20, tracked by the existing height kernel). If-true: gait_valid walking at +20 mm with tracking comparable to nominal; if-false: raised stance destabilizes the gait (knees run out of travel or slip rises).

**gate**: DR0 det+sto 6/6 at +20mm: gait_valid, zero terminations, mean height err <= 8 mm

**verdict**: FAIL. +20mm stance ref largely ignored: det end-height err med 20.4mm (gate ≤8) ≈ the full offset; gait itself unharmed (gv 12/12, 0 term, slip/m med 1.29 ≈ champ). Not the destabilization branch — the height reference channel is too weak to move the stance (lineage-wide height sloppiness, cf. ruling 1 rework). Stance-height arms should wait for the height-economy rework, not more steps.

**refused_reason**: hexapod-mjx-train-7 already runs cw-walk-highgait — GPU pods host exactly one run; pick a free GPU pod.

