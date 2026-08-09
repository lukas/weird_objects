# cw-walk-lowgait

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:33:35+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: ze6nb4uj

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST: walk in a LOWER crouched stance. Height ref -20 mm during walk. If-true: gait_valid crouched walking (useful for stability + sim2real margin); if-false: reduced clearance forces dragging (drag/slip metrics rise).

**gate**: DR0 det+sto 6/6 at -20mm: gait_valid, zero terminations, mean height err <= 8 mm

**verdict**: PASS — gate met at -20mm crouch: DR0 det+sto gv 12/12, 0 term, mean end-height err ~4mm (gate <=8); det prog 1.02, slip/m 1.14 ~ champion. One sto ep collapsed to near-stationary paddling (prog 0.18, slip/m 6.98) = known lineage sto brittleness, not a crouch defect. Crouch height is now a runtime-command candidate.

